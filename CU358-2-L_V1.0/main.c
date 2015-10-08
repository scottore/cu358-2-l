/*This is the main file*/
#include "stm8s_conf.h"
#include "hal_delay.h"
#include "misc.h"
#include "sync_communication.h"
#include "position.h"
#include "relais.h"
#include "relais_port.h"
#include "steuerung.h"
#include "nrf24l01.h"
#include "nrf24l01_port.h"
#include "rf-protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define SYS_CLOCK   16
#define TIM1_PRESCALER_16  15
#define STORE_MOVE_TIMEOUT	250	//	ca. 2,5 Sekunden
// these variables are used by the nrf24l01 library...
const unsigned char MODUL_ID = 0x00;
const unsigned char VERSION_ID = 0x02;
const unsigned char REVISION_ID = 0x00;
volatile unsigned char flag_10ms = 0;
volatile unsigned int seed;
/*
void timer1_freetimer_setup(void)
{
  TIM1_DeInit();
  TIM1_TimeBaseInit(TIM1_PRESCALER_16,TIM1_COUNTERMODE_DOWN, 10000-1, 0x00);//Time1:1MHz,10ms定时
  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
  TIM1_Cmd(ENABLE);
}
*/
//设置定时器4为1ms中断
void timer4_freetime_config(void)
{
  /* Set the Prescaler value */
  TIM4->PSCR = (uint8_t)(TIM4_PRESCALER_128);
  /* Set the Autoreload value */
  TIM4->ARR = (uint8_t)(124);
  /* Enable the Interrupt sources */
  TIM4->IER |= (uint8_t)TIM4_IT_UPDATE;
  /* set the CEN Bit */
  TIM4->CR1 |= TIM4_CR1_CEN;
 // TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
 // TIM4_Cmd(ENABLE);

}

void gpio_init(void)
{
  /* Clear High speed internal clock prescaler */
  CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);

  /* Set High speed internal clock prescaler */
  CLK->CKDIVR |= (uint8_t)CLK_PRESCALER_HSIDIV1;
gpio_Oc1ptr_init();
  gpio_UBB_init();
  gpio_Ain0_init();
  gpio_Ain1_init();
  gpio_Ain2_init();
  gpio_M1up_init();
  gpio_M1down_init();
  gpio_M2up_init();
  gpio_M2down_init();
 // TXPort->DDR |= TXPin;//输出模式
 // TXPort->CR1 &= ~TXPin;//推挽输出   
    
}
void func_init(void)
{
  timer4_freetime_config();
  syncInit();
}
void sys_1ms_timeout(void)
{
  static unsigned char prescaler = 0;
  prescaler++;
    if (prescaler >= 10)
    {
      prescaler = 0;
      RF_Timer_10ms();
      
      if (flag_10ms < 10)
        flag_10ms++;
    //  UBB_toggle();
    }
    Relais_Position_Timer_1ms();
}
int main( void )
{
  uint8_t i;
  unsigned long RfButtons;
  uint8_t moveTimeout = STORE_MOVE_TIMEOUT;
  
  gpio_init();
    srand(seed);

  
  for(i = 0;i < 2;i++)
  {
    delay_ms(50);
    UBB_TOOGLE();
  }
  UBB_OFF();
  func_init();
  SteuerungFunkInit();
  Init_RF_Settings(TRANSCEIVER_OPERATION_MODE_RX);
  loadPulses();	//	after start-up we have to restore the pulse-counter from EEPROM.

  while(1)
  {
    i = (uint8_t)(TIM4->CNTR);
    seed++; 
    seed += i; 
      
    srand(seed);
    
    FunkAppl();		// call the nrf24l01 library to handle the radio communication

    RfButtons = getRFbuttons();
    
    syncComDo(&RfButtons);
    doSteuerung(&RfButtons);
    checkPulseReset();
    if (flag_10ms != 0)	//	do some timing stuff if the flag has been set by the SysTick_Handler().
    {
      flag_10ms--;
      syncCom10msTimer();
      strgTimer_10ms();
      relaisTimer_10ms();
      if ((driveState[0] == stop)
          && (driveState[1] == stop)
#if (NUM_DRIVES >= 3)
          && (driveState[2] == stop)
#endif
#if (NUM_DRIVES >= 4)
          && (driveState[3] == stop)
#endif
        )
      {
        if (moveTimeout != 0)
        {
          moveTimeout--;
          if (moveTimeout == 0)
          {
            storePulses();	//	store the pulse-counters to EERPOM if all actuators are stopped...
          };
        };
      } 
      else
      {
        moveTimeout = STORE_MOVE_TIMEOUT;
      };
    }
  }
}




#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif