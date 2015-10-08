#include "stm8s_conf.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "string.h"
#include "nrf24l01.h"
#include "nrf24l01_port.h"
#include "rf-protocol.h"
#include "hal_delay.h"
#include "steuerung.h"
#include "position.h"
#include "analog.h"
#include "sync_communication.h"

/***********************************
函数功能：UART IO口初始化
输入参数：无
输出参数：无
备   注：ＩＯ在输出模式下，可通过ＣＲ２寄存器
         控制输出速率
***********************************/

#define RX_FRAME_MAX  40
#define TX_FRAME_MAX  40
#define RX_TIMER_TIMEOUT    5   
#define	SYNC_TIMEOUT			5	//	2 * 50 ms = 50ms Interval
#define	SERVICE_STANDBY_TIMEOUT	100	//	ca. 1000ms Wartezeit...
#define	SYNC_PACKET_TIME		3	//	ein Packet brauch ca. 30 ms...
#define SYNC_COM_TIMEOUT		2	//	20 ms
#define SYNC_RAND_TIME_MASK		0x3	//	0 bis 30 ms zuf鋖lige Wartezeit wenn out-of-sync...

volatile unsigned char syncComWaitTimer = 0;
volatile unsigned char syncComTimeoutTimer;

int16_t pulseCounterSync[NUM_DRIVES];
#ifdef USE_TIMECOUNT
unsigned int timeCounterSync[NUM_DRIVES];
unsigned char syncPulseTime[NUM_DRIVES];
#endif
uint32_t syncKeysReceived;
uint32_t syncBtKeysReceived;
signed int syncBtpulseCounter[NUM_DRIVES];
uint8_t syncBtpulseIndex = 0;
//unsigned char syncBtKeysReceivedCounter[2] = {0,0};
uint32_t syncKeysTransmitted;
signed char syncActive;
signed char btActive;
signed char serviceActive = 0;
/**
 * This variable is used to hold the feedback value that will be transmitted over MFP (and any Bluetooth connection).
 */
uint32_t	ledCode;
unsigned char myControlUnitIndex = 1;

uint8_t	myAddr;
#if (MYADDR_SIZE > 8)
#error type of myAddr to small!
#endif

#define	SYNC_INACTIVE		0
#define	SYNC_ACTIVE			15	//	gibt an wie oft eine Antwort ausbleiden kann bis der Sync-Betrieb abgeschaltet wird...

//unsigned long syncLEDs;
unsigned char syncStatus;
unsigned char syncRecStatus;

//volatile enum usageMode_t syncUsageMode = unknown;




uint8_t symcBtKeysReceivedIndex;

uint8_t TxFrame[TX_FRAME_MAX];
volatile unsigned char TxIndex = 0;
volatile unsigned char RxIndex = 0;
union SyncCommunicationData_t
{
	struct
	{
		uint8_t length;
		uint8_t type;
		uint8_t data[40];
		uint8_t checksum;
	};
	struct
	{
		unsigned char length;
		unsigned char type;
		unsigned long keys;
		int16_t pulseCounter[NUM_DRIVES];
		}bleAnglePacket;
	struct
	{
		uint8_t length;
		uint8_t type;
		uint32_t keys;
		uint8_t ledData[5];
		uint8_t UBB							:	1;
		uint8_t stopAll						:	1;
		uint8_t automaticMovementIsActive	:	1;
#ifdef LP_STYLE_FUNCTIONALITY
		uint8_t usageMode					:	1;
		uint8_t reserved					:	(4 - MYADDR_SIZE);
#else
		uint8_t sync						:	1;
		uint8_t lock						:	1;
		uint8_t angleAdj                    :   1;
		uint8_t reserved					:	(2 - MYADDR_SIZE);
#endif
		uint8_t	addr						:	MYADDR_SIZE;
#ifdef ELEGANCE_T
		uint8_t massageProgramm;
		uint8_t massageIntensity;
		uint8_t massagePeriode;
		uint8_t massageStatus;
#else
		uint8_t massage_status[2];
#endif
		uint32_t massageTimer;

		int16_t pulseCounter[NUM_DRIVES];
                
		int16_t current[NUM_DRIVES];
//		unsigned char controlUnitIndex;
#ifdef USE_TIMECOUNT
		unsigned int timer[NUM_DRIVES];
		unsigned char pulseTime[NUM_DRIVES];
#error pulseTime pr黤en...
#endif


#ifdef HAS_WAVE_BUTTON
		uint8_t waveStatus;
		int16_t waveValue;
#endif
		uint8_t checkSum;
	}syncPacket;
	struct
	{
		uint8_t _length_2;
		uint8_t _type_2;
		uint8_t serviceType;
		int16_t memoryPosition[4][2];
		uint8_t selfTestSetupData;
		uint8_t _checkSum_2;
	};
	
	uint8_t rawData[1];	//	[0] is not allowed by IAR-compilers, the whole packet is accessible with [1] too.
} syncFrame;

/**
 * Convert an u16_t from ctrlbox- to sync byte order.
 *
 * @param n u16_t in host byte order
 * @return n in network byte order
 */
int
sync_htons(int n)
{
  return ((n & 0xff) << 8) | ((n & 0xff00) >> 8);
}

void uart1TrvCompleteCallBack(void)
{
   TxIndex ++;
  if(TxIndex < TX_FRAME_MAX)
  {
    UART1->DR = TxFrame[TxIndex];
  }
  else
  {
    /* Clear the Transmitter Disable bit */
        UART1->CR2 &= (uint8_t)(~UART1_CR2_TEN); 
        /* Set the Receiver Enable bit */
        UART1->CR2 |= (uint8_t)UART1_CR2_REN; 
  }

}

void uart1ReceiveCallback(uint8_t data)
{
  if(RxIndex < RX_FRAME_MAX)
  {
    syncFrame.rawData[RxIndex] = data;
    RxIndex++;
  }
  syncComTimeoutTimer = SYNC_COM_TIMEOUT;
}

void UART_IOConfig(void)
{ 
    TXPort->DDR |= TXPin;//输出模式
    TXPort->CR1 |= TXPin;//推挽输出   
    
    RXPort->DDR &=~RXPin;//输入模式
    RXPort->CR1 &=~RXPin;//浮空输入
}
void uart1InitTx(u8 SYS_Clk, u32 baud)
{   
    u16 UART_Temp;
    UART1->CR2 = 0;// 禁止UART发送和接收
    UART1->CR1 = 0;// b5 = 0,允许UART  b2 = 0,禁止校验
                                                      
    UART1->CR3 = 0x0;// b5,b4 = 00,1个停止位

    UART1->CR5 |= UART1_CR5_HDSEL;  /**< UART1 Half Duplex Enable  */
                           
    
    UART_Temp = SYS_Clk*1000000/baud;
    
    UART1->BRR2 = (u8)((UART_Temp&0x000F)|((UART_Temp&0xF000)>>8));
    UART1->BRR1 = (u8)((UART_Temp&0x0FF0)>>4);
                                                 
                                    
    UART1->CR2 = 0x28; // b3 = 1,允许发送
                       // b2 = 1,允许接收
                       // b5 = 1,允许产生接收中断 
                       // b6 = 1,发送完成产生中断

}
void uart1InitRx(u8 SYS_Clk, u32 baud)
{   
    u16 UART_Temp;
 // uint32_t BaudRate_Mantissa = 0, BaudRate_Mantissa100 = 0;
  //  UART_IOConfig();//UART IO引脚初始化 //单线通信模式下，引脚不需要配置
    
 //   UART1->BRR2 = UART1_BRR2_RESET_VALUE;  /* Set UART1_BRR2 to reset value 0x00 */
 //   UART1->BRR1 = UART1_BRR1_RESET_VALUE;  /* Set UART1_BRR1 to reset value 0x00 */

 //   UART1->CR1 = UART1_CR1_RESET_VALUE;  /* Set UART1_CR1 to reset value 0x00 */
  //  UART1->CR2 = UART1_CR2_RESET_VALUE;  /* Set UART1_CR2 to reset value 0x00 */
  //  UART1->CR3 = UART1_CR3_RESET_VALUE;  /* Set UART1_CR3 to reset value 0x00 */
  //  UART1->CR4 = UART1_CR4_RESET_VALUE;  /* Set UART1_CR4 to reset value 0x00 */
  //  UART1->CR5 = UART1_CR5_RESET_VALUE;  /* Set UART1_CR5 to reset value 0x00 */

  //  UART1->GTR = UART1_GTR_RESET_VALUE;
   // UART1->PSCR = UART1_PSCR_RESET_VALUE;
    
    UART1->CR2 = 0;// 禁止UART发送和接收
    UART1->CR1 = 0;// b5 = 0,允许UART  b2 = 0,禁止校验
                                                      
    UART1->CR3 = 0x0;// b5,b4 = 00,1个停止位

    //UART1->CR5 &= ~(UART1_CR5_SCEN | UART1_CR5_IREN);
    UART1->CR5 |= UART1_CR5_HDSEL;  /**< UART1 Half Duplex Enable  */
                            
/************************************************** 
    设置波特率，必须注意以下几点：
    (1) 必须先写BRR2
    (2) BRR1存放的是分频系数的第11位到第4位，
    (3) BRR2存放的是分频系数的第15位到第12位，和第3位到第0位
    例如对于波特率位9600时，分频系数=2000000/9600=208
    对应的十六进制数为00D0，BBR1=0D,BBR2=00
*************************************************/ 
    /* Clear the LSB mantissa of UART1DIV  */
  //  UART1->BRR1 &= (uint8_t)(~UART1_BRR1_DIVM);  
    /* Clear the MSB mantissa of UART1DIV  */
  //  UART1->BRR2 &= (uint8_t)(~UART1_BRR2_DIVM);  
    /* Clear the Fraction bits of UART1DIV */
  //  UART1->BRR2 &= (uint8_t)(~UART1_BRR2_DIVF);  
    
    /* Set the UART1 BaudRates in BRR1 and BRR2 registers according to UART1_BaudRate value */
  //  BaudRate_Mantissa    = ((uint32_t)SYS_Clk*1000000 / (baud << 4));
  //  BaudRate_Mantissa100 = (((uint32_t)SYS_Clk*1000000 * 100) / (baud << 4));
    /* Set the fraction of UART1DIV  */
  //  UART1->BRR2 |= (uint8_t)((uint8_t)(((BaudRate_Mantissa100 - (BaudRate_Mantissa * 100)) << 4) / 100) & (uint8_t)0x0F); 
    /* Set the MSB mantissa of UART1DIV  */
  //  UART1->BRR2 |= (uint8_t)((BaudRate_Mantissa >> 4) & (uint8_t)0xF0); 
    /* Set the LSB mantissa of UART1DIV  */
  //  UART1->BRR1 |= (uint8_t)BaudRate_Mantissa;   
    
    UART_Temp = SYS_Clk*1000000/baud;
    
    UART1->BRR2 = (u8)((UART_Temp&0x000F)|((UART_Temp&0xF000)>>8));
    UART1->BRR1 = (u8)((UART_Temp&0x0FF0)>>4);
                                                 
                                    
    UART1->CR2 = 0x24; // b3 = 1,允许发送
                       // b2 = 1,允许接收
                       // b5 = 1,允许产生接收中断 
                       // b6 = 1,发送完成产生中断

}
/**************************************
函数功能：从UART3发送一个字符
输入参数：ch -- 要发送的字符
输出参数：无
备    注：无
***************************************/
uint8_t UART_SendStringStart(const unsigned char *string, uint8_t len)
{
  uint8_t i;
    if(TxIndex != TX_FRAME_MAX || len == 0)
      return 0;
    TxIndex = TX_FRAME_MAX - len;
    for(i = 0;i < len;i++)
      TxFrame[TxIndex + i] = string[i];
    UART1->DR = TxFrame[TxIndex];
    return len;
}

void UART_SendChar(unsigned char ch)
{
    
     UART1->DR = ch;                     // 将要发送的字符送到数据寄存器  
     while((UART1->SR&0X40)==0);   
}
void UART_SendString(unsigned char *string, uint8_t len)
{
  uint8_t i;
  for(i = 0;i < len;i++)
    UART_SendChar(string[i]);
}
unsigned char syncCalcCheckSum()
{
	uint16_t i;
	uint8_t sum;
	sum = 0xff;
	for (i = 0; i < ((uint16_t)2 + (uint16_t)syncFrame.length); i++)
	{
		sum -= syncFrame.rawData[i];
	}
	
	return sum;
}
void syncInit(void)
{
  uart1InitRx(16,38400);
}

#ifndef __ICCAVR__
uint8_t syncNewFrame = -1;
#endif

void syncSendData(unsigned long* RfButtons)
{
  static uint32_t lastKeys = 0;
  uint32_t syncNewKeys;
	
  if (serviceActive != 0)
  {
    if (syncFrame.length == 0) return;

/*		syncFrame.type = 0x80;
		syncFrame.length = 1 + sizeof(ee_memoryPosition) + 1;
		syncFrame.data[0] = 0x01;	// service: Steuerung Memory-Positionen...

		ee2memcpy(syncFrame.data + 1, ee_memoryPosition, sizeof(ee_memoryPosition));

		syncFrame.data[1 + sizeof(ee_memoryPosition)] = ee_read_value(&ee_selfTestSetup.data);

		syncFrame._checkSum_2 = syncCalcCheckSum();*/
 
  } else
  {
    syncFrame.syncPacket.length			=	sizeof(syncFrame.syncPacket) - 3;	//4 + 4 + 1 + 1 + sizeof(syncFrame.pulseCounter);
    syncFrame.syncPacket.type			=	0x07;

    syncNewKeys = *RfButtons;//getRFbuttons();
    syncFrame.syncPacket.automaticMovementIsActive	= (((syncKeysTransmitted != 0) || (syncNewKeys == 0)) && ((getMemoryActive() != 0) || (allFlatTimer != 0))) ? 1 : 0;

    syncKeysTransmitted					=	syncNewKeys;
    syncFrame.syncPacket.keys			=	syncKeysTransmitted | lastKeys;
    syncNewKeys = 0;
    if	(lastKeys == syncKeysTransmitted)
    {
      lastKeys = 0;
    } else
    {
      lastKeys = syncKeysTransmitted;
    };
    if (syncKeysTransmitted != 0)
    {
      myControlUnitIndex = 1;
    };
		

     doSteuerung(RfButtons);
		
    memcpy(syncFrame.syncPacket.ledData, &ledCode, 4);
#ifdef ELEGANCE_T
    syncFrame.syncPacket.ledData[4]		=	syncStatus == 0xFF ? syncRecStatus : syncStatus;
#else
    syncFrame.syncPacket.ledData[4]		=	syncStatus;
#endif
    syncFrame.syncPacket.UBB				=	UBB_get();
    syncFrame.syncPacket.stopAll			=	stopAll;

#ifdef LP_STYLE_FUNCTIONALITY
    syncFrame.syncPacket.usageMode			=	ee_read_value(&ee_usageMode) == pressAndRelease ? 0 : 1;
#endif
    syncFrame.syncPacket.reserved			=	0;
    syncFrame.syncPacket.angleAdj           =   1;
    syncFrame.syncPacket.addr				=	myAddr;
#ifdef ELEGANCE_T
    syncFrame.syncPacket.massageProgramm	=	massageProgramm;
    syncFrame.syncPacket.massageIntensity	=	massageIntensity;
    syncFrame.syncPacket.massagePeriode	=	massagePeriode;
    syncFrame.syncPacket.massageStatus		=	syncStatus;
#else
    syncFrame.syncPacket.massage_status[0]	=	0;
    syncFrame.syncPacket.massage_status[1]	=	0;
#endif
    syncFrame.syncPacket.massageTimer		=	0;
#ifdef HAS_WAVE_BUTTON
    syncFrame.syncPacket.waveStatus			=	waveStatus;
    syncFrame.syncPacket.waveValue			=	waveValue >> 1;
#endif
		//memcpy((void*)&syncFrame.massageTimer, (void*)&massageTimer, sizeof(syncFrame.massageTimer));
    __disable_interrupt();
    syncFrame.syncPacket.pulseCounter[0] = sync_htons(pulseCounter[0]);
    syncFrame.syncPacket.pulseCounter[1] = sync_htons(pulseCounter[1]);
    syncFrame.syncPacket.current[0] = sync_htons(current[0]);
    syncFrame.syncPacket.current[1] = sync_htons(current[1]);
   // syncFrame.syncPacket.current[2] = sync_htons(current[2]);
   // memcpy(syncFrame.syncPacket.pulseCounter, pulseCounter, sizeof(syncFrame.syncPacket.pulseCounter));
   // memcpy(syncFrame.syncPacket.current, current, sizeof(syncFrame.syncPacket.current));
#ifdef USE_TIMECOUNT
    memcpy(syncFrame.syncPacket.timer, timeCounter, sizeof(syncFrame.syncPacket.timer));
    memcpy(syncFrame.syncPacket.pulseTime, pulseTime, sizeof(syncFrame.syncPacket.pulseTime));
#error pulseTime pr黤en...
#endif
#ifndef RAVEN
  //  memcpy(&syncFrame.syncPacket.analog, &analog, sizeof(syncFrame.syncPacket.analog));
   // syncFrame.syncPacket.seftTestSignals.data	=	SELF_TEST_SIGNALS;
#endif
    __enable_interrupt();
	//	syncFrame.syncPacket.controlUnitIndex	=	myControlUnitIndex == 0 ? 1 : 0;
//		syncFrame.syncPacket.checkSum			=	syncCalcCheckSum();
  }
  syncFrame.data[syncFrame.length] = syncCalcCheckSum();
	
	//syncDataIndex = 0;
	
#ifdef __ICCAVR__
  UCSR0B	=	1	<<	TXEN0;
#else
  syncNewFrame = -1;
 if(symcBtKeysReceivedIndex == 0)
  {
   // TXPort->DDR |= TXPin;//输出模式
   // TXPort->CR1 &= ~TXPin;//推挽输出 
   // delay_ms(1);
    uart1InitTx(16,38400);
    UART_SendString((unsigned char*)&syncFrame,syncFrame.length+3);
   // UART_SendStringStart((unsigned char*)&syncFrame,syncFrame.length+3);
   // delay_ms(1);
  //  TXPort->DDR |= TXPin;//输出模式
   // TXPort->CR1 &= ~TXPin;//推挽输出  
  uart1InitRx(16,38400);
    
  }
    
	//mfpResetInBuffer();

#endif
}
/*
void checkSyncBtKeysReceivedCounter()
{
	unsigned char i;
	for (i = 0; i < sizeof(syncBtKeysReceivedCounter); i++)
	{
		if (syncBtKeysReceivedCounter[i] != 0)
		{
			syncBtKeysReceivedCounter[i]--;
		} else
		{
			syncBtKeysReceived[i] = 0;
		};
	};
}*/
void syncCom10msTimer(void)
{
  if (syncComWaitTimer != 0)
  {
  syncComWaitTimer--;
  };
	
  if (syncComTimeoutTimer != 0)
    syncComTimeoutTimer--;        
}
const uint8_t data[] = "Hello World!\r\n";
void syncComDo(unsigned long* RfButtons)
{

  
  if(RxIndex > 0 && syncComTimeoutTimer == 0)
    RxIndex = 0;
  if(RxIndex > 0 && RxIndex >= (syncFrame.length + 3))
  {
    			if (syncFrame.data[syncFrame.length] == syncCalcCheckSum())
			{
#ifndef NO_SYNC
				if ((syncFrame.type & 0x05) == 0x01)
				{
					//if (symcBtKeysReceivedIndex < sizeof(syncBtKeysReceivedCounter))
					{
						//syncBtKeysReceivedCounter[0] = 10;
						syncBtKeysReceived = syncFrame.syncPacket.keys;
						if(syncBtKeysReceived == ANGLE_POSITION_V2)
						{
							//syncBtpulseIndex = symcBtKeysReceivedIndex;
							syncBtpulseCounter[0] = sync_htons(syncFrame.bleAnglePacket.pulseCounter[0]);
							syncBtpulseCounter[1] = sync_htons(syncFrame.bleAnglePacket.pulseCounter[1]);
						}
						//symcBtKeysReceivedIndex++;
						btActive = SYNC_ACTIVE;
					};
				}
				if ((syncFrame.type & 0x05) == 0x05)
				{
					//symcBtKeysReceivedIndex = 0;
					
					syncKeysReceived	=	syncFrame.syncPacket.keys & (~(MASSAGE_BUTTONS | UBB_BUTTON));	// ohne UBB, ohne MASSAGE
//					myControlUnitIndex	=	syncFrame.syncPacket.controlUnitIndex;
					if (syncFrame.syncPacket.keys != 0)
						myControlUnitIndex = 0;

					if (syncFrame.syncPacket.addr == myAddr)
					{
						myAddr++;
					};
					myAddr &= MYADDR_MASK;

					if (myControlUnitIndex == 0)
					{
//					if (syncFrame.syncPacket.keys & 0x00020000)
//					{
#ifdef RF_TOUCH
						syncModeActive = syncFrame.syncPacket.sync;
						safetyChild    = syncFrame.syncPacket.lock;
						
						if(!UBB && syncFrame.syncPacket.UBB)
						{
							ubbTimeOutTimer = UBB_TIMEOUT;
							ubb_flag = 1;
						}
#endif
						UBB_set(syncFrame.syncPacket.UBB);
//					}
						
//					if ((syncFrame.keys & MASSAGE_BUTTONS) != 0u)
//					{
#ifdef ELEGANCE_T
						strgSyncMassage(syncFrame.syncPacket.massageProgramm, syncFrame.syncPacket.massageIntensity, syncFrame.syncPacket.massagePeriode);
						syncRecStatus = syncFrame.syncPacket.massageStatus;
#else
						//massage_status[0]	=	syncFrame.syncPacket.massage_status[0];
						//massage_status[1]	=	syncFrame.syncPacket.massage_status[1];
#endif
						//massageTimer		=	syncFrame.syncPacket.massageTimer;
#ifdef HAS_WAVE_BUTTON
						if (syncFrame.length >= (sizeof(syncFrame.syncPacket.waveValue) + (int)&syncFrame.syncPacket.waveValue - (int)syncFrame.data))
						{
							BUILD_BUG_ON((int)&syncFrame.syncPacket.waveValue < (int)&syncFrame.syncPacket.waveStatus);	// snycFrame.syncPacket.waveValue must be placed behind the other members - otherwise the previous if-statement does not work.
							waveStatus			=	syncFrame.syncPacket.waveStatus;
							waveValue			=	syncFrame.syncPacket.waveValue << 1;
							waveValue |= 1;
						}
#endif
					}
					if ((syncFrame.syncPacket.keys & (MASSAGE_BUTTONS | UBB_BUTTON)) != 0u)
					{
						allFlatTimer = 0;
						stopMemory();
					};
					
					if (syncFrame.syncPacket.stopAll != 0)
						stopAll = active;
					
#ifdef LP_STYLE_FUNCTIONALITY
					syncUsageMode = (syncFrame.syncPacket.usageMode == 0) ? pressAndRelease : pressAndHold;
#endif

					automaticMovementIsActive = syncFrame.syncPacket.automaticMovementIsActive;
					
                                       pulseCounterSync[0]  = sync_htons(syncFrame.syncPacket.pulseCounter[0]);
                                       pulseCounterSync[1] = sync_htons( syncFrame.syncPacket.pulseCounter[1]);
					//memcpy(pulseCounterSync, syncFrame.syncPacket.pulseCounter, sizeof(pulseCounterSync));
#ifdef USE_TIMECOUNT
					memcpy(timeCounterSync, syncFrame.syncPacket.timer, sizeof(timeCounterSync));
					memcpy(syncPulseTime, syncFrame.syncPacket.pulseTime, sizeof(syncPulseTime));
#endif
					syncActive = SYNC_ACTIVE;
					serviceActive = 0;
#endif
					syncComWaitTimer	=	SYNC_TIMEOUT;
#ifndef NO_SYNC
				}
#endif
    }
    RxIndex = 0;
  }
  
  if(RxIndex == 0 && syncComWaitTimer == 0)
  {
    FlushRfModule();
    syncSendData(RfButtons);
			
   // checkSyncBtKeysReceivedCounter();
	symcBtKeysReceivedIndex = 0;
			
			if (syncActive != 0)
				syncActive--;
			if (btActive != 0)
				btActive--;
			if (serviceActive != 0)
				serviceActive--;
			
	 syncComWaitTimer	=	SYNC_PACKET_TIME + (SYNC_TIMEOUT + SYNC_TIMEOUT)+ (rand() & SYNC_RAND_TIME_MASK);
  }
}