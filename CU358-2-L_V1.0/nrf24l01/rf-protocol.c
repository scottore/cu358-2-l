
#ifdef __ICCAVR__
#include "..\toolbox\clock.h"
#include <ioavr.h>
#include <inavr.h>
#endif

//******************************************************************************
// Funk_Appl_Quad8Sync.c
//******************************************************************************
#define _ACK_MODE

#include "stm8s_conf.h"
#include "misc.h"
#include <string.h>

//#include "..\hal.h"
#include "eeprom.h"
#include "rf-protocol.h"
#include "nrf24l01.h"

#define IRQ_Ext	!nrf24l01_getIRQ()


//extern EEMEM unsigned char ee_RF_CH;					// eingestellter HF-Kanal
//extern EEMEM unsigned char ee_RF_CURRENT_PIPE;		// eingestellte Pipe
extern __eeprom struct Member ee_Member_Tab;

#ifdef __ICCAVR__
#define	shadow_ee_RF_CURRENT_ADDR	ee_RF_CURRENT_ADDR
#define	shadow_ee_Member_Tab		ee_Member_Tab
#else
extern unsigned char shadow_ee_RF_CURRENT_ADDR;			// eingestellte Funk-Adresse						// eingestellte Funk-Adresse
extern struct Member shadow_ee_Member_Tab;
#endif


// TODO durch Makros ersetzen...
extern const unsigned char MODUL_ID;
extern const unsigned char VERSION_ID;
extern const unsigned char REVISION_ID;


volatile signed char nrfTimerFlag10ms;                                           	// Timer Flag Register
volatile unsigned char RfTimer;
//volatile unsigned char TimeOut = 0;
#define FLUSH_24L01_TIMEOUT        50
volatile unsigned char Flush24L01RegTimer;
volatile unsigned int TeachTimer;
volatile unsigned char MasterTimeOut;

volatile unsigned char RequestFlag = 0;
unsigned int main_count;
struct Member member;
union Frame Tx_Frame, Rx_Frame;
union
{
	unsigned long result;
	unsigned char data[4];
} RfButtonResult;
unsigned char Rf_mode = RECEIVE;
unsigned char Rx_mode = NORMAL_MODE;
//unsigned char FifoStat = 0;
//unsigned char RfAddr[5];
//unsigned char DataArr[15];

unsigned long long switchCmd[4];
uint8_t ledDispTimeout = 10;

//******************************************************************************
// void Timer3(void)
//******************************************************************************
void RF_Timer_10ms(void)
{
	static unsigned char TimeCnt;
	
	nrfTimerFlag10ms = -1;
	
	if (RfTimer != 0)
		RfTimer--;
	
	if (dataReceivedTimer != 0)
		dataReceivedTimer--;
        if(Flush24L01RegTimer != 0)
          Flush24L01RegTimer --;
	if(TimeCnt >= 10)
	{
		if(TeachTimer)
			TeachTimer--;	
		TimeCnt = 0;
	}
          if(ledDispTimeout > 0)
            ledDispTimeout --;
        if(strgKeyTimer > 0)
          strgKeyTimer --;
   
	
	TimeCnt++;
}


//******************************************************************************
// void Init_Funk(void)
//******************************************************************************
void Init_Funk(void)
{
	CsnFunk_TransmitDeactivate();
	CeFunk_TransceiverDeactivate();
	
	Rf_mode = RECEIVE;

//	Transceiver_On();
 	Init_RF_Settings(Rf_mode);
 	SPI_RF_Init();
}

//******************************************************************************
// void Transmit_ID_Data(void)
//******************************************************************************
void Transmit_ID_Data(void)
{	
	Tx_Frame.command = RF_MODULID;
	Tx_Frame.length = 10;
	Tx_Frame.data[0] = MODUL_ID;
	Tx_Frame.data[1] = VERSION_ID;
	Tx_Frame.data[2] = REVISION_ID;
#ifdef __ATmega16__
	Tx_Frame.data[3] = 1;	//CONTROLLER_ID;
#elif defined(__ATmega88P__) || defined(__ATmega88__)
	Tx_Frame.data[3] = 5;	//CONTROLLER_ID;
#elif defined(__ATmega164P__)
	Tx_Frame.data[3] = 7;	//CONTROLLER_ID;
#elif defined(__ATmega324P__)
	Tx_Frame.data[3] = 0x0E;	//CONTROLLER_ID;
#elif defined(STM32F030K6T6)
	Tx_Frame.data[3] = 0x0F;	//CONTROLLER_ID;
#elif defined(STM32F030C6T6)
	Tx_Frame.data[3] = 0x10;	//CONTROLLER_ID;
#elif defined(STM8S207)
        Tx_Frame.data[3] = 0x11;	//CONTROLLER_ID;
#elif defined(STM8S003) || defined(STM8S103)
        Tx_Frame.data[3] = 0x10;	//CONTROLLER_ID;
        
#else
#error unbekannter Prozessor
#endif	
	Tx_Frame.data[4] = shadow_ee_Member_Tab.pipe;
#ifdef __ICCAVR__
	Tx_Frame.data[5] = ee_Member_Tab.addr_bytes[0];
	Tx_Frame.data[6] = ee_Member_Tab.addr_bytes[1];
	Tx_Frame.data[7] = ee_Member_Tab.addr_bytes[2];
	Tx_Frame.data[8] = ee_Member_Tab.addr_bytes[3];
	Tx_Frame.data[9] = ee_Member_Tab.addr_bytes[4];
#else
	memcpy(Tx_Frame.data + 5, shadow_ee_Member_Tab.addr_bytes, 5);
#endif
	SPI_Write_Buf(SPI_CMD_W_TX_PAYLOAD,SPI_REG_RX_PW_MAX,&Tx_Frame.daten[0]);
	Wait_Tx();

	Rf_mode = RECEIVE;
	Switch_Rf_mode(Rf_mode);
}

//******************************************************************************
// unsigned char Wait_Tx(void)
//******************************************************************************
unsigned char Wait_Tx(void)
{
	unsigned char temp;
	RfTimer = 4;
	while (RfTimer != 0)
	{
		if(IRQ_Ext)
		{
			temp = Ext_IRQ_Service();
			if((temp & SPI_REG_STATUS_TX_DS) != 0)
				return 1;
			if((temp & SPI_REG_STATUS_MAX_RT) != 0)
                        {
                           UBB_TOOGLE();
				return 0;
                        }
		}	
	}
	return 0;
}


//**********************************************************************************************************
// void StartRfTestMode(void)
//**********************************************************************************************************
void StartRfTestMode(void)
{
	Rx_mode = RF_TEST_MODE;
	Rf_mode = TRANSMIT;
	//Switch_Rf_mode(Rf_mode);
	//Change_RF_Channel(50);
}


////////////////////////////////////////////////////////////////////////////////
// void endRfTestMode(void)
////////////////////////////////////////////////////////////////////////////////
void endRfTestMode(void)
{
	Rf_mode = RECEIVE;
	Switch_Rf_mode(Rf_mode);
	Change_RF_Channel(shadow_ee_Member_Tab.channel);
	Rx_mode = NORMAL_MODE;
}


//******************************************************************************
// void FunkAppl(void)
//******************************************************************************
void FunkAppl(void)
{
	unsigned char i;
	unsigned char Rf_status = 0;
	unsigned char Req_Flag = 0;
	static unsigned char switchIndex;
	unsigned char FifoStat;
	unsigned char irq;

	main_count++;

	FifoStat = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_FIFO_STATUS);//1:RX FIFO 寄存器空;0: RX FIFO 寄存器非空
/*
	if(FifoStat & 0x02)
	{
//		asm("nop");
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_RX,0x00);	// RX-Fifo l鰏chen wenn die RX-Fifo voll ist...
	}
*/
	irq = IRQ_Ext;
	
	if(irq || ((FifoStat & 0x01) == 0))//中断产生 或 RX FIFO 寄存器非空标志
	{
		if(irq)
		{
			Rf_status = Ext_IRQ_Service();//清中断，读取中断标志
		}
		if (((Rf_status & SPI_REG_STATUS_RX_DR) == 0 )//中断未产生，且 RX FIFO 寄存器非空
		 && ((FifoStat & 0x01) == 0))
		{
			//SPI_Read_Buf(SPI_CMD_R_RX_PAYLOAD, SPI_REG_RX_PW_MAX, spiTransceiver.data);
			if (readRxPayload() == 0) FifoStat |= 0x01;//内部读取过一次，无效数据，清除本次RX FIFO 寄存器非空标志
		}
		if (((Rf_status & SPI_REG_STATUS_RX_DR) !=0 )
		 || ((FifoStat & 0x01) == 0))
		{
			//  GPIO_WriteReverse(LEDB_PORT, LEDB_PIN);
			Get_Data(&Rx_Frame.daten[0]);

			switch(Rx_Frame.command)
			{
			  case RF_MODULID:
				Rf_mode = TRANSMIT;
				Switch_Rf_mode(Rf_mode);//切换为发送模式
				if(Rx_mode == WAIT_FOR_REQ)
				{
					Set_Teach_Addr(0,255);
					Wait_Tx();
					Req_Flag = 1;
				}else
				{
					Set_Current_Addr();
					Wait_Tx();
				}
       
				Transmit_ID_Data();
                                Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
				break;
				
//			  case RF_DATA:
//				break;
				
			  case RF_BUTTON:
                              // UBB_TOOGLE();
//				RfButtonResult = (unsigned long)Rx_Frame.data[0] << 24;
//				RfButtonResult |= (unsigned long)Rx_Frame.data[1] << 16;
//				RfButtonResult |= (unsigned long)Rx_Frame.data[2] << 8;
//				RfButtonResult |= (unsigned long)Rx_Frame.data[3];
				RfButtonResult.data[3] = Rx_Frame.data[0];
				RfButtonResult.data[2] = Rx_Frame.data[1];
				RfButtonResult.data[1] = Rx_Frame.data[2];
				RfButtonResult.data[0] = Rx_Frame.data[3];
				RfTimer = RF_TIMEOUT;
                                 Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
				break;
				
			  case RF_WAKE_UP:
				Tx_Frame.command = RF_WAKE_UP;
				Tx_Frame.length = 0;
				SPI_Write_Buf(SPI_CMD_W_ACK_PAYLOAD + spiTransceiver.current_pipe, 0 + 3, Tx_Frame.daten);
                                 Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
				break;
				
			  case RF_CHANGE_ADDR:
				Rf_mode = CHANGE_RF_ADDR;
				member.channel = Rx_Frame.data[0];
				member.pipe = Rx_Frame.data[1];
				for(i=0;i<5;i++)
					member.addr_bytes[i] = Rx_Frame.data[i+2];
				TeachTimer = 100;
                                 Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
				break;
				
			  case RF_SET_CURRENT_ADDR:
				if((Rx_Frame.length == 1)&&(Rx_Frame.data[0]==1))
				{
#ifdef __ICCAVR__
					ee_Member_Tab[ee_RF_CURRENT_ADDR].channel = member.channel;
					ee_Member_Tab[ee_RF_CURRENT_ADDR].pipe = member.pipe;
					for(i=0;i<5;i++)
						ee_Member_Tab[ee_RF_CURRENT_ADDR].addr_bytes[i] = member.addr_bytes[i];
#else
					//rfCurrentAddrTemp = ee_shadow_read_value(ee_RF_CURRENT_ADDR);
					member.id = shadow_ee_Member_Tab.id;
					member.mode = shadow_ee_Member_Tab.mode;
					memcpy(&shadow_ee_Member_Tab, &member, sizeof(member));
					store_shadow_ee_Member_Tab();
					//ee_write_bytes(ee_Member_Tab + rfCurrentAddrTemp, &member, sizeof(member));
#endif
				}
				Change_RF_Channel(member.channel);
				Set_Current_Addr();
				Rx_mode = NORMAL_MODE;
				Rf_mode = NORMAL_MODE;
                                 UBB_OFF();
				TeachTimer = 0;
                                 Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
				break;
			}
		}else if(Rf_status == SPI_REG_STATUS_TX_DS)		// Tx-Ack empfangen
		{
			Rf_mode = RECEIVE;
                         Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
//			Switch_Rf_mode(Rf_mode);
		}else if(Rf_status == SPI_REG_STATUS_MAX_RT)	// Empfangsbest鋞igung (Ack) augeblieben
		{
			Rf_mode = RECEIVE;
                         Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
//			Switch_Rf_mode(Rf_mode);
		}
	}

////////////////////////////////////////////////////////////////////////////////

	if(Rx_mode != NORMAL_MODE)
	{
           Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
		MasterTimeOut = 5;
		switch(Rx_mode)
		{
		  case TEACH_MODE://主控盒，设置学习模式
			Set_Teach_Addr(0,255);
			TeachTimer = 200;
			Rx_mode = WAIT_FOR_REQ;
                         UBB_ON();
			SPI_Write_Buf(SPI_CMD_FLUSH_TX, 0, NULL);
			break;

		  case WAIT_FOR_REQ:
			if(Req_Flag && TeachTimer)
			{
				Req_Flag = 0;
                                
			}else
			{
				if(!TeachTimer)
				{
					Set_Current_Addr();
					Rx_mode = NORMAL_MODE;
                                         UBB_OFF();
				}
			}
                         if(ledDispTimeout == 0)
                          {
                           
                             
                             ledDispTimeout = 10;
                           }
			break;

		  case CHANGE_RF_ADDR:
			if(TeachTimer == 0)
				Rx_mode = NORMAL_MODE;
                        
			break;

		  case RF_TEST_MODE:
			SPI_Write_Buf(SPI_CMD_W_TX_PAYLOAD,SPI_REG_RX_PW_MAX,&Tx_Frame.daten[0]);
			RfTimer = 2;
			while (RfTimer != 0);
			break;			
		}
	}
//------------------------------------------------------------------------------
	if(nrfTimerFlag10ms != 0)
	{
		if (RfTimer == 0)
		{
			RfButtonResult.result = 0;
		};
		
		nrfTimerFlag10ms = 0;
	};
//------------------------------------------------------------------------------
	
	if ((getRFactive() == 0) || (getTeachMode() != 0))
	{
		memset(switchCmd, 0, sizeof(switchCmd));
	};
	
	if (switchIndex >= 4)
		switchIndex = 0;
	
	if ((switchCmd[switchIndex] != 0)
	 && ((SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_FIFO_STATUS) & 0x20) == 0))
	{
		// LEDs senden...		
		Tx_Frame.command = RF_LED_ON + switchIndex;
		Tx_Frame.length = 8;

		memcpy(Tx_Frame.data, switchCmd + switchIndex, 8);
			
		switchCmd[switchIndex] = 0;
		SPI_Write_Buf(SPI_CMD_W_ACK_PAYLOAD + spiTransceiver.current_pipe, 8 + 3, Tx_Frame.daten);
	};

	switchIndex++;
	// ApplFunk_Button(RfButtonResult.result);
	return;
}



void FlushRfModule(void)
{
  if( Rx_mode == NORMAL_MODE && Flush24L01RegTimer == 0)
  {
    Flush24L01RegTimer = FLUSH_24L01_TIMEOUT;
    Init_Funk();
           // UBB_toggle();
  }
}

unsigned long getRFbuttons()
{
	return RfButtonResult.result;
}

unsigned char getRFmode()
{
	return Rf_mode;
}

/*
void switchLEDs(unsigned long temp, unsigned char cmd)
{
#warning to do: Flusskontrolle implementieren!
	if (cmd <= 1)
		cmd ^= 1;
	// LEDs senden...		
	Tx_Frame.command = RF_LED_ON + cmd;
	Tx_Frame.length = 4;
	Tx_Frame.data[0] = temp;
	Tx_Frame.data[1] = temp >> 8;
	Tx_Frame.data[2] = temp >> 16;
	Tx_Frame.data[3] = temp >> 24;
	if(SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_FIFO_STATUS) & 0x20)
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_TX,0x00);
	SPI_Write_Buf(SPI_CMD_W_ACK_PAYLOAD + spiTransceiver.current_pipe, Tx_Frame.length + 3, Tx_Frame.daten);
}
*/

void switchLEDs(unsigned long long temp, unsigned char cmd)
{
	if (cmd <= 1)
		cmd ^= 1;
	
	if (cmd < 4)
		switchCmd[cmd] |= temp;
}

void overrideLEDsCmd(unsigned long long temp, unsigned char cmd)
{
	if (cmd <= 1)
		cmd ^= 1;
	
	if (cmd < 4)
		switchCmd[cmd] = temp;
}
	
void RF_PWR_Down()
{
	CeFunk_TransceiverDeactivate();
//	__delay_cycles(6000);
//	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,(spiTransceiver.config & 0xfd));
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x00);
}

void RF_PWR_Up()
{
	unsigned char stat;

	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x0f);
	delay_us(1500);	//__delay_cycles(6000);										//	ca. 1.5ms @ 4MHz
	stat = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_CONFIG);
	if(stat != 0x0f)
	{
		 Init_RF_Settings(Rf_mode);
 		 SPI_RF_Init();
	}
	CeFunk_TransceiverActivate();
}	

//#if defined(__ATmega88__) || defined(__ATmega88P__) || defined(__ATmega48__) || defined(__ATmega644P__) || defined(__ATmega164P__)
#ifdef CLKPCE

void RF_PWR_Up_lowPower()
{
	unsigned char stat;

	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x0f);
	
	//__delay_cycles(1500);										//	ca. 1.5ms @ 1MHz
	stat = getClkSettings();
	setClkDiv256();
	__delay_cycles(47-8);										//	ca. 1.5ms + overhead... - abz黦lich 8 Zyklen f黵 das 膎dern des Taktes
	restoreClkSettings(stat);
	
	stat = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_CONFIG);
	if(stat != 0x0f)
	{
		 Init_RF_Settings(Rf_mode);
 		 SPI_RF_Init();
	}
	CeFunk_TransceiverActivate();
}	

#endif

void rfEnterTestmode(unsigned char channel, unsigned char data)
{
	memset(spiTransceiver.data, data, sizeof(spiTransceiver.data));
	
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x02);
	delay_us(5000);	//__delay_cycles(40000);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x02);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,0x17);
	SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&spiTransceiver.data[0]);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_CH,channel);
	SPI_Write_Buf(SPI_CMD_W_TX_PAYLOAD,32,&spiTransceiver.data[0]);
	delay_us(5000);	//__delay_cycles(40000);
	CeFunk_TransceiverActivate();
	SPI_Write_Reg(SPI_CMD_REUSE_TX_PL,0x00);
}


void rfEnterTestmodeConstCarrier(unsigned char channel)
{
	memset(spiTransceiver.data, 0xFF, sizeof(spiTransceiver.data));
	
	CeFunk_TransceiverDeactivate();
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x02);		//	PWR_UP = 1
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,0x06);		//	ConstCarrier = 0, PLL_LOCK = 0, max gain
	delay_us(5000);	//__delay_cycles(40000);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,0x96);
	SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&spiTransceiver.data[0]);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_CH,channel);
	SPI_Write_Buf(SPI_CMD_W_TX_PAYLOAD,32,&spiTransceiver.data[0]);
	delay_us(5000);	//__delay_cycles(40000);
	CeFunk_TransceiverActivate();
}

void rfSendSingleTestPacket(unsigned char channel, unsigned char data)
{
	memset(spiTransceiver.data, data, sizeof(spiTransceiver.data));
	
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x02);
	delay_us(5000);	//__delay_cycles(40000);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x02);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x00);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,0x07);
	SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&spiTransceiver.data[0]);
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_CH,channel);
	SPI_Write_Buf(SPI_CMD_W_TX_PAYLOAD,32,&spiTransceiver.data[0]);
	delay_us(5000);	//__delay_cycles(40000);
	CeFunk_TransceiverActivate();
}
