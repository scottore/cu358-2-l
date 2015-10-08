//**********************************************************************************************************
// nrf24l01_lib.c
// Datum: 23.07.2008
// Ersteller: K.Gehrke
// portiert: S.Loley
//**********************************************************************************************************

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#include "hal_delay.h"

#include "nrf24l01.h"
#include "nrf24l01_port.h"
#include "eeprom.h"
//#define TEST_MODE
#define _ACK_MODE



volatile unsigned char dataReceivedTimer;
extern volatile unsigned int seed;
extern volatile unsigned char TimeOut;
//volatile unsigned char IRQ_Ext = 0;
struct TransceiverData spiTransceiver;
unsigned char val;
unsigned char valDat[5];
const unsigned char trans_base_addr[5][4] = {	{0x69,0x96,0x69,0x96},
												{0x7e,0x81,0x7e,0x81},
												{0x24,0x42,0x24,0x42},
												{0x3c,0xc3,0x3c,0xc3},
												{0xa5,0x5a,0xa5,0x5a}}; 		// 5*4 address bytes highLSB..MSB

const unsigned char trans_pipe0_addr[5] = {0x90,0x80,0x40,0xc0,0x50}; 	// LSB address byte tx and rx for each Addr(pipe0)

//EEMEM unsigned char ee_RF_CH;							// eingestellter HF-Kanal
 __eeprom unsigned char ee_RF_CURRENT_ADDR;					// eingestellte Funk-Adresse
//EEMEM unsigned char ee_RF_CURRENT_PIPE;				// eingestellte Pipe
 __eeprom struct Member ee_Member_Tab;

#ifdef __ICCAVR__
#define	shadow_ee_RF_CURRENT_ADDR	ee_RF_CURRENT_ADDR
#define	shadow_ee_Member_Tab		ee_Member_Tab
#else
unsigned char shadow_ee_RF_CURRENT_ADDR;				// eingestellte Funk-Adresse						// eingestellte Funk-Adresse
struct Member shadow_ee_Member_Tab;
#endif


#ifdef __ICCAVR__
//**********************************************************************************************************
// unsigned char SPI_Transmit(unsigned char ch)
// Diese Funktion sendet ein Byte 黚er die SPI-Schnittstelle und gibt das empfangene Byte zur點k.
//**********************************************************************************************************
unsigned char SPI_Transmit(unsigned char ch)
{	
	SPDR = ch;
#if defined(__ATmega164P__) || defined(__ATmega324P__)
	while(!(SPSR & (1<<SPIF0)));
#else
	while(!(SPSR & (1<<SPIF)));
#endif
	SPSR &= 0x7f;
	return SPDR;
}
#endif

//**********************************************************************************************************
// unsigned char SPI_Read(unsigned char ch)
// Diese Funktion 黚ertr鋑t ein Byte per Funk oder sendet ein Command an das Funkmodul.
//**********************************************************************************************************
/*
unsigned char SPI_Read(unsigned char ch)
{
	unsigned char ret;
	
	CsnFunk_TransmitActivate();
	SPI_Transmit(ch);
	ret = SPI_Transmit(0);
	CsnFunk_TransmitDeactivate();
	return ret;
}
*/
//**********************************************************************************************************
// unsigned char SPI_Write_Reg(unsigned char reg, unsigned char val)
// Diese Funktion schreibt den Wert (val) in das Register (reg) und liefert den Status zur點k.
//**********************************************************************************************************

unsigned char SPI_Write_Reg(unsigned char reg, unsigned char val)
{
	unsigned char ret;
	
	CsnFunk_TransmitActivate();
//	asm("nop");
	ret = SPI_Transmit(reg);
	SPI_Transmit(val);	
        asm("nop");
        asm("nop");
	CsnFunk_TransmitDeactivate();
	return ret;
}
//**********************************************************************************************************
// unsigned char SPI_Read_Reg(unsigned char reg)
// Diese Funktion liest das Register reg aus.
//**********************************************************************************************************
unsigned char SPI_Read_Reg(unsigned char reg)
{
	unsigned char ret;
	
	CsnFunk_TransmitActivate();
//	asm("nop");
	SPI_Transmit(reg);	
	ret = SPI_Transmit(0);	
        asm("nop");
        asm("nop");
	CsnFunk_TransmitDeactivate();
	return ret;
}

//**********************************************************************************************************
// unsigned char SPI_Write_Buf(unsigned char reg, unsigned char num, unsigned char *p_buf)
// Diese Funktion sendet die in einem Buffer gespeicherten Daten.
//**********************************************************************************************************
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char num, unsigned char *p_buf)
{
	unsigned char i,ret;
	
	CsnFunk_TransmitActivate();
//	asm("nop");
	ret = SPI_Transmit(reg);
	for(i=0;i<num;i++)
		SPI_Transmit(p_buf[i]);
        asm("nop");
        asm("nop");
	CsnFunk_TransmitDeactivate();
	if(reg == SPI_CMD_W_TX_PAYLOAD)
	{
		CeFunk_TransceiverActivate();
		delay_us(15);						// min. 10祍 warten
		CeFunk_TransceiverDeactivate();
	}
	return ret;
}
//**********************************************************************************************************
// unsigned char SPI_Read_Buf(unsigned char reg, unsigned char num, unsigned char *p_buf)
// Diese Funktion liest die angegebene Anzahl von Bytes aus dem Emfangs Buffer und speichert diese in die
// mit p_buf angegebene Datenstruktur.
//**********************************************************************************************************
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char num, unsigned char *p_buf)
{
	unsigned char i,ret;
	
	CsnFunk_TransmitActivate();
	ret = SPI_Transmit(reg);	
	for(i=0;i<num;i++)
		p_buf[i] = SPI_Transmit(0);
        asm("nop");
        asm("nop");
	CsnFunk_TransmitDeactivate();
	return ret;	
}
//**********************************************************************************************************
// void SPI_RF_Init(void)
// Diese Funktion initialisiert das Funkmodul.
//**********************************************************************************************************
#ifdef __ICCAVR__
#pragma optimize= z 3
#endif
////Enables Dynamic Payload Length;Enables Payload with ACK
//Enable dynamic payload length data pipe 0,1,2,3,4,5
//Max 32 bytes in RX payload in current data pipe
void SPI_RF_Init(void)
{
	unsigned char i;
	
	/////////////////////////////////////////////
	// initialisierung der SPI-Schnittstelle...
	
#if defined(__ATmega16__) || defined(__ATmega88P__) || defined(__ATmega88__)
	SPSR	|= 	(0<<SPI2X);		// Double speed
	SPCR	=   (1<<SPE)        // Enable SPI Transmission
            |   (1<<MSTR)       // SPI-Master
            |   (1<<SPR0);      // Clock Rate 1/16 fosc
								// kein Interrupt
#elif defined(__ATmega164P__) || defined(__ATmega324P__)
	SPSR	|= 	(0<<SPI2X0);	// Double speed
	SPCR	=   (1<<SPE0)		// Enable SPI Transmission
            |   (1<<MSTR0)		// SPI-Master
            |   (1<<SPR00);		// Clock Rate 1/16 fosc
								// kein Interrupt
#elif defined(STM32F030K6T6)
	// TODO...
#elif defined(STM32F030C6T6)
	// TODO...
#elif defined(STM8S207)
#elif defined(STM8S003)
#elif defined(STM8S103)
	// TODO...
#else
#error unbekannter Prozessor
#endif	
	
	/////////////////////////////////////////////
	
		
	//n = spiTransceiver.current_addr;	

#ifdef __ICCAVR__
	for(i=0;i<5;i++)
		spiTransceiver.data[i] = ee_Member_Tab[n].addr_bytes[i];
#else
	//ee_read_bytes(ee_Member_Tab[n].addr_bytes, spiTransceiver.data, 5);
	copy_from_shadow_ee_Member_Tab(shadow_ee_Member_Tab.addr_bytes, spiTransceiver.data, 5);
#endif
	
#ifdef _ACK_MODE	
	if((SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_FEATURE) & 0x07) == 0)
	{
			SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,0x00);
			delay_us(5000);	//__delay_cycles(60000);		// war unspr黱glich 40000 f黵 8MHz
			SPI_Write_Reg(SPI_CMD_ACTIVATE,0x73);
			SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_FEATURE,0x06);//Enables Dynamic Payload Length;Enables Payload with ACK
	}
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_DYNPD,0x3f);	//Enable dynamic payload length data pipe 0,1,2,3,4,5
	SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_PW_P0 + spiTransceiver.current_pipe),SPI_REG_RX_PW_MAX);	//Number of bytes in RX payload in current data pipe
#endif		
	
	if(spiTransceiver.operation_mode == TRANSCEIVER_OPERATION_MODE_TX)
	{
		CeFunk_TransceiverDeactivate();
		spiTransceiver.data[0] += spiTransceiver.current_pipe;//current addr: 0x84,0x7E,0x81,0x7E,0x81
		SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&spiTransceiver.data[0]);	// TX Adresse einstellen
		SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&spiTransceiver.data[0]);	// RX Adresse Pipe0 einstellen	
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x3f);	//Enable auto acknowledgement data pipe 0,1,2,3,4,5													// All Pipes AutoAck
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_RXADDR,0x3f);	//Enable data pipe 0,1,2,3,4,5												// Pipe0,1,4,5 einschalten				
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_CH,spiTransceiver.rf_ch);	//0x04;Sets the frequency channel nRF24L01+ operates on
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x22);//Auto Retransmit Count:2,Auto Retransmit Delay:750us
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,spiTransceiver.rf_setup);//0x07;0dbm, 2Mbps
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,spiTransceiver.config);//0x0e;Tx mode
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_TX,0x00);
		SPI_Write_Buf(SPI_CMD_FLUSH_TX, 0, NULL);
#ifdef TEST_MODE		
//		val = SPI_Read(SPI_CMD_R_REGISTER | SPI_REG_RF_SETUP);
//		asm("nop");
//		val = SPI_Read(SPI_CMD_R_REGISTER | SPI_REG_CONFIG);
//		asm("nop");
//		SPI_Read_Buf(SPI_CMD_R_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&valDat[0]);
//		asm("nop");
//		SPI_Read_Buf(SPI_CMD_R_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&valDat[0]);
//		asm("nop");
//		asm("nop");
#endif
	}else
	{
	//	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,spiTransceiver.config);
          // RX Adresse Pipe0: 0x80,0x7E,0x81,0x7E,0x81
		if(spiTransceiver.current_pipe == 0)
		{
//			spiTransceiver.data[0] = trans_pipe0_addr[n];
			SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&spiTransceiver.data[0]);
			SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&spiTransceiver.data[0]);
		}else
		{
                  // RX Adresse Pipe1: 0x81,0x7E,0x81,0x7E,0x81
			spiTransceiver.data[0] += 1;
			SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P1,SPI_LNG_ADDR,&spiTransceiver.data[0]);
			SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P1,SPI_LNG_ADDR,&spiTransceiver.data[0]);		
		}
//		if(spiTransceiver.current_pipe > 1)
//			SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_ADDR_P0 + spiTransceiver.current_pipe),spiTransceiver.data[0]+spiTransceiver.current_pipe-1);
		// RX Adresse Pipe2: 0x82,0x7E,0x81,0x7E,0x81
                // RX Adresse Pipe3: 0x83,0x7E,0x81,0x7E,0x81
                // RX Adresse Pipe4: 0x84,0x7E,0x81,0x7E,0x81
		for (i = 2; i < 6; i++)
		{
			SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_ADDR_P0 + i), spiTransceiver.data[0] + i - 1);
		};
		
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x3f);														// All Pipes AutoAck
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_RXADDR,0x3f);													// Pipe0,1,4,5 einschalten		
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,(1 << spiTransceiver.current_pipe));	// Enable auto acknowledgement data current pipe 
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_RXADDR,(1 << spiTransceiver.current_pipe) | (1 << 4) | (1 << 5));		// //Enable  current pipe + pipe 4 + pipe 5 
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_CH,spiTransceiver.rf_ch);										// Kanal w鋒len
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x22);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_PW_P0 + spiTransceiver.current_pipe),SPI_REG_RX_PW_MAX);	//Number of bytes in RX payload in  current pipe 
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,spiTransceiver.rf_setup);//0x07;0dbm, 2Mbps
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,spiTransceiver.config);//0x0f
		//SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_RX,0x00);
		SPI_Write_Buf(SPI_CMD_FLUSH_RX, 0, NULL);
#ifdef TEST_MODE		
		val = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_RF_SETUP);
		asm("nop");
		val = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_CONFIG);
		asm("nop");
		SPI_Read_Buf(SPI_CMD_R_REGISTER | SPI_REG_RX_ADDR_P1,SPI_LNG_ADDR,&valDat[0]);	
		asm("nop");
#endif		
		CeFunk_TransceiverActivate();
	}
}
//**********************************************************************************************************
// void Init_RF_Settings(void)
// Diese Funktion initialisiert die spiTransceiver-Struktur. Sie muss nach jedem Power-Down bzw. Brown-Out
// Reset aufgerufen werden.
//**********************************************************************************************************
#ifndef	DEFAULT_PIPE
#define	DEFAULT_PIPE	4
#endif
//Pipe = 4;Channel = 4;
void Init_RF_Settings(unsigned char mode)
{	
	unsigned char i;
	//struct Member Member_Tab;

	if(mode != 0)
	{
		spiTransceiver.operation_mode = TRANSCEIVER_OPERATION_MODE_TX;
		spiTransceiver.config = 0x0e;
	}else
	{
		spiTransceiver.operation_mode = TRANSCEIVER_OPERATION_MODE_RX;
		spiTransceiver.config = 0x0f;
	}
       // if(ee_read_bytes(&ee_RF_CURRENT_ADDR, &shadow_ee_RF_CURRENT_ADDR, sizeof(ee_RF_CURRENT_ADDR)))
	if (ee_shadow_read_init_value(ee_RF_CURRENT_ADDR) > 4)
	{
		ee_shadow_write_value(ee_RF_CURRENT_ADDR, 1);
	}
	

	init_shadow_ee_Member_Tab();
//	if(Member_Tab[n].pipe == 0xff)
	if ((shadow_ee_Member_Tab.pipe > 5)	//	max. 6 Pipes => 5 ist die h鯿hste m鰃liche Pipe...
	 || (shadow_ee_Member_Tab.channel < MIN_CHANNEL))
	{
		shadow_ee_Member_Tab.mode = 1;
		shadow_ee_Member_Tab.channel = 4;
		shadow_ee_Member_Tab.pipe = DEFAULT_PIPE;		// Die Pipe wird beim Teachen auf 4 oder 5 gesetzt...
		shadow_ee_Member_Tab.addr_bytes[0] = trans_pipe0_addr[1];
		for(i=0;i<4;i++) shadow_ee_Member_Tab.addr_bytes[i+1] = trans_base_addr[1][i];//n = 0;  addr:0x80,0x7E,0x81,0x7E,0x81
		store_shadow_ee_Member_Tab();
	}
        for(i=0;i<5;i++)
        {
          seed = shadow_ee_Member_Tab.addr_bytes[i];
          srand(seed);
        }
	spiTransceiver.rf_ch = shadow_ee_Member_Tab.channel;

	spiTransceiver.current_pipe	= shadow_ee_Member_Tab.pipe;
	spiTransceiver.rf_setup = 0x07;


	nrf24l01_port_init();
        delay_us(200);
    SPI_RF_Init();
}

//**********************************************************************************************************
//	void nrf_destroyAddress()
//	"l鰏cht" die aktuelle Adresse indem sie hochgez鋒lt wird...
//**********************************************************************************************************
/*
void nrf_destroyAddress()
{
	unsigned char i;
	
	for (i = 0; i < 4; i++)
		Member_Tab[EE_RF_CURRENT_ADDR].addr_bytes[i + 1]++;
	Member_Tab[EE_RF_CURRENT_ADDR].channel++;
	Member_Tab[EE_RF_CURRENT_ADDR].pipe = 0;		// Die Pipe wird beim Teachen auf 4 oder 5 gesetzt...
	
	Set_Current_Addr(EE_RF_CURRENT_ADDR);
	Change_RF_Channel(Member_Tab[EE_RF_CURRENT_ADDR].channel);
}
*/
//**********************************************************************************************************
//	void nrf_genRandomAddress()
//	"l鰏cht" die aktuelle Adresse indem einen neue Adresse per Zufallsgenerator erzeugt wird...
//	die initialisierung durch srand() erfolg NICHT in der lib, sondern muss durch das Programm erfolgen...
//**********************************************************************************************************

void nrf_genRandomAddress()
{
//	struct Member Member_Tab;

	unsigned int channel;


	
	shadow_ee_Member_Tab.addr_bytes[0] = rand() & 0xF0;

	*(int*)(shadow_ee_Member_Tab.addr_bytes + 1) = rand();
	channel = (rand() % 78) + MIN_CHANNEL;
	shadow_ee_Member_Tab.channel = channel;
	shadow_ee_Member_Tab.pipe = 0;		// Die Pipe wird beim Teachen auf 4 oder 5 gesetzt...

	store_shadow_ee_Member_Tab();
	
	Set_Current_Addr();
	Change_RF_Channel(channel);
}

void nrf_setDefaultAddress()
{
	//struct Member Member_Tab;

	ee_shadow_write_value(ee_RF_CURRENT_ADDR, 1);

	shadow_ee_Member_Tab.mode = 1;
	shadow_ee_Member_Tab.channel = 4;
	shadow_ee_Member_Tab.pipe = DEFAULT_PIPE;
	shadow_ee_Member_Tab.addr_bytes[0] = trans_pipe0_addr[1];
#ifdef __ICCAVR__
	for(unsigned char i=0; i<4; i++)
		ee_Member_Tab.addr_bytes[i+1] = trans_base_addr[1][i];
#else
	memcpy(shadow_ee_Member_Tab.addr_bytes + 1, trans_base_addr[1], 4);
#endif

	store_shadow_ee_Member_Tab();

	Set_Current_Addr();
	Change_RF_Channel(4);
}

//**********************************************************************************************************
// unsigned char Ext_IRQ_Service(void)
// Diese Funktion wertet die Interrupt-Anfrage aus.
//**********************************************************************************************************
unsigned char Ext_IRQ_Service(void)
{
	static unsigned char temp;
	
	temp = SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_STATUS, 0x70);	//	IRQs auslesen und alle IRQs l鰏chen
#warning	//IRQ_Ext = 0;

//	IntFunkEin();


	if(temp & SPI_REG_STATUS_RX_DR)				// Rx Daten abholen
	{
//		spiTransceiver.rx_pipe_nr = (temp>>1)&0x07;
		//读取到的数据长度为0，认为没有读到数据，清标志
		if (readRxPayload() == 0) temp &= ~SPI_REG_STATUS_RX_DR;
                
//		return SPI_REG_STATUS_RX_DR;
	}
//	if(temp & SPI_REG_STATUS_TX_DS)				// Tx-Ack empfangen
//	{
//		return SPI_REG_STATUS_TX_DS;
//	}
	if(temp & SPI_REG_STATUS_MAX_RT)			// Empfangsbest鋞igung (Ack) augeblieben
	{
//		发送FIFO满
		SPI_Write_Buf(SPI_CMD_FLUSH_TX, 0, NULL);
//		return SPI_REG_STATUS_MAX_RT;
	}
	return temp;
}
//**********************************************************************************************************
// void Change_RF_Channel(unsigned char ch)
// Diese Funktion wechselt den Funk-Kanal.
//**********************************************************************************************************
void Change_RF_Channel(unsigned char ch)
{
	spiTransceiver.rf_ch = ch;
	SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_CH,spiTransceiver.rf_ch);
}
//**********************************************************************************************************
// void Change_Tx_Addr(unsigned char addr, unsigned char pipe)
//
//**********************************************************************************************************
void Set_Teach_Addr(unsigned char addr, unsigned char pipe)
{
	unsigned char i;
					
	for(i=0;i<4;i++)				// Adresse kopieren
		spiTransceiver.data[i+1] = trans_base_addr[addr][i];
    spiTransceiver.current_teach_addr = addr;					// 膎derungen speichern
	if(pipe <= 5)
	{
		spiTransceiver.data[0] = trans_pipe0_addr[addr];
		spiTransceiver.current_pipe	= pipe;	
	}else
	{
		spiTransceiver.data[0] = trans_pipe0_addr[addr];
	}
	Change_Addr();
}
//**********************************************************************************************************
// void Set_Current_Addr(void)
//
//**********************************************************************************************************
/*
void Set_Current_Addr(void)
{
	unsigned char i;
	
	for(i=0;i<5;i++)
		spiTransceiver.data[i] = Member_Tab[spiTransceiver.current_addr].addr_bytes[i];
	spiTransceiver.current_pipe	= Member_Tab[spiTransceiver.current_addr].pipe;
	Change_Addr();
}
*/
void Set_Current_Addr(void)
{
#ifdef __ICCAVR__
	unsigned char i;
	
	for(i=0;i<5;i++)
		spiTransceiver.data[i] = ee_Member_Tab.addr_bytes[i];
#else
	copy_from_shadow_ee_Member_Tab(shadow_ee_Member_Tab.addr_bytes, spiTransceiver.data, 5);
#endif
	spiTransceiver.current_pipe	= shadow_ee_Member_Tab.pipe;
	Change_Addr();
}
//**********************************************************************************************************
// void Change_Rf_Addr(unsigned char* addr, unsigned char pipe, unsigned char index)
//
//**********************************************************************************************************
void Change_Rf_Addr(unsigned char* addr, unsigned char pipe)
{
	spiTransceiver.current_pipe	= pipe;
	shadow_ee_Member_Tab.pipe = pipe;
#ifdef __ICCAVR__
	for(i=0;i<5;i++)						// Adresse kopieren
	{
		spiTransceiver.data[i] = addr[i];
		ee_Member_Tab.addr_bytes[i] = addr[i];
	}
#else
	memcpy(spiTransceiver.data, addr, 5);
	memcpy(shadow_ee_Member_Tab.addr_bytes, addr, 5);
#endif
	store_shadow_ee_Member_Tab();
	Change_Addr();
}
//**********************************************************************************************************
// void Change_Addr(void)
//
//**********************************************************************************************************
void Change_Addr(void)
{
	unsigned char i;
	
	if(spiTransceiver.operation_mode == TRANSCEIVER_OPERATION_MODE_TX)
	{
		spiTransceiver.data[0] += spiTransceiver.current_pipe;
		SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&spiTransceiver.data[0]);		// TX Adresse wechseln
		SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&spiTransceiver.data[0]);	// RX Adresse Pipe0 wechseln
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x3f);														// All Pipes AutoAck
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_RXADDR,0x3f);													// Pipe0,1,4,5 einschalten	
		CeFunk_TransceiverDeactivate();
	}else
	{
		if(spiTransceiver.current_pipe == 0)
		{
			SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&spiTransceiver.data[0]);				// RX Adresse Pipe0 einstellen
		}else
		{
			spiTransceiver.data[0] += 1;
			SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P1,SPI_LNG_ADDR,&spiTransceiver.data[0]);				// RX Adresse Pipe1 einstellen
		}
//		if(spiTransceiver.current_pipe > 1)
//			SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_ADDR_P0 + spiTransceiver.current_pipe),spiTransceiver.data[0]+(spiTransceiver.current_pipe-1));	
		for (i = 2; i < 6; i++)
		{
			SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_ADDR_P0 + i), spiTransceiver.data[0] + i - 1);
		};
		
#ifdef TRANS_TX		
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,0x3f);														// All Pipes AutoAck
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_RXADDR,0x3f);													// Pipe0,1,4,5 einschalten	
#else		
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_AA,(1 << spiTransceiver.current_pipe));						// Pipe x AutoAck
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_EN_RXADDR,(1 << spiTransceiver.current_pipe) | (1 << 4) | (1 << 5));			// Pipe x und 4 + 5 einschalten
#endif		
		SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_PW_P0 + spiTransceiver.current_pipe),SPI_REG_RX_PW_MAX);		// Anzahl der Datenbytes
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x22);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_RF_SETUP,spiTransceiver.rf_setup);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,spiTransceiver.config);
		//SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_RX,0x00);		
		SPI_Write_Buf(SPI_CMD_FLUSH_RX, 0, NULL);
		CeFunk_TransceiverActivate();
	}
#ifdef TEST_MODE
		SPI_Read_Buf(SPI_CMD_R_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&valDat[0]);	
		asm("nop");
		SPI_Read_Buf(SPI_CMD_R_REGISTER | SPI_REG_RX_ADDR_P1,SPI_LNG_ADDR,&valDat[0]);	
		asm("nop");
		val = SPI_Read_Reg(SPI_CMD_R_REGISTER | (SPI_REG_RX_ADDR_P0+spiTransceiver.current_pipe));
		asm("nop");
		val = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_RF_CH);
		asm("nop");		
#endif
}
//**********************************************************************************************************
// void ChangeTxPipe(unsigned char pipe,unsigned char member)
//**********************************************************************************************************
void ChangeTxPipe(unsigned char pipe,unsigned char member)
{
	
	spiTransceiver.current_pipe = pipe;
#ifdef __ICCAVR__
	for(i=0;i<5;i++)
		spiTransceiver.data[i] = ee_Member_Tab[member].addr_bytes[i];
#else
	copy_from_shadow_ee_Member_Tab(shadow_ee_Member_Tab.addr_bytes, spiTransceiver.data, 5);
#endif
	spiTransceiver.data[0] += spiTransceiver.current_pipe;
	SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_TX_ADDR,SPI_LNG_ADDR,&spiTransceiver.data[0]);		// TX Adresse wechseln
	SPI_Write_Buf(SPI_CMD_W_REGISTER | SPI_REG_RX_ADDR_P0,SPI_LNG_ADDR,&spiTransceiver.data[0]);	// RX Adresse Pipe0 wechseln	
}
//**********************************************************************************************************
// void Save_RF_Settings(void)
//
//**********************************************************************************************************
/*
void Save_RF_Settings(void)
{
	ee_shadow_write_value(&ee_RF_CH, shdw_RF_CH, spiTransceiver.rf_ch);
	ee_write_value(&ee_RF_CURRENT_ADDR, spiTransceiver.current_addr);
	ee_write_value(&ee_RF_CURRENT_PIPE, spiTransceiver.current_pipe);
}
*/
//**********************************************************************************************************
// void Get_Data(unsigned char* p_dat)
// Diese Funktion kopiert Daten aus der spiTransceiver-Structur in den mit p_dat angegebenen Speicherort.
//**********************************************************************************************************
void Get_Data(unsigned char* p_dat)
{
	unsigned char i;
	for(i=0;i<SPI_REG_RX_PW_MAX;i++)
		p_dat[i] = spiTransceiver.data[i];
}
//**********************************************************************************************************
// unsigned char Create_new_channel(unsigned int s)
// Diese Funktion gibt eine Zufallszahl zwischen 3...80 zur點k. Sie wird mit dem Parameter s initialisiert.
//**********************************************************************************************************
unsigned char Create_new_channel(unsigned int s)
{	
	srand(s);
	return((unsigned char)(rand() % 78)+3);
}
//**********************************************************************************************************
// void Create_new_address(unsigned int s, unsigned char* p_dat)
// Diese Funktion erzeugt eine neue Member-Adresse aus Zufallszahlen. Sie wird mit dem Parameter s
// initialisiert.
//**********************************************************************************************************
void Create_new_address(unsigned int s, unsigned char* p_dat)
{
	srand(s);
	p_dat[0] = ((unsigned char)rand()) & 0xf0;
	p_dat[1] = (unsigned char)rand();
	p_dat[2] = (unsigned char)rand();
	p_dat[3] = (unsigned char)rand();
	p_dat[4] = (unsigned char)rand();
}
//**********************************************************************************************************
// void Transceiver_On(void)
//**********************************************************************************************************
/*
void Transceiver_On(void)
{
#warning kann weg...
	//FunkEin_Activate();             		// Transceiverbetriebsspannung einschalten

	__delay_cycles(40000); 					// warte 2 ms bis Power Down Mode erreicht
	__delay_cycles(40000);
	__delay_cycles(40000);
}
*/
//**********************************************************************************************************
// void Transceiver_Off(void)
//**********************************************************************************************************
/*
void Transceiver_Off(void)
{
#warning kann weg...
	//FunkEin_Deactivate();
	__delay_cycles(40000);
}
*/
//**********************************************************************************************************
// void Switch_Rf_mode(unsigned char mode)
//**********************************************************************************************************
/*
void Switch_Rf_mode(unsigned char mode)
{
	Init_RF_Settings(mode);
	SPI_RF_Init();
}
*/
void Switch_Rf_mode(unsigned char mode)
{
  SPI_Write_Reg(SPI_CMD_ACTIVATE,0x73);
  SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_FEATURE,0x06);
  SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_DYNPD,0x3f);	//Enable dynamic payload length data pipe 0,1,2,3,4,5
  SPI_Write_Reg(SPI_CMD_W_REGISTER | (SPI_REG_RX_PW_P0 + spiTransceiver.current_pipe),SPI_REG_RX_PW_MAX);	//Number of bytes in RX payload in current data pipe
	
	if(mode != 0)
	{
            CeFunk_TransceiverDeactivate();

		spiTransceiver.operation_mode = TRANSCEIVER_OPERATION_MODE_TX;
		spiTransceiver.config = 0x0e;
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_AW, 0x03);
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_STATUS, 0x70);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x22);
		//SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_TX,0);
		SPI_Write_Buf(SPI_CMD_FLUSH_RX, 0, NULL);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,spiTransceiver.config);
		delay_us(10);	//__delay_cycles(1800);	// 150 祍 warten
		
	}else
	{
//		CeFunk_TransceiverDeactivate();
		spiTransceiver.operation_mode = TRANSCEIVER_OPERATION_MODE_RX;
		spiTransceiver.config = 0x0f;
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_AW, 0x03);
//		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_STATUS, 0x70);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_SETUP_RETR,0x22);
		//SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_CMD_FLUSH_RX,0);
		SPI_Write_Buf(SPI_CMD_FLUSH_RX, 0, NULL);
		SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_CONFIG,spiTransceiver.config);
		delay_us(10);	//__delay_cycles(1800);	// 150 祍 warten
		CeFunk_TransceiverActivate();
	}
}

//**********************************************************************************************************
// SPI_TransceiverTimer_1ms
//
// Die Timer-Routine wird jede Millisekunde vom 1ms-Timer-Interrupt
// aufgerufen und sorgt f黵 eine Entprellung der Tastendr點ke bzw.
// zur Vermeidung eines Transmitter黚erlaufes beim Handsender.
//
// Sie wird au遝rdem empf鋘gerseitig verwendet, um beim Ausbleiben von
// Tastatur-Befehlen f黵 eine sichere R點kstellung (keine Taste gedr點kt)
// nach einer maximalen Zeit vom 300 ms zu sorgen.
//**********************************************************************************************************

void SPI_TransceiverTimer_1ms(void)
{
    if (spiTransceiver.timer_1ms > 0)
    {
        spiTransceiver.timer_1ms--;
//        if (spiTransceiver.timer_1ms == 0)
//            spiTransceiver.timer_flag = 0x99;   // Timer abgelaufen Flag setzen
    }
}


//**********************************************************************************************************
//
//**********************************************************************************************************

signed char readRxPayload()
{
	unsigned char status;
	unsigned char pipe;
	signed char retValue = 0;
	unsigned char dummy[SPI_REG_RX_PW_MAX];
//	unsigned char maxCycles = 15;
#warning Pr黤en ob die Schleife terminiert!
//	while (maxCycles--)
	while(42)
	{
//		status = SPI_Read_Reg(SPI_CMD_R_REGISTER | SPI_REG_STATUS);
		status = SPI_Write_Reg(SPI_CMD_W_REGISTER | SPI_REG_STATUS, SPI_REG_STATUS_RX_DR);	//	status auslesen und RX_DR IRQ l鰏chen
		pipe = (status >> 1) & 0x07;
	
		if (pipe == 0x07)							//	FIFO empty...
		{
			return retValue;
		} else
		{
			dataReceivedTimer = DATA_TIMEOUT;	//	dieser Timer gibt zur點k ob Pakete empfangen wurden.
												//	wird benutzt um zu entscheiden ob das System in Stand-By gehen darf - soll also auch gesetzt werden wenn ung黮tige Pakete oder Pakete mit 躡ertragungsfehlern empfangen werden.
			
			if (SPI_Read_Reg(SPI_CMD_R_RX_PL_WID) > SPI_REG_RX_PW_MAX)
			{
				SPI_Write_Buf(SPI_CMD_FLUSH_RX, 0, NULL);	// 躡ertragungsfehler - FIFO l鰏chen...
				continue;
			}

			if (pipe == spiTransceiver.current_pipe)	//	Daten empfangen...
			{
				
				if (retValue != 0)
					return retValue;
				
				retValue = -1;
				SPI_Read_Buf(SPI_CMD_R_RX_PAYLOAD, SPI_REG_RX_PW_MAX, spiTransceiver.data);
			} else										// 	sontige Daten empfangen - werden verworfen...
			{
				SPI_Read_Buf(SPI_CMD_R_RX_PAYLOAD, SPI_REG_RX_PW_MAX, dummy);
			};
		};
	};
//	__asm("nop");
//	return retValue;
}

//**********************************************************************************************************
//
//**********************************************************************************************************



