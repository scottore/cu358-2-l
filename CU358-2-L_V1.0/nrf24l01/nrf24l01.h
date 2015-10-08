/**
 * @file nrf24l01.h
 * @author Steffen Loley
 * @brief Prototypes for the nrf24l01 library
 *
 * This header contains prototypes to access the nrf24l01 library, and prototypes that are used by the nrf24l01 library.
 */

#include "nrf24l01_port.h"
/**
 * @defgroup nrfAddress radio frequency and address control
 *
 * @brief These functions can be called by the main program to change the radio frequency and address.
 * @{
 */
void nrf_destroyAddress();		///< This function deletes the current RF address and channel.
void nrf_genRandomAddress();	///< This function generates new RF address and channel (and overrides the old ones). @remark Initialization of the random number generator is not part of the library. srand() must be called by the main program.
void nrf_setDefaultAddress();	///< This function restores the default address.
/** @}*/ // end of group nrfAddress

/**
 * @brief busy waiting
 * This function is called by the library to wait the specified amount of time.
 * @remark This function is not part of the library and need to be implemented by the main program.
 */
void delay_us(unsigned int microSec);

/**
 * @brief default pipe number
 * This variable set up the default pipe number which is part of the default address.
 * If unsure set it to 4 which is the default.
 * @remark This variable is not part of the library and need to be implemented by the main program.
 */
extern  const unsigned char DEFAULT_PIPE;	// default = 4

/** @}*/ // end of group halFunctions


/** @cond */
////////////////////////////////////////////////////////////////////////////////
//
//  internal stuff - do not touch!
//
////////////////////////////////////////////////////////////////////////////////


//******** Makros **************************************************************

#ifdef __ICCAVR__
void ee2memcpy(char * const dst, const char __eeprom * const src, const unsigned char size);
#define	init_shadow_ee_Member_Tab()
#define	store_shadow_ee_Member_Tab(n)
#define	copy_from_shadow_ee_Member_Tab(mTab, var, size)	ee2memcpy(var, mTab, size)
#else
#define	init_shadow_ee_Member_Tab()					ee_read_bytes((unsigned char*)&ee_Member_Tab, (unsigned char*)&shadow_ee_Member_Tab, sizeof(shadow_ee_Member_Tab))
#define	store_shadow_ee_Member_Tab()					ee_write_bytes((unsigned char*)&ee_Member_Tab, (unsigned char*)&shadow_ee_Member_Tab, sizeof(shadow_ee_Member_Tab))
#define	copy_from_shadow_ee_Member_Tab(mTab, var, size)	memcpy(var, mTab, size)
#endif


#define TRANSCEIVER_OPERATION_MODE_RX  0   // Empfängerbetrieb
#define TRANSCEIVER_OPERATION_MODE_TX  1   // Handsenderbetrieb

                // Transceiver Interruptrequest
#define SPI_TRANSCEIVER_REQUEST 1

                // Kommandos der Transceiverschnittstelle
#define SPI_CMD_R_REGISTER      0x00
#define SPI_CMD_W_REGISTER      0x20U  /**< Register write command */
#define SPI_CMD_ACTIVATE	0x50U  /**< Activate features */
#define SPI_CMD_R_RX_PL_WID	0x60U  /**< Read RX payload command */
#define SPI_CMD_R_RX_PAYLOAD    0x61U  /**< Read RX payload command */
#define SPI_CMD_W_TX_PAYLOAD    0xA0U  /**< Write TX payload command */
#define SPI_CMD_W_ACK_PAYLOAD	0xA8U  /**< Write ACK payload command */
#define SPI_CMD_FLUSH_TX        0xE1U  /**< Flush TX register command */
#define SPI_CMD_FLUSH_RX        0xE2U  /**< Flush RX register command */
#define SPI_CMD_REUSE_TX_PL     0xE3U  /**< Reuse TX payload command */

    // Register der Transceiverschnittstelle
#define SPI_REG_CONFIG          0x00    // Configuration
#define SPI_REG_EN_AA           0x01    // Autoacknowledgement
#define SPI_REG_EN_RXADDR       0x02    // Enable Data-Pipes
#define SPI_REG_SETUP_AW		0x03	// Setup of Address Widths
#define SPI_REG_SETUP_RETR      0x04    // Auto Retransmission
#define SPI_REG_RF_CH           0x05    // Frequency Channel
#define SPI_REG_RF_SETUP        0x06    // Radio Parameter Setup
#define SPI_REG_STATUS          0x07    // Status Register Setup
#define SPI_REG_OBSERVE_TX      0x08    // Transmit Observe
#define SPI_REG_CD              0x09    // Carrier Detect
#define SPI_REG_RX_ADDR_P0      0x0a    // Empfangsaddresse Pipe 0
#define SPI_REG_RX_ADDR_P1      0x0b    // Empfangsaddresse Pipe 1
#define SPI_REG_RX_ADDR_P2      0x0c    // Empfangsaddresse Pipe 2
#define SPI_REG_RX_ADDR_P3      0x0d    // Empfangsaddresse Pipe 3
#define SPI_REG_RX_ADDR_P4      0x0e    // Empfangsaddresse Pipe 4
#define SPI_REG_RX_ADDR_P5      0x0f    // Empfangsaddresse Pipe 5
#define SPI_REG_TX_ADDR         0x10    // Sendeadresse
#define SPI_REG_RX_PW_P0        0x11    // RX Pipe 0 Byteanzahl
#define SPI_REG_RX_PW_P1        0x12    // RX Pipe 1 Byteanzahl
#define SPI_REG_RX_PW_P2        0x13    // RX Pipe 2 Byteanzahl
#define SPI_REG_RX_PW_P3        0x14    // RX Pipe 3 Byteanzahl
#define SPI_REG_RX_PW_P4        0x15    // RX Pipe 4 Byteanzahl
#define SPI_REG_RX_PW_P5        0x16    // RX Pipe 5 Byteanzahl
#define SPI_REG_FIFO_STATUS     0x17    // FIFO Statusregister
#define SPI_REG_DYNPD			0x1c	// Dynamic Payload-Register
#define SPI_REG_FEATURE			0x1d	// Feature-Register

                // Bit-Masken der Transceiver-Register
#define SPI_REG_CONFIG_EN_CRC       0x08    // Enable CRC
#define SPI_REG_CONFIG_CRCO         0x04    // 1: 16 Bit CRC, 0: 8 Bit CRC
#define SPI_REG_CONFIG_PWR_UP       0x02    // Power Up
#define SPI_REG_CONFIG_PRIM_RX      0x01    // RX Mode

#define SPI_REG_EN_AA_ALL           0x3f    // Enable Auto ACK on all Pipes

#define SPI_REG_EN_RXADDR_ERX_ALL   0x3f    // Enable all Data-Pipes
#define SPI_REG_EN_RXADDR_ERX_P01   0x03    // Enable Data-Pipes 0 und 1
#define SPI_REG_EN_RXADDR_ERX_P014  0x13    // Enable Data-Pipes 0, 1 und 4
#define SPI_REG_EN_RXADDR_ERX_P015  0x23    // Enable Data-Pipes 0, 1 und 5
#define SPI_REG_EN_RXADDR_ERX_P0145 0x33    // Enable Data-Pipes 0, 1, 4u.5


#define SPI_REG_SETUP_RETR_ARD      0x10    // Auto retransmission delay 500us
#define SPI_REG_SETUP_RETR_ARC      0x05    // Auto retransmission count 5 times

#define SPI_REG_RF_SETUP_RF_DR      0x00    // 1: 2 Mbps, 0: 1 Mbps
#define SPI_REG_RF_SETUP_RF_PWR     0x06    // RF output power 0 dBm
#define SPI_REG_RF_SETUP_LNA_HCURR  0x01    // LNA max gain

#define SPI_REG_STATUS_RX_DR        0x40    // Data Ready in RX-FIFO
#define SPI_REG_STATUS_TX_DS        0x20    // Data Send from TX-FIFO
#define SPI_REG_STATUS_MAX_RT       0x10    // TX-FIFO full
#define SPI_REG_STATUS_RX_P_NO_EMPTY 0x0e   // RX-FIFO empty

#define SPI_REG_FIFO_STATUS_TX_EMPTY 0x10   // TX FIFO EMPTY
#define SPI_REG_FIFO_STATUS_RX_EMPTY 0x01   // RX FIFO EMPTY
#define SPI_IDLE            		 0xff   // Nothing to do
#define SPI_REG_RX_PW_MAX           32      // Maximale Pipe Grösse

                // Transferlängen für Einzel und Payload-Register
#define SPI_LNG_SINGLE_CMD  1       // nur 1 Byte CMD
#define SPI_LNG_SINGLE_REG  2       // 1 Byte Einzelregister plus 1 Byte Cmd/Status
#define SPI_LNG_ADDR		5
#define SPI_LNG_PAYLOAD     33      // 32 Byte Payload plus 1 Byte Cmd/Status

#define CeFunk_TransceiverActivate()    nrf24l01_setCE_high()	// Transceiver CE = 1
#define CeFunk_TransceiverDeactivate()  nrf24l01_setCE_low()	// Transceiver CE = 0
#define CsnFunk_TransmitActivate()      nrf24l01_setCSN_low()	// Transceiver CSN = 0
#define CsnFunk_TransmitDeactivate()    nrf24l01_setCSN_high()	// Transceiver CSN = 1

#define DATA_TIMEOUT	40		// ca. 400 ms...

#define	MIN_CHANNEL		3		//	unterster Kanal der möglich ist...

struct TransceiverData
{               //------------------------ Betriebsart -------------------------
    unsigned char operation_mode;       // TX(Handsender), RX(Nachrüstempfänger)
				//------------------------ Teilnehmeradresse -------------------

	unsigned char current_teach_addr;
	unsigned char current_pipe;
                //------------------------ Fehlerbehandlung/Wartung ------------
    unsigned char config;               // Configuration Register
    unsigned char rf_ch;                // Frequency Channel
    unsigned char rf_setup;             // Air Data Rate, RF Output Power
    unsigned char status;               // Statusregister
    unsigned char observe_tx;           // Transmitter observe register
    unsigned char cd;                   // Carrier Detect
    unsigned char fifo_status;          // Fifo Status Register
//    unsigned char rx_pipe_nr;           // Empfangene Pipe Nummer vom Status-Register
    unsigned char tx_pipe_offset;       // Transmit-Address-Offset 0..5
                //------------------------ Auftragsverwaltung ------------------
    unsigned int  timer_1ms;            // Entprellung Applikations- oder Empfangsaufträge
    unsigned char timer_flag;           // Timer abgelaufen Flag
    unsigned char application_request;  // Applikationsaufträge
    unsigned char transceiver_request;  // Transceiverinterrupts
    unsigned char transmit_request;     // Transmitsteuerung
                //------------------------ Schnittstellenverwaltung ------------
    unsigned char state;                // Schnittstellenzustand
    unsigned char size;                 // Transferbyteanzahl
    unsigned char count;                // Transferbytezähler
    unsigned char data[33];             // 1 SPI-Cmd/Status-Byte + 32 Transferbytes
};

struct Member
{
	unsigned char id;
	unsigned char mode;
	unsigned char channel;
	unsigned char pipe;
	unsigned char addr_bytes[5];
};


//********* Variablen **************************************************************************************

//extern volatile unsigned char IRQ_Ext;
extern volatile unsigned char dataReceivedTimer;
extern struct TransceiverData spiTransceiver;

//*********Funktions Prototypen*****************************************************************************
unsigned char SPI_Transmit(unsigned char ch);
unsigned char SPI_Read(unsigned char ch);
unsigned char SPI_Write_Reg(unsigned char reg, unsigned char val);
unsigned char SPI_Read_Reg(unsigned char reg);
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char num, unsigned char *p_buf);
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char num, unsigned char *p_buf);
void SPI_RF_Init(void);
void Init_RF_Settings(unsigned char mode);
unsigned char Ext_IRQ_Service(void);
void SPI_TransceiverTimer_1ms(void);
void Change_RF_Channel(unsigned char ch);
void Set_Teach_Addr(unsigned char addr, unsigned char pipe);
//void Set_Current_Addr(void);
void Set_Current_Addr(void);
void Change_Rf_Addr(unsigned char* addr, unsigned char pipe);
void Change_Addr(void);
//void Save_RF_Settings(void);
void Get_Data(unsigned char* p_dat);
unsigned char Create_new_channel(unsigned int s);
void Create_new_address(unsigned int s, unsigned char* p_dat);
//void Transceiver_On(void);
//void Transceiver_Off(void);
void Switch_Rf_mode(unsigned char mode);
void ChangeTxPipe(unsigned char pipe,unsigned char member);
signed char readRxPayload();
//**********************************************************************************************************/

/** @endcond */
