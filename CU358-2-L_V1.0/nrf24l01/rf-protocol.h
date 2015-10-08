/**
 * @file rf-protocol.h
 * @author Steffen Loley
 * @brief Prototypes for the nrf24l01 library
 *
 * This header contains prototypes to access the nrf24l01 library.
 */

/** \mainpage nrf24L01 library
 *
 * \section intro_sec Introduction
 *
 * This library will give you access to the nrf24L01 chips from nordic.
 * It also implements the protocol that is used for RF communication by standard DewertOkin handsets that operate at 2.4Ghz
 *
 * \section install_sec Usage
 *
 * @subsection step1 Step 1: hardware
 *
 * The following functions must be implemented to access the hardware: \n
 * @ref nrf24l01_setCE_high \n
 * @ref nrf24l01_setCE_low \n
 * @ref nrf24l01_setCSN_high \n
 * @ref nrf24l01_setCSN_low \n
 * @ref nrf24l01_getIRQ \n
 * @ref delay_us \n
 * @ref DEFAULT_PIPE \n
 *
 * \subsection step2 Step 2: initialization
 *
 * For initialization @ref Init_Funk must be called once after power up.
 *
 * \subsection step3 Step 3: handling the traffic
 *
 * @ref FunkAppl must be called inside the infinite loop from the main() function. \n
 * @ref RF_Timer_10ms must be called every 10 ms.
 *
 * @subsection step4 Step 4: getting the key-codes
 *
 * Call @ref getRFbuttons to poll the last received key-codes.
 */


/**
 * @defgroup rfProtocol RF protocol
 *
 * @brief These functions can be used to handle the RF protocol.
 * @{
 */

/**
 * @brief This function return the last received key-codes.
 * @return 32-bit value containing the key-code.
 */
unsigned long getRFbuttons();

/**
 * @brief This function must be called every 10ms from the main programm.
 * This is the basis for all timers of the ner24L01 library.
 */
void RF_Timer_10ms(void);

/**
 * @brief This function enters a test-mode. It can be useful to test the antenna.
 * @deprecated use @ref rfEnterTestmode or @ref rfEnterTestmodeConstCarrier instead
 */
void StartRfTestMode(void);

/**
 * @brief This function exits the test-mode. see also @ref StartRfTestMode
 * @deprecated use @ref rfEnterTestmode or @ref rfEnterTestmodeConstCarrier instead
 */
void endRfTestMode(void);

/**
 * @brief This function does the initialization of the library.
 *
 * This function must be called once after the device is powered up.
 */
void Init_Funk(void);

/**
 * @brief This function does the radio protocol handling.
 *
 * This function must be called from the infinite loop of the main() function.
 * This function handles the radio protocol.
 */
void FunkAppl(void);
void FlushRfModule(void);
/**
 * @brief This function does the radio protocol handling with synchronization.
 *
 * This function must be called from the infinite loop of the main() function.
 * This function handles the radio protocol.
 * This is the same as FunkAppl(), but this version allows to transmit some synchronization data to the handset, which is than transmitted to a 2nd control unit.
 */
void FunkApplSync(void);

/**
 * @brief This function sets feedback values (without overriding old ones).
 *
 * This function allows to send a command the the handset to control its illumination to have some kind of feedback.
 * The first parameter is a 32-bit value, representing a bitfield that indicates which LEDs should be controlled.
 * The second parameter can be one of the following macros:
 * <pre>
 * 		SW_CMD_LED_OFF    Turn the selected LEDs off
 * 		SW_CMD_LED_ON     Turn the selected LEDs on
 * 		SW_CMD_LED_TOGGLE Toggle the selected LEDs
 * 		SW_CMD_LED_AUTO   Reset the selected LEDs
 * </pre>
 * Multiple switchLEDs-calls will be combined. Previous calls will not be overridden.
 * see also @ref overrideLEDsCmd
 */
void switchLEDs(unsigned long long temp, unsigned char cmd);

/**
 * @brief This function sets feedback values (by overriding old ones).
 *
 * This function allows to send a command the the handset to control its illumination to have some kind of feedback.
 * The first parameter is a 32-bit value, representing a bitfield that indicates which LEDs should be controlled.
 * The second parameter can be one of the following macros:
 * <pre>
 * 		SW_CMD_LED_OFF    Turn the selected LEDs off
 * 		SW_CMD_LED_ON     Turn the selected LEDs on
 * 		SW_CMD_LED_TOGGLE Toggle the selected LEDs
 * 		SW_CMD_LED_AUTO   Reset the selected LEDs
 * </pre>
 * Previous calls will be overridden (if they are not sent out to the handset).
 * see also @ref switchLEDs
 */
void overrideLEDsCmd(unsigned long long temp, unsigned char cmd);

#define	SW_CMD_LED_OFF		0	///<	This macro is used together with switchLEDs to turn feedback off.
#define	SW_CMD_LED_ON		1	///<	This macro is used together with switchLEDs to turn feedback on.
#define	SW_CMD_LED_TOGGLE	2	///<	This macro is used together with switchLEDs to toggle feedback.
#define	SW_CMD_LED_AUTO		3	///<	This macro is used together with switchLEDs to reset feedback.

void RF_PWR_Up_lowPower();
void RF_PWR_Up_1MHz();		//	Funkchip aufwecken für 1MHz CPU clock oder kleiner
void RF_PWR_Up_4MHz();		//	Funkchip aufwecken für 4MHz CPU clock oder kleiner
void RF_PWR_Down();			//	Funkchip in stand-by setzen

/**
 * @brief This function enters a test-mode.
 *
 * During test-mode packets are sent endlessly without any pause between two packets.
 *
 * @param channel This can be any channel between 0 and 83 resulting in a carrier frequency between 2.400 GHz and 2.483 GHz.
 * @param data This is the value that is used to fill the payload of the packets.
 *
 * see also @ref rfEnterTestmodeConstCarrier
 */
void rfEnterTestmode(unsigned char channel, unsigned char data);

/**
 * @brief This function enters a test-mode and sends a constant carrier
 *
 * This function enters a test-mode. During this test-mode a constant carrier is sent.
 * This can be useful to test the antenna or to check the radio frequency.
 *
 * @param channel This can be any channel between 0 and 83 resulting in a carrier frequency between 2.400 GHz and 2.483 GHz.
 */
void rfEnterTestmodeConstCarrier(unsigned char channel);

/**
 * @brief This function sends out a single packet for testing.
 *
 * @param channel This can be any channel between 0 and 83 resulting in a carrier frequency between 2.400 GHz and 2.483 GHz.
 * @param data This is the value that is used to fill the payload of the packet.
 */
void rfSendSingleTestPacket(unsigned char channel, unsigned char data);

/**
 * @brief This macros enters teach mode.
 *
 * While in teach-mode this control unit can be paired with a handset.
 * Teach-mode will automatically exit.
 */
#define setTeachMode()	{Rx_mode = TEACH_MODE;}

/**
 * @brief This macros exits teach-mode.
 *
 * @remark Usually teach-mode will exit automatically after a short amount of time. So usually there is no need to call this functions. Only call this function if you really want to exit teach-mode before the pairing process has finished, or if you have entered teach-mode accidently.
 */
#define clrTeachMode()	{TeachTimer = 0;}

/**
 * @brief check if teach-mode is active.
 *
 * This macro can be used to read out if teach mode is active.
 */
#define getTeachMode()	((Rx_mode == TEACH_MODE) || (Rx_mode == WAIT_FOR_REQ))

/**
 * @brief This macro is non-zero if there are incoming radio packets.
 *
 * It can be used to check if the radio link is active or not.
 */
#define getRFactive()	(RfTimer || dataReceivedTimer)

#define getPipe()		spiTransceiver.current_pipe

extern unsigned char rfSyncDataOutLen;	///<	This variable indicates the length of rfSyncDataOut[].
extern unsigned char rfSyncDataOut[];	///<	Store Date that should be sent to a second control unit into this array.

extern unsigned char	rfSyncDataInLen;	///<	This variable indicates the length of rfSyncDataIn[].
extern unsigned char	rfSyncDataIn[];		///<	Incoming data from a second control unit can be read from this array.
extern unsigned char	rfSyncDataInTimer;
extern signed char		rfSyncDataInFlag;

/** @} */

/** @cond */
////////////////////////////////////////////////////////////////////////////////
// 
// internal stuff...
//

#define Timer3_Reload_5ms()		TCNT3 = 25535
#define BUTTON 	34
#define RF_TIMEOUT	40				// 400 ms Timeout
//****************************************************************************************************************
// Commands (Funkübertragung)
//****************************************************************************************************************
#define RF_MODULID		0x01		// Identifizierung
#define RF_DATA			0x02		// Datenübertragung
#define RF_BUTTON		0x03		// Tastenstatus des Handsenders
#define RF_CHANGE_ADDR	0x04		// Adress bzw. Kanalwechsel
#define RF_SET_CURRENT_ADDR 0x05	// current Adr übernehmen
#define	RF_LED_ON		0x06
#define	RF_LED_OFF		0x07
#define	RF_LED_TOGGLE	0x08
#define	RF_LED_AUTO		0x09
#define	RF_WAKE_UP		0x0A

#define Flag_1ms   0x01
#define Flag_5ms   0x02
#define Flag_10ms  0x04
#define Flag_25ms  0x08
#define Flag_50ms  0x10
#define Flag_100ms 0x20
#define Flag_250ms 0x40
#define Flag_500ms 0x80

#define RECEIVE			0			// Funkmodus Empfangen
#define TRANSMIT		1			// Funkmodus Senden
#define SLAVE           0
#define MASTER          1

#define OK          0
#define TxError     1
#define RxError     2

#define NORMAL_MODE			0
#define TEACH_MODE			1
#define CHANGE_ADDR_MODE	2
#define WAIT_FOR_REQ		3
#define WAIT_FOR_CLICK		4
#define CHANGE_RF_ADDR		5
#define WAIT_FOR_NAS_CLICK	6
#define RF_TEST_MODE		7

//#define SW1     			PINE_Bit6
//#define MaskEntpr  			0x07

#ifdef __ICCAVR__
#define SEI() asm("sei")                                    // global Int enable
#define CLI() asm("cli")                                    // global Int disable
#define NOP() asm("nop")
#endif
#define SETBIT(ADRESS,BIT) (ADRESS |= (1 << BIT))           // set a bit
#define CLEARBIT(ADRESS,BIT) (ADRESS &= ~(1 << BIT))        // clear a bit
#define CHECKBIT(ADRESS,BIT) (ADRESS & (1 << BIT))          // check a bit
#define SETFLAG(REG,FLAG) (REG |= (FLAG))
#define CLEARFLAG(REG,FLAG) (REG &= (~FLAG))
#define CHECKFLAG(REG,FLAG) (REG & (FLAG))
#define _enableRx1 UCSR1B |= (1 << RXEN1)                   // enable USART receive mode
#define _disableRx1 UCSR1B &= ~(1 << RXEN1)                 // diasble USART receive mode
#define _enableTx1 UCSR1B |= (1 << TXEN1)                   // enable USART transmit mode
#define _disableTx1 UCSR1B &= ~(1 << TXEN1)                 // disable USART transmit mode
#define _enableIntRx1 UCSR1B |= (1 << RXCIE1)               // enable USART receive Int
#define _disableIntRx1 UCSR1B &= ~(1 << RXCIE1)             // disable USART receive Int
#define _enableIntTx1 UCSR1B |= (1 << TXCIE1)               // enable USART transmit Int
#define _disableIntTx1 UCSR1B &= ~(1 << TXCIE1)             // disable USART transmit Int
#define _enableIntUDRE1 UCSR1B |= (1 << UDRIE1)              // enable USART UDR Int
#define _disableIntUDRE1 UCSR1B &= ~(1 << UDRIE1)            // disable USART UDR Int

typedef struct
{
 unsigned char Ent:3,
		      Lock:1,
		      Proc:1,
		      On:1,
		      Off:1,
		      Togg:1;
}BitButton;

typedef struct
{
 BitButton status;
// unsigned char time;
// unsigned char error;
}BStatus;

union Frame{
	struct
	{
		unsigned char length;
		unsigned char sequenz;
		unsigned char command;
		unsigned char data[29];
	};
	unsigned char daten[32];
};


void Init_SPI(void);
void Init_TimeBase(void);
//void ButtonStatus(void);
//void StartScript(void);
void Transmit_ID_Data(void);
unsigned char Wait_Tx(void);
//void SendFurniFrame(unsigned char adr, unsigned char datalength, unsigned char command, unsigned char *data);
//uint1 furniScriptCheckKeyPressed(unsigned char key);
//sint16 furniScriptGetFeedback(unsigned char addr_dev, unsigned char type);
//void furniScriptSendFurniPacket(unsigned char addr, unsigned char len, unsigned char cmd, unsigned char *data);
//void furniScriptLockStatus(uint8 filter, uint8 status);
//unsigned char WaitFrame(unsigned char t);
//void ErrorFunc(unsigned char dev);
//void FrameCopy(struct frame* q,struct frame* z);
//void SetRS485En(void);
//void ReleaseRS485En(void);


extern unsigned char Rx_mode;
extern volatile unsigned int TeachTimer;
extern volatile unsigned char RfTimer;

/** @endcond */
