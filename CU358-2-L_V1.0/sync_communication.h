#ifndef _SYNC_COMMUNICATION_H_
#define _SYNC_COMMUNICATION_H_
#include "misc.h"
#define ANGLE_POSITION_V2   0x00000020

extern volatile unsigned char syncComWaitTimer;
//extern unsigned char syncComTimeoutTimer;

extern int16_t pulseCounterSync[NUM_DRIVES];
#ifdef USE_TIMECOUNT
extern unsigned int timeCounterSync[NUM_DRIVES];
extern unsigned char syncPulseTime[NUM_DRIVES];
#endif
extern signed char syncActive;
extern signed char btActive;

extern unsigned long syncLEDs;
extern unsigned char syncStatus;
extern unsigned long syncKeysReceived;
extern unsigned long syncBtKeysReceived;
extern uint8_t syncBtpulseIndex;
extern signed int syncBtpulseCounter[NUM_DRIVES];
extern unsigned long syncKeysTransmitted;

extern volatile enum usageMode_t syncUsageMode;

#define	MYADDR_SIZE	1							///	size of myAddr in Bit
#define MYADDR_MASK	((1 << MYADDR_SIZE) - 1)
extern uint8_t	myAddr;
#if (MYADDR_SIZE > 8)
#error type of myAddr to small!
#endif

void syncInit();
void syncCom10msTimer();
void syncComDo(unsigned long* RfButtons);

void mfpResetInBuffer(void);

void uart1TrvCompleteCallBack(void);
void uart1ReceiveCallback(uint8_t data);
void uart1Init(u8 SYS_Clk, u32 baud);

#endif