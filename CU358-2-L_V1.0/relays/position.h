#ifndef _POSITION_H_
#define _POSITION_H_
#include "relais.h"
#include "stm8s_conf.h"

#define __disable_irq disableInterrupts  
#define __enable_irq enableInterrupts  
union smartSyncState_t
{
	unsigned char data;
	struct
	{
		unsigned char aboveM1 : 1;
		unsigned char aboveM2 : 1;
		unsigned char aboveM3 : 1;
		unsigned char aboveM4 : 1;
		
		unsigned char limitM1 : 1;
		unsigned char limitM2 : 1;
		unsigned char limitM3 : 1;
		unsigned char limitM4 : 1;
	};
};
union pulse_t
{
	unsigned char data;
	struct
	{
		unsigned char m1pulse : 1;
		unsigned char m2pulse : 1;
		unsigned char m3pulse : 1;
		unsigned char m4pulse : 1;
		
		unsigned char dummy : 4;
	};
};
extern signed char rstFlag[NUM_DRIVES];
//extern union pulse_t pulse;
extern  int16_t pulseCounter[NUM_DRIVES];
extern volatile uint8_t pulseTime[];

#define	PULSETIME_MAX_VALUE	50

#define resetRstFlag(drive)	{rstFlag[drive] = 0;}
void storePulses(void);
void loadPulses(void);
void checkPulseReset(void);
void resetPulseCounter(unsigned char drive);
int16_t getPulseCounter(unsigned char drive);
void copyPulseCounterToEE(int16_t * ee_value, unsigned char drive);
void readPulseCounterFromEE(int16_t * ee_value, unsigned char drive);
void Relais_Position_Timer_1ms(void);
#endif