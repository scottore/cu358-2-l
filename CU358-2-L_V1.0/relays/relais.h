#ifndef _RELAIS_H_
#define _RELAIS_H_

#define NUM_DRIVES 2

#define	DRIVESTATE_TIMEOUT	10	// ca. 100ms

//#define	RELAIS_UP_TIME				9.5	//	9.0-8.2-8.0-7.8 - zu wenig
//#define	RELAIS_DOWN_TIME			0.5	//	1.6-2.4-2.6-2.8 - zu viel
#define	INRUSH_TIME					500
#define INRUSH_TIMERSPEED			10
#define CALC_DBL_PULSE_TIME(x)		((20.0 - x) / 2.0)
#define	CALC_PULSE_PAUSE_TIME(x)	((CALC_DBL_PULSE_TIME(x) * 3.0) + 20.0)

//	#define RELAIS_DOUBLE_PULSE_TIME	7	//	7 ms
//	#define RELAIS_PULSE_PAUSE_TIME		27	//	27 ms = 6ms Reaktions-Zeit des Relais + 14ms aus Messung berechnete Impulse + 14ms/2 Impulse die später doppelt gezählt werden.
#define PULSE_TIMER_SPEED			20	//	50us Interval --> 20kHz
#define RELAIS_DELAY_TIME			200	//	ca. 2 Sekunde...

void relaisTimer_10ms();
//void relaisTimer_1ms();
//void doRelais();
void setPWMup(unsigned char drive, unsigned char pwm);
void setPWMdown(unsigned char drive, unsigned char pwm);

enum driveDirection_t
{
	stopped,
	moving_down,
	moving_up
};


enum driveState_t
{
	stop = 0,
	waitUp = 1,
	moveUp = 2,
	stopUp = 3,
	waitDown = 4,
	moveDown = 5,
	stopDown = 6
};


extern enum driveDirection_t driveDirection[NUM_DRIVES];
extern enum driveState_t driveState[NUM_DRIVES];

extern unsigned char inRushTimer[NUM_DRIVES];
#endif