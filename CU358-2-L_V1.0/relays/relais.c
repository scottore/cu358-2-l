
#ifdef __ICCAVR__
#include <ioavr.h>
#include <inavr.h>

#elif  defined(STM8S003) || defined(STM8S103)
#include "stm8s_conf.h"
#include "position.h"
#else
#define	__tinyflash const
#include "stm32f0xx_gpio.h"
#include "hal.h"
#include "pulse.h"
#endif


#include "misc.h"
#include "relais.h"


enum driveState_t driveState[NUM_DRIVES];

enum driveDirection_t driveDirection[NUM_DRIVES];

unsigned char driveStateTimer[NUM_DRIVES];
unsigned char inRushTimer[NUM_DRIVES];
//signed int impulseDiff[NUM_DRIVES];
//signed int impulseLastCount[NUM_DRIVES];
//unsigned int pulsePauseTimer[NUM_DRIVES];

#ifdef RELAIS_DELAY
//unsigned char relaisDelayTimer;
unsigned int softStartTimer[NUM_DRIVES];
#endif

#define	SOFTSTART_MAXTIME	(40 * 100)	//	ca. 40 Sekunden...
#define	SOFTSTART_DELAY		(7 * 100)	//	ca. 7 Sekunden...

//unsigned char softStartState[NUM_DRIVES];
//#define	SOFTSTART_TIME	50
//                                                                 缩进减速                              伸出减速                   伸出减速                                           缩进减速
//                                                       停止         等待伸出             伸出          停止伸出                  等待缩进             缩进                     停止缩进
//Event:			\\\\\\\     State:	stop		waitUp		moveUp		stopUp		waitDown	moveDown	stopDown
enum driveState_t setUpTable[7]			=	{moveUp,	waitUp,		moveUp,		moveUp,		moveUp,		waitUp,		waitUp};
enum driveState_t setStopTable[7]		=	{stop,		stopDown,	stopUp,		stopUp,		stopUp,		stopDown,	stopDown};
enum driveState_t setDownTable[7]		=	{moveDown,	moveDown,	waitDown,	waitDown,	waitDown,	moveDown,	moveDown};
//延迟100ms
enum driveState_t setTimeoutTable[7]	=	{stop,		moveUp,		moveUp,		stop,		moveDown,	moveDown,	stop};

enum driveDirection_t setDirTable[7]	=	{stopped,	moving_down,moving_up,	moving_up,	moving_up,	moving_down,moving_down};
//__tinyflash enum driveDirection_t setDirTable[7]	=	{stopped,	stopped,	moving_up,	stopped,	stopped,	moving_down,stopped};

//	TODO...
#define	disable_relaisTimer_10ms();
#define	enable_relaisTimer_10ms();


#ifdef RELAIS_DELAY
__monitor void incSoftStartTimer(unsigned char drive)
{
	softStartTimer[drive] += SOFTSTART_DELAY;
}

__monitor signed char getSoftStartTimer(unsigned char drive)
{
	signed int retValue = softStartTimer[drive];
	retValue -= SOFTSTART_MAXTIME;
	
//	if (retValue > (255 - RELAIS_DELAY_TIME))
//		return (255 - RELAIS_DELAY_TIME);
	
	if (retValue > 0)
		return -1;

	return 0;
}
#endif

void setRelaisUp(unsigned char drive, signed char value)
{
	switch (drive)
	{
	  case 0:
		setM1up(value);
		break;
	  case 1:
		setM2up(value);
		break;
#if (NUM_DRIVES > 2)
	  case 2:
		setM3up(value);
		break;
#endif
#if (NUM_DRIVES > 3)
	  case 3:
		setM4up(value);
		break;
#endif
#if (NUM_DRIVES > 4)
#warning Implementierung fehlt!
#endif
	}
}

void setRelaisDown(unsigned char drive, signed char value)
{
	switch (drive)
	{
	  case 0:
		setM1down(value);
		break;
	  case 1:
		setM2down(value);
		break;
#if (NUM_DRIVES > 2)
	  case 2:
		setM3down(value);
		break;
#endif
#if (NUM_DRIVES > 3)
	  case 3:
		setM4down(value);
		break;
#endif
#if (NUM_DRIVES > 4)
#warning Implementierung fehlt!
#endif
	}
}
/*
void setMassage(unsigned char drive, signed char enabled)
{
	switch (drive)
	{
	  case 1:
		if (enabled == 0)
		{
			_MASSAGE_1 = 0;
		} else
		if (enabled == 1)
		{
			_MASSAGE_1 = 255;
		} else
		{
			_MASSAGE_1 = MASSAGE_1;
		}
		break;
	  case 0:
		if (enabled == 0)
		{
			_MASSAGE_2 = 0;
		} else
		if (enabled == 1)
		{
			_MASSAGE_2 = 255;
		} else
		{
			_MASSAGE_2 = MASSAGE_2;
		}
		break;
#if (NUM_DRIVES > 2)
	  case 2:
		if (enabled == 1)
		{
			PORTD_Bit7 = 1;
		} else
		{	
			PORTD_Bit7 = 0;
		}
		break;
#endif
#if (NUM_DRIVES > 3)
	  case 3:
		if (enabled == 1)
		{
			PORTB_Bit3 = 1;
		} else
		{
			PORTB_Bit3 = 0;
		}
		break;
#endif
#if (NUM_DRIVES > 4)
#warning Implementierung fehlt!
#endif
	}
}
*/

#ifdef IS_AWAR_OF_LOWER_LIMITSWITCH
int_fast16_t getLowerLimitSwitch(uint_fast8_t drive)
{
	switch (drive)
	{
	case 0:
		return M1_LIMIT_SWITCH_BOTTOM;
	case 1:
		return M2_LIMIT_SWITCH_BOTTOM;
#if NUM_DRIVES > 2
	case 2:
		return M3_LIMIT_SWITCH_BOTTOM;
#endif
	}
#if NUM_DRIVES > 4
#error not implemented
#endif

	return 0;
}
#endif

void setRelais()
{
	unsigned char drive;

	
	disable_relaisTimer_10ms();

	for (drive = 0; drive < NUM_DRIVES; drive++)
	{
		switch (driveState[drive])
		{
		  case waitUp:
		  case stopDown:
	//		if (hall.m1end != 0)
#ifdef IS_AWAR_OF_LOWER_LIMITSWITCH
			if (getLowerLimitSwitch(drive) != 0)
#endif
			{
				setRelaisDown(drive, -1);
				setRelaisUp(drive, -1);
				resetRstFlag(drive);
				break;
			};

		  default:
			setRelaisDown(drive, 0);
			setRelaisUp(drive, 0);
			resetRstFlag(drive);
			break;
		  case moveUp:
			setRelaisDown(drive, 0);
			setRelaisUp(drive, -1);
			break;
		  case moveDown:
			setRelaisDown(drive, -1);
			setRelaisUp(drive, 0);
			break;
		};
	};
	
#ifdef POWER_RELAIS
#error changed to POWER_RELAIS_set()
#endif
#ifdef POWER_RELAIS_set
	i = NUM_DRIVES;
	while (i--)
	{
		if (driveState[i] != stop) break;
	}

	//POWER_RELAIS = i != -1 ? 1 : 0;
	POWER_RELAIS_set(i != -1 ? 1 : 0);
/*
	for (drive = 0; drive < NUM_DRIVES; drive++)
	{
		switch (driveState[drive])
		{
		  case stop:
			if (i == -1)
			{
				setMassage(drive, 2);
			} else
			{
				setMassage(drive, 0);
			};
			softStartState[drive] = SOFTSTART_TIME;
			break;
//		  case waitUp:
//		  case stopDown:
//		  case waitDown:
//		  case stopUp:
		  default:
			setMassage(drive, 0);
			softStartState[drive] = SOFTSTART_TIME;
			break;
		  case moveUp:
		  case moveDown:
			if (softStartState[drive] == 0)
			{
				setMassage(drive, 1);
			} else
			{
				setMassage(drive, 0);
			}
			break;
		};
	};
*/
#endif
	
	
	enable_relaisTimer_10ms();
}

void relaisTimer_10ms()
{
  unsigned char i;
  enum driveState_t state;
	
  for (i = 0 ; i < NUM_DRIVES; i++)
  {
    if (inRushTimer[i] != 0)
      inRushTimer[i]--;

    state = driveState[i];

    if (driveStateTimer[i] != 0)
    {
      driveStateTimer[i]--;
    } 
    else
    {
      state = driveState[i];
      if (state > 6) state = stop;
			
      switch(state)
      {
        case waitUp:
        case waitDown:
        case stopUp:
        case stopDown:
        //calcPulsePause(i);
        break;
      };
			
      driveState[i] = setTimeoutTable[state];
      if (driveState[i] != state)
      {
        inRushTimer[i] = INRUSH_TIME / INRUSH_TIMERSPEED;
#ifdef RELAIS_DELAY
//				relaisDelayTimer = RELAIS_DELAY_TIME;
//				relaisDelayTimer += getSoftStartTimer(drive);
        if ((state == moveDown) || (state == moveUp))
          incSoftStartTimer(i);
#endif
      };
    };
		
    state = driveState[i];
    driveDirection[i] = setDirTable[state];
    
#ifdef RELAIS_DELAY
    if (softStartTimer[i] != 0)
      softStartTimer[i]--;
#endif
  };
	
  setRelais();
	
#ifdef RELAIS_DELAY
//	if (relaisDelayTimer != 0)
//		relaisDelayTimer--;
#endif
}

/*
__monitor void calcUpPause(unsigned char drive)
{
	pulseCounter[drive] += impulseDiff[drive];
	pulseCounter[drive] += impulseDiff[drive];
	pulsePauseTimer[drive] = CALC_PULSE_PAUSE_TIME(RELAIS_UP_TIME) * PULSE_TIMER_SPEED;
}

__monitor void calcDownPause(unsigned char drive)
{
	pulseCounter[drive] += impulseDiff[drive];
	pulseCounter[drive] += impulseDiff[drive];
	pulsePauseTimer[drive] = CALC_PULSE_PAUSE_TIME(RELAIS_DOWN_TIME) * PULSE_TIMER_SPEED;
}
*/


void setPWMup(unsigned char drive, unsigned char pwm)
{
	enum driveState_t state;
	enum driveState_t lastState;
	
	state = driveState[drive];
	if (state > 6) state = stop;

#ifdef RELAIS_DELAY	
	if ((pwm != 0)
//	 && ((relaisDelayTimer == 0) || (state == moveUp)))
	 && ((getSoftStartTimer(drive) == 0) || (state == moveUp)))
#else
	if (pwm != 0)
#endif
	{
		lastState = state;
		state = setUpTable[state];
		
		if (lastState != state)
		{
                  inRushTimer[drive] = INRUSH_TIME / INRUSH_TIMERSPEED;
#ifdef RELAIS_DELAY			
			if (state == moveUp)
				incSoftStartTimer(drive);
#endif
		}

	}
	else
	{
		state = setStopTable[state];
	};
	
	if (driveState[drive] != state)
		driveStateTimer[drive] = DRIVESTATE_TIMEOUT;
	
	driveState[drive] = state;

	setRelais();
}

void setPWMdown(unsigned char drive, unsigned char pwm)
{
	enum driveState_t state;
	enum driveState_t lastState;
	
//	if (drive >= NUM_DRIVES)	return;
/*
	switch(drive)
	{
	  case 0:
		if (hall.m1end == 0)
			pwm = 0;
		break;
	  case 1:
		if (hall.m2end == 0)
			pwm = 0;
		break;
	  default:
		return;
	};
*/
	
	state = driveState[drive];
	if (state > 6) state = stop;
	
#ifdef RELAIS_DELAY
	if ((pwm != 0)
//	 && ((relaisDelayTimer == 0) || (state == moveDown)))
	 && ((getSoftStartTimer(drive) == 0) || (state == moveDown)))
#else
	if (pwm != 0)
#endif
	{
		lastState = state;
		state = setDownTable[state];
		
		if (lastState != state)
		{
                  inRushTimer[drive] = INRUSH_TIME / INRUSH_TIMERSPEED;

#ifdef RELAIS_DELAY
//			relaisDelayTimer = RELAIS_DELAY_TIME;
//			relaisDelayTimer += getSoftStartTimer(drive);
			if (state == moveDown)
				incSoftStartTimer(drive);
#endif
		}
	}
	else
	{
//		if (state == moveUp)
//			calcUpPause(drive);
//		if (state == moveDown)
//			calcDownPause(drive);

//		pulsePauseTimer[drive] = 0;

		state = setStopTable[state];
	};
	
	if (driveState[drive] != state)
		driveStateTimer[drive] = DRIVESTATE_TIMEOUT;
	
	driveState[drive] = state;
	
	setRelais();
}


