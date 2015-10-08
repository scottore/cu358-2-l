#include "position.h"
#include "analog.h"
#include "eeprom.h"
#ifndef TOP_VALUE_M1
#warning TOP_VALUE_M1 not defined - reset disabled - upper limitswitch needs to be teached in by end-user...
#define TOP_VALUE_M1	0
#endif
#ifndef TOP_VALUE_M2
#warning TOP_VALUE_M2 not defined - reset disabled - upper limitswitch needs to be teached in by end-user...
#define TOP_VALUE_M2	0
#endif
#define	CURRENT_MIN_VALUE	 0x01
#define CURRENT_CAL_MIN_VALUE    0x03
#define CURRENT_MAX_VALUE        250    
#define CURRENT_COMPENSATE_NUM   200


__eeprom int16_t ee_pulseCounter[NUM_DRIVES];// = {0,0};							///< This is the current position of the actuators.
__eeprom int16_t ee_pulseTopValue[NUM_DRIVES];// = {TOP_VALUE_M1,TOP_VALUE_M2};	///< This is the position of the upper limit switches
__eeprom int16_t ee_pulseBottomValue[NUM_DRIVES];// = {-5974,-11918};				///< This is the position of the lower limit switches. This values is only used if the actuators has smart-sync-switches.
__eeprom union smartSyncState_t ee_smartSyncState;// = {0x00};

int16_t pulseCounter[NUM_DRIVES];
int16_t pulseCounterCompensate[NUM_DRIVES];

int16_t* motorCurrent;
volatile uint8_t pulseTime[NUM_DRIVES];
volatile uint8_t pulseStartTime[NUM_DRIVES];
union smartSyncState_t smartSyncState;
signed char rstFlag[NUM_DRIVES];

void Relais_Position_Timer_1ms(void)
{
  uint8_t drive;
  int16_t temp_value;
  static uint8_t pulseCountActive = 0;
  static int16_t pulseCounterCompensate[NUM_DRIVES] = {0,0};
  if(pulseCountActive)
  {
    ADC_SetCurrent();
    for(drive = 0; drive < NUM_DRIVES;drive++)
    {
      if(current[drive] < CURRENT_MIN_VALUE)
      {//电流约为0,驱动器关闭，延迟100ms，该100ms为继电器打开到关闭的时间。脉冲不计数。
        if(pulseTime[drive] < PULSETIME_MAX_VALUE)
          pulseTime[drive] ++;
        pulseStartTime[drive] = 0;
        pulseCounterCompensate[drive]  = 0;
      }
      else
      {
        pulseTime[drive] = 0;
        if(pulseStartTime[drive] < PULSETIME_MAX_VALUE)
          pulseStartTime[drive] ++;
      }
        //方向，启动时间常数，关闭时间
      if(driveState[drive] == moveUp && pulseStartTime[drive] > 2 && pulseTime[drive] < PULSETIME_MAX_VALUE )
      {
        pulseCounter[drive] ++;
        temp_value = (current[drive] > CURRENT_CAL_MIN_VALUE)?current[drive] : CURRENT_CAL_MIN_VALUE;
        temp_value = (current[drive] < CURRENT_MAX_VALUE)?current[drive] : CURRENT_MAX_VALUE;
        pulseCounterCompensate[drive]  += (CURRENT_MAX_VALUE - temp_value)/ temp_value;
        if(pulseCounterCompensate[drive]  > CURRENT_COMPENSATE_NUM)
        {
          pulseCounterCompensate[drive]  -= CURRENT_COMPENSATE_NUM;
          pulseCounter[drive] ++;
        }
      }
      else if(driveState[drive] == moveDown && pulseStartTime[drive] > 2 && pulseTime[drive] < PULSETIME_MAX_VALUE)
      {
        
        pulseCounter[drive] --;
       // temp_value = CURRENT_CAL_MIN_VALUE;
          temp_value = (current[drive] > CURRENT_CAL_MIN_VALUE)?current[drive] : CURRENT_CAL_MIN_VALUE;
          temp_value = (current[drive] < CURRENT_MAX_VALUE)?current[drive] : CURRENT_MAX_VALUE;
          pulseCounterCompensate[drive]  -= (CURRENT_MAX_VALUE - temp_value)/ temp_value;
          if(pulseCounterCompensate[drive]  < 0)
          {
            pulseCounterCompensate[drive]  += CURRENT_COMPENSATE_NUM;
            pulseCounter[drive] --;
          }
      }
    }
  }
  else
  {
    ADC_Start(0);
  }
  pulseCountActive = pulseCountActive^0x01;
}

void storePulses()
{
	// TODO check which IRQs really need to be disabled...
	__disable_irq();

	mem2eecpy((uint8_t*)ee_pulseCounter, (uint8_t *)pulseCounter, sizeof(ee_pulseCounter));
	mem2eecpy((uint8_t *)&ee_smartSyncState, (uint8_t *)&smartSyncState, sizeof(ee_smartSyncState));

	__enable_irq();
}

void loadPulses()
{
	__disable_irq();

	ee2memcpy((uint8_t*)pulseCounter, (uint8_t*)ee_pulseCounter, sizeof(pulseCounter));
	ee2memcpy((uint8_t*)&smartSyncState, (uint8_t*)&ee_smartSyncState, sizeof(smartSyncState));

	__enable_irq();
}

void resetPulseCounter(unsigned char drive)
{
	__disable_irq();

	pulseCounter[drive] = 0;

	__enable_irq();
}

void copyPulseCounterToEE(int16_t * ee_value, unsigned char drive)
{
	__disable_irq();

	mem2eecpy((uint8_t*)(ee_value + drive), (uint8_t*)(pulseCounter + drive), sizeof(int));

	__enable_irq();
}

void readPulseCounterFromEE(int16_t * ee_value, unsigned char drive)
{
	__disable_irq();

	pulseCounter[drive] = ee_read_value(&ee_value[drive]);

	__enable_irq();
}
void checkPulseReset()
{
	static union pulse_t lastPulse;
	union pulse_t newPulse;
	unsigned char i;
#ifdef USE_TIMECOUNT
	static signed char timeResetFlag[NUM_DRIVES];
#endif

#ifndef NO_SMART_SYNC
	newPulse.data = pulse.data;
	
	lastPulse.data ^= newPulse.data;
	
	if (lastPulse.m1mid != 0)
	{
		if (((driveDirection[0] == moving_up) && (newPulse.m1mid == 0))
		 || ((driveDirection[0] == moving_down) && (newPulse.m1mid != 0)))
		{
			resetPulseCounter(0);
			rstFlag[0] = -1;
			smartSyncState.aboveM1 = driveDirection[0] == moving_up;
		};
	};
	
	if (lastPulse.m2mid != 0)
	{
		if (((driveDirection[1] == moving_up) && (newPulse.m2mid == 0))
		 || ((driveDirection[1] == moving_down) && (newPulse.m2mid != 0)))
		{
			resetPulseCounter(1);
			rstFlag[1] = -1;
			smartSyncState.aboveM2 = driveDirection[1] == moving_up;
		};
	};
	
	lastPulse.data = newPulse.data;
#endif

#if (NUM_DRIVES == 2)
	if ((inRushTimer[0] == 0) && (inRushTimer[1] == 0))
	{
		if ((pulseTime[0] < PULSETIME_MAX_VALUE) && ((driveState[0] == moveUp) || (driveState[0] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM1 != 0)
			 && (smartSyncState.aboveM1 == 0)
			 && (driveState[0] == moveUp))
				rstFlag[0] = -1;
#endif
			smartSyncState.limitM1 = 0;
		};
		if ((pulseTime[1] < PULSETIME_MAX_VALUE) && ((driveState[1] == moveUp) || (driveState[1] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM2 != 0)
			 && (smartSyncState.aboveM2 == 0)
			 && (driveState[1] == moveUp))
				rstFlag[1] = -1;
#endif
			smartSyncState.limitM2 = 0;
		};
	}
#elif (NUM_DRIVES == 3)
	if ((inRushTimer[0] == 0) && (inRushTimer[1] == 0) && (inRushTimer[2] == 0))
	{
		if ((pulseTime[0] < PULSETIME_MAX_VALUE) && ((driveState[0] == moveUp) || (driveState[0] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM1 != 0)
			 && (smartSyncState.aboveM1 == 0)
			 && (driveState[0] == moveUp))
				rstFlag[0] = -1;
#endif
			smartSyncState.limitM1 = 0;
		};
		if ((pulseTime[1] < PULSETIME_MAX_VALUE) && ((driveState[1] == moveUp) || (driveState[1] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM2 != 0)
			 && (smartSyncState.aboveM2 == 0)
			 && (driveState[1] == moveUp))
				rstFlag[1] = -1;
#endif
			smartSyncState.limitM2 = 0;
		};
		if ((pulseTime[2] < PULSETIME_MAX_VALUE) && ((driveState[2] == moveUp) || (driveState[2] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM3 != 0)
			 && (smartSyncState.aboveM3 == 0)
			 && (driveState[2] == moveUp))
				rstFlag[2] = -1;
#endif
			smartSyncState.limitM3 = 0;
		};
	}
#elif (NUM_DRIVES == 4)
	if ((inRushTimer[0] == 0) && (inRushTimer[1] == 0) && (inRushTimer[2] == 0) && (inRushTimer[3] == 0))
	{
		if ((pulseTime[0] < PULSETIME_MAX_VALUE) && ((driveState[0] == moveUp) || (driveState[0] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM1 != 0)
			 && (smartSyncState.aboveM1 == 0)
			 && (driveState[0] == moveUp))
				rstFlag[0] = -1;
#endif
			smartSyncState.limitM1 = 0;
		};
		if ((pulseTime[1] < PULSETIME_MAX_VALUE) && ((driveState[1] == moveUp) || (driveState[1] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM2 != 0)
			 && (smartSyncState.aboveM2 == 0)
			 && (driveState[1] == moveUp))
				rstFlag[1] = -1;
#endif
			smartSyncState.limitM2 = 0;
		};
		if ((pulseTime[2] < PULSETIME_MAX_VALUE) && ((driveState[2] == moveUp) || (driveState[2] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM3 != 0)
			 && (smartSyncState.aboveM3 == 0)
			 && (driveState[2] == moveUp))
				rstFlag[2] = -1;
#endif
			smartSyncState.limitM3 = 0;
		};
		if ((pulseTime[3] < PULSETIME_MAX_VALUE) && ((driveState[3] == moveUp) || (driveState[3] == moveDown)))
		{
#ifdef NO_SMART_SYNC
			if ((smartSyncState.limitM4 != 0)
			 && (smartSyncState.aboveM4 == 0)
			 && (driveState[3] == moveUp))
				rstFlag[3] = -1;
#endif
			smartSyncState.limitM4 = 0;
		};
	}
#else
#error unbekannte Knofiguration
#endif
	
	for (i = 0; i < NUM_DRIVES; i++)
	{
#ifdef USE_TIMECOUNT
		switch (driveState[i])
		{
		  case moveDown:
//			if (inRushTimer[i] == 0)
//			{
				if (pulseTime[i] < MIN_PULSE_TIME)
				{
					timeResetFlag[i] = -1;
				}
//			}
			break;
		  default:
			timeResetFlag[i] = 0;
			break;
		}
#endif
		if ((inRushTimer[i] == 0)
		 && (current[i] <      CURRENT_MIN_VALUE )
		 && (current[i] > (0 - CURRENT_MIN_VALUE))
		 && (pulseTime[i] >= PULSETIME_MAX_VALUE))
		{
			if (rstFlag[i] != 0)
			{
				switch (driveState[i])
				{
				  case moveUp:
					copyPulseCounterToEE((int16_t*)ee_pulseTopValue, i);
					break;
				  case moveDown:
#ifndef NO_SMART_SYNC
					copyPulseCounterToEE((int16_t*)ee_pulseBottomValue, i);
#else
					resetPulseCounter(i);
#endif
#ifdef USE_TIMECOUNT
					if (timeResetFlag[i] != 0)
					{
						timerCounterSetCounter(i, TIMERCOUNTER_RESETVALUE);
					}
					timeResetFlag[i] = 0;
#endif
					break;
				};
				rstFlag[i] = 0;
			} else
			{
				switch (driveState[i])
				{
				  case moveUp:
					if (ee_read_value(&ee_pulseTopValue[i]) != 0)
						readPulseCounterFromEE((int16_t*)ee_pulseTopValue, i);
					break;
				  case moveDown:
#ifndef NO_SMART_SYNC
					if (ee_read_val(&ee_pulseBottomValue[i]) != 0)
						readPulseCounterFromEE((int16_t*)ee_pulseBottomValue,i);
#else
					resetPulseCounter(i);
#endif
#ifdef USE_TIMECOUNT
					if (timeResetFlag[i] != 0)
					{
						timerCounterSetCounter(i, TIMERCOUNTER_RESETVALUE);
					}
					timeResetFlag[i] = 0;
#endif
					break;
				};
			};

//			if (i == 0) { smartSyncState.limitM1 = 1; }
//			if (i == 1) { smartSyncState.limitM2 = 1; }
			
			switch (driveState[i])
			{
			  case moveUp:
				if (i == 0) { smartSyncState.aboveM1 = 1; smartSyncState.limitM1 = 1;}
				if (i == 1) { smartSyncState.aboveM2 = 1; smartSyncState.limitM2 = 1;}
#if (NUM_DRIVES >= 3)
				if (i == 2) { smartSyncState.aboveM3 = 1; smartSyncState.limitM3 = 1;}
#endif
#if (NUM_DRIVES >= 4)
				if (i == 3) { smartSyncState.aboveM4 = 1; smartSyncState.limitM4 = 1;}
#endif
				break;
			  case moveDown:
				if (i == 0) { smartSyncState.aboveM1 = 0; smartSyncState.limitM1 = 1;}
				if (i == 1) { smartSyncState.aboveM2 = 0; smartSyncState.limitM2 = 1;}
#if (NUM_DRIVES >= 3)
				if (i == 2) { smartSyncState.aboveM3 = 0; smartSyncState.limitM3 = 1;}
#endif
#if (NUM_DRIVES >= 4)
				if (i == 3) { smartSyncState.aboveM4 = 0; smartSyncState.limitM4 = 1;}
#endif
				break;
			};
		};
	};
}


int16_t getPulseCounter(unsigned char drive)
{
	int16_t retValue;

//	__disable_irq();

	retValue = pulseCounter[drive];

	//__enable_irq();

	return retValue;
}
