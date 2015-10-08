#ifdef __ICCAVR__
#include "main.h"
#include "toolbox\eeprom.h"
#else
//#include "eeprom.h"	//	is already included with hal.h

#endif

#include <stdio.h>
#include <string.h>


#include "nrf24l01/nrf24l01.h"
#include "nrf24l01/rf-protocol.h"
//#include "nrf24l01\elegance_t.h"
#include "nrf24l01\ergomotion-hs.h"

#ifdef __ICCAVR__
//#include "hardware.h"
#include "relais.h"
#include "pulse.h"
#include "steuerung.h"
#include "sync_communication.h"
#else
#include "relais.h"
#include "eeprom.h"
#include "steuerung.h"
#include "sync_communication.h"
#include "position.h"
#endif
#ifdef USE_TIMECOUNT
#include "timerCount.h"
#endif
#include "misc.h"

#define	ON_OFF_ON_DELAY			100		//	ca. 500ms...
#define	KEY_TIMEOUT				150		//	ca. 2000ms...
#define	COMFORT_TIMEOUT			200		//	ca. 2000ms...
#define	UESA_TIMEOUT			500		//	ca. 5000ms...
//#define	INRUSH_TIME				50		//	ca. 500ms...
#define	MASSAGE_TIME			90000ul	//	ca. 15min = 15*60*100
#define	POWER_TIMEOUT			150		//	ca. 150 ms
#define	ALL_FLAT_PULSE_TIMEOUT	20		//	ca. 200 ms
#define SYNC_TIMEOUT			150		//	ca. 1500 ms
#define MEM_STORE_DELAY			50		//	ca. 5000 ms -> ca. 5 sec
#define MEM_RESTORE_DELAY		50		//	ca. 5000 ms -> ca. 5 sec

#define NEW_TIMER_LED_STATUS    12

#define	POS_DIFF	50
#define TIME_DIFF	20


#ifdef ASHLEY

//#define POS_M1		3875
//#define	POS_M2		4322

#define POS_M1			0x145C
#define POS_M2			0x2AB4
#define LOUNGE_M1		0x3279
#define LOUNGE_M2		0x1CBF
#define TV_PC_M1		0x373A
#define	TV_PC_M2		0x2976
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001

#elif defined(ERGO400) || defined(ERGO400_2)

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

#define POS_M1			CALC_M1_FROM_mm(55)
#define POS_M2			CALC_M2_FROM_mm(65)
#define LOUNGE_M1		CALC_M1_FROM_mm(117)	// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define LOUNGE_M2		CALC_M2_FROM_mm(35)		// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define TV_PC_M1		CALC_M1_FROM_mm(207)	// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define	TV_PC_M2		CALC_M2_FROM_mm(60)		// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001

#elif defined(ERGO400JAPAN)

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

#define POS_M1			CALC_M1_FROM_mm(67)
#define POS_M2			CALC_M2_FROM_mm(63)
#define LOUNGE_M1		CALC_M1_FROM_mm(117)	// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define LOUNGE_M2		CALC_M2_FROM_mm(35)		// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define TV_PC_M1		CALC_M1_FROM_mm(207)	// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define	TV_PC_M2		CALC_M2_FROM_mm(60)		// nicht definiert f榛�400er Systeme - Kopiert vom 300er System (debug)
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001

#elif defined(TEMPUR_500)

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

// Werte vom 22.11.2012 - V1.13 - Besuch mit Ergomotion in China
#define POS_M1			CALC_M1_FROM_mm(69)
#define POS_M2			CALC_M2_FROM_mm(63)
#define LOUNGE_M1		CALC_M1_FROM_mm(156)
#define LOUNGE_M2		CALC_M2_FROM_mm(49)
#define TV_PC_M1		CALC_M1_FROM_mm(215)
#define	TV_PC_M2		CALC_M2_FROM_mm(69)
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001

#elif (NUM_DRIVES == 4)	// ergomotion 900

/*	300 & 900
 *			Zero G	TV/PC	Lounge
 *	Head	52mm	207mm	117mm
 *	Feet	68mm	60mm	35mm
 */
// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

// Werte vom 22.11.2012 - V1.13 - Besuch mit Ergomotion in China
#define POS_M1			CALC_M1_FROM_mm(52)
#define POS_M2			CALC_M2_FROM_mm(68)
#define LOUNGE_M1		CALC_M1_FROM_mm(117)
#define LOUNGE_M2		CALC_M2_FROM_mm(35)
#define TV_PC_M1		CALC_M1_FROM_mm(207)
#define	TV_PC_M2		CALC_M2_FROM_mm(60)
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001

/* Werte vor V1.13
#define POS_M1			4010	//	57mm
#define POS_M2			8576	//	62mm
#define LOUNGE_M1		8695	//	124mm
#define LOUNGE_M2		3994	//	29mm
#define TV_PC_M1		14348	//	207mm
#define	TV_PC_M2		6827	//	49mm
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001
*/
//TOP_VALUE_M1=15910
//TOP_VALUE_M2=11515

/*
alte Werte...
TOP_VALUE_M1=13729
TOP_VALUE_M2=9943
*/

#elif defined(SW1002929)

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

#define POS_M1			CALC_M1_FROM_mm(47)
#define POS_M2			CALC_M2_FROM_mm(67)
#define LOUNGE_M1		CALC_M1_FROM_mm(88)
#define LOUNGE_M2		CALC_M2_FROM_mm(43)
#define TV_PC_M1		CALC_M1_FROM_mm(150)
#define	TV_PC_M2		CALC_M2_FROM_mm(58)
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001


#elif defined(SERIE330)

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

#define POS_M1			CALC_M1_FROM_mm(52)
#define POS_M2			CALC_M2_FROM_mm(68)
#define LOUNGE_M1		CALC_M1_FROM_mm(117)
#define LOUNGE_M2		CALC_M2_FROM_mm(35)
#define TV_PC_M1		CALC_M1_FROM_mm(207)
#define	TV_PC_M2		CALC_M2_FROM_mm(60)
#define ANTI_SNORE_M1	CALC_M1_FROM_mm(72)
#define ANTI_SNORE_M2	0x8001	// unterer Endschalter


#elif defined (SW1003008)

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

//Werte aus KOM08232
#define POS_M1			CALC_M1_FROM_mm(45)
#define POS_M2			CALC_M2_FROM_mm(28)
#define LOUNGE_M1		CALC_M1_FROM_mm(121)
#define LOUNGE_M2		CALC_M2_FROM_mm(17)
#define TV_PC_M1		CALC_M1_FROM_mm(215)
#define	TV_PC_M2		CALC_M2_FROM_mm(24)
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001


#elif defined(RAVEN)

// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
// 1 Motorumdrehung = 12 Impulse
#define CALC_M1_FROM_mm(x)	((35*12/3)*x)
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

// Werte vom 22.11.2012 - V1.13 - Besuch mit Ergomotion in China
#define MEM_POS_1	{CALC_M1_FROM_mm(26), CALC_M2_FROM_mm(42), 0x8001}		//	"Z-Grav."-key
#define MEM_POS_2	{CALC_M1_FROM_mm(81), CALC_M2_FROM_mm(33), 0x8001}		//	"TV"-key
#define MEM_POS_3	{0x8001, 0x8001, 0x8001}								//	"Custom"-key -> flat
#define MEM_POS_4	{0x8001, 0x8001, 0x8001}

#if (NUM_MEM_DRIVES != 3)
#error	must be 3...
#endif

#elif defined(LP_POSITIONS)

// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
// 1 Motorumdrehung = 12 Impulse
#define CALC_M1_FROM_mm(x)	((35*12/3)*x)
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

// Werte vom 22.11.2012 - V1.13 - Besuch mit Ergomotion in China
#define POS_M1			CALC_M1_FROM_mm(25.2571428571428)
#define POS_M2			CALC_M2_FROM_mm(68)
#define LOUNGE_M1		CALC_M1_FROM_mm(56.8285714285714)
#define LOUNGE_M2		CALC_M2_FROM_mm(35)
#define TV_PC_M1		CALC_M1_FROM_mm(100.542857142857)
#define	TV_PC_M2		CALC_M2_FROM_mm(60)
#define ANTI_SNORE_M1	CALC_M1_FROM_mm(31.8785714285714)
#define ANTI_SNORE_M2	0x8001


#else	// ergomotion 300

// 34 Motorumdrehungen = 1 Spindelumdrehung
// 1 Motorumdrehung = 12 Impulse
// 1 Spindelumdrehung = 6 mm
#define CALC_M1_FROM_mm(x)	((34*12/6)*x)
// 35 Motorumdrehungen = 1 Spindelumdrehung
// 1 Spindelumdrehung = 3mm
#define CALC_M2_FROM_mm(x)	((35*12/3)*x)

// Werte vom 22.11.2012 - V1.13 - Besuch mit Ergomotion in China
#define POS_M1			3975 //CALC_M1_FROM_mm(52)
#define POS_M2			4856 //CALC_M2_FROM_mm(68)
#define LOUNGE_M1		8830 //CALC_M1_FROM_mm(117)
#define LOUNGE_M2		4105 //CALC_M2_FROM_mm(35)
#define TV_PC_M1		13398 //CALC_M1_FROM_mm(207)
#define	TV_PC_M2		5314 //CALC_M2_FROM_mm(60)
#define ANTI_SNORE_M1	3723 //0x116F
#define ANTI_SNORE_M2	-1000 //0x8001

#ifdef USE_TIMECOUNT
__no_init __eeprom unsigned int ee_timePosition[4][2];// =
//{
//	{TIMERCOUNTER_RESETVALUE + 600,		TIMERCOUNTER_RESETVALUE + 1600},	//	Zero-G
//	{TIMERCOUNTER_RESETVALUE + 1300,	TIMERCOUNTER_RESETVALUE + 900},	//	Lounge
//	{TIMERCOUNTER_RESETVALUE + 2200,	TIMERCOUNTER_RESETVALUE + 1400},	//	TV
//	{TIMERCOUNTER_RESETVALUE + 100,		TIMERCOUNTER_RESETVALUE + 100}	//	Anti-Snore
//};
#endif

/* Werte vor V1.13
#define POS_M1			4010	//	57mm
#define POS_M2			8576	//	62mm
#define LOUNGE_M1		8695	//	124mm
#define LOUNGE_M2		3994	//	29mm
#define TV_PC_M1		14348	//	207mm
#define	TV_PC_M2		6827	//	49mm
#define ANTI_SNORE_M1	0x116F
#define ANTI_SNORE_M2	0x8001
*/
//TOP_VALUE_M1=15910
//TOP_VALUE_M2=11515

#endif


#ifndef MEM_POS_1

 __eeprom int16_t ee_memoryPosition[4][2];// =
//{
//	{POS_M1,		POS_M2},
//	{LOUNGE_M1,		LOUNGE_M2},
//	{TV_PC_M1,		TV_PC_M2},
//	{ANTI_SNORE_M1,	ANTI_SNORE_M2}
//};

const int16_t ee_originalPosition[4][2] =
{
	{POS_M1,		POS_M2},
	{LOUNGE_M1,		LOUNGE_M2},
	{TV_PC_M1,		TV_PC_M2},
	{ANTI_SNORE_M1,	ANTI_SNORE_M2}
};

#else

//EEMEM int16_t ee_memoryPosition[4][NUM_MEM_DRIVES] = {MEM_POS_1, MEM_POS_2, MEM_POS_3, MEM_POS_4};
//EEMEM int16_t ee_originalPosition[4][NUM_MEM_DRIVES] = {MEM_POS_1, MEM_POS_2, MEM_POS_3, MEM_POS_4};

#endif
/*
EEMEM char ee_massage[4][3] =
{
	{0, 0, 0},	//	kopf, fu锟�zeit
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0}
};

EEMEM uint8_t ee_waveStatus[4] = {0, 0, 0, 0};

EEMEM char ee_UBB[4] = {0, 0, 0, 0};
*/
//volatile uint8_t onOffOnDelay[NUM_DRIVES];
volatile unsigned char allFlatTimer = 0;
volatile unsigned int allFlatTimeoutTimer = 0;
//volatile unsigned char inRushTimer = 0;
//volatile unsigned int uesaTimer = 0;
//volatile uint32_t massageTimer = 0;
volatile uint16_t moveToTimer[NUM_MEM_DRIVES];
volatile unsigned char syncTimeout[NUM_DRIVES];
volatile unsigned char memTimer;
volatile char blinkState;
volatile unsigned char feedbackTimer = 0;
//volatile unsigned char keyChangedTimer;

#ifdef RAVEN
extern int_fast8_t	ravenM3active;
#endif

enum StopAll_t stopAll					=	inactive;
signed char automaticMovementIsActive	=	0;

#ifdef SOFTSTART_SOFTSTOP_MASSAGE
volatile uint_fast16_t	newMassageValue[2];
#endif

union MData_t
{
	struct
	{
		uint16_t _buffer_1:8;

		uint16_t m1up:1;
		uint16_t m1down:1;
		uint16_t m2up:1;
		uint16_t m2down:1;
		uint16_t m3up:1;
		uint16_t m3down:1;
		uint16_t m4up:1;
		uint16_t m4down:1;
	};
	struct
	{
		uint16_t _buffer_2:8;

		uint16_t m1:2;
		uint16_t m2:2;
		uint16_t m3:2;
		uint16_t m4:2;
	};
	uint16_t data;
};





enum KeyState_t
{
	none			=	0,
	programm		=	1,
	intensity		=	2,
	periode			=	3,
	storePos		=	4,
	storeComfort	=	5,
	mem1toggle,
	mem2toggle,
	mem3toggle,
	mem4toggle,
	f1toggle,
	f2toggle,
	f3toggle
	/*
	mem1on			=	6,
	mem1off			=	7,
	mem2on			=	8,
	mem2off			=	9,
	mem3on			=	10,
	mem3off			=	11,
	mem4on			=	12,
	mem4off			=	13,
	f1on			=	14,
	f1off			=	15,
	f2on			=	16,
	f2off			=	17,
	f3on			=	18,
	f3off			=	19
	*/
};


//__eeprom volatile enum usageMode_t ee_usageMode = pressAndHold;

enum state_t
{
	idle			=	0,
	storePosition
};

#ifdef TOGGLE_LEDS
#define	LED_TOGGLE_TIME	50	//	ca. 500ms...
#define	setLEDtoggle(code, count)	{ledToggleCode = code; ledToggleTimer = LED_TOGGLE_TIME; ledToggleCount = count;}
unsigned long long	ledToggleCode;
unsigned char		ledToggleCount = 0;
unsigned char 		ledToggleTimer	=	LED_TOGGLE_TIME;
#else
#define	setLEDtoggle(code, count)
#endif



#if (NUM_MEM_DRIVES == 2)
int16_t memPos[NUM_MEM_DRIVES] = {MEM_OFF, MEM_OFF};
#else
#error not implemented...
#endif


#ifdef USE_TIMECOUNT
unsigned int timePos[2]	=	{TIMERCOUNTER_TIMEOFF,	TIMERCOUNTER_TIMEOFF};
#endif


unsigned char massageStartTimer[2];
#define MASSAGE_START_TIME	20	//	ca. 200ms
#define MASSAGE_START_VALUE	35	//	bei 75 starten die meisten Antriebe zuverl閶漵ig...

#ifdef HAS_WAVE_BUTTON
uint_fast8_t waveStatus = 0;
int_fast32_t waveValue = 0;
#endif


void stopMemory(void)
{
	uint8_t i;
	for (i = 0; i < NUM_MEM_DRIVES; i++)
	{
		memPos[i] = MEM_OFF;
	}
}

uint8_t getMemoryActive(void)
{
	uint8_t i = NUM_MEM_DRIVES - 1;
	do
	{
		if (memPos[i] != MEM_OFF)
			break;
	} while (i--);
	i++;
	return i;
}

void loadMemory(unsigned char posNo)
{
	ee2memcpy((unsigned char*)memPos, (unsigned char*)ee_memoryPosition[posNo], sizeof(memPos));

#ifdef STORE_UBB
	UBB_set(ee_read_value(&ee_UBB[posNo]));
#endif
	
#ifdef STORE_MASSAGE
	massage_status[0] = ee_read_value(&ee_massage[posNo][0]);
	massage_status[1] = ee_read_value(&ee_massage[posNo][1]);

	switch(ee_read_value(&ee_massage[posNo][2]))
	{
	  case 0:
		massageTimer = 0;
		break;
	  case 1:
		massageTimer = MASSAGE_TIME_10;
		break;
	  case 2:
		massageTimer = MASSAGE_TIME_20;
		break;
	  case 3:
		massageTimer = MASSAGE_TIME_30;
		break;
	};
#ifdef HAS_WAVE_BUTTON
	waveStatus = ee_read_value(&ee_waveStatus[posNo]);
#endif
#endif
}

void storeMemory(unsigned char posNo)
{
	ee_write_value((unsigned char*)&ee_memoryPosition[posNo][0], getPulseCounter(0));
	ee_write_value((unsigned char*)&ee_memoryPosition[posNo][1], getPulseCounter(1));

#ifdef STORE_UBB
	ee_write_value(&ee_UBB[posNo], UBB_get());
#endif
	
#ifdef STORE_MASSAGE
	ee_write_value(&ee_massage[posNo][0], massage_status[0]);
	ee_write_value(&ee_massage[posNo][1], massage_status[1]);
	
//	if ((massage_status[0] == 0)
//	 && (massage_status[1] == 0))
	if (massageTimer == 0)
	{
		ee_write_value(&ee_massage[posNo][2], 0);
	} else
	if (massageTimer <= MASSAGE_TIME_10)
	{
		ee_write_value(&ee_massage[posNo][2], 1);
	} else
	if (massageTimer <= MASSAGE_TIME_20)
	{
		ee_write_value(&ee_massage[posNo][2], 2);
	} else
	//if (massageTimer <= MASSAGE_TIME_30)
	{
		ee_write_value(&ee_massage[posNo][2], 3);
	};
#ifdef HAS_WAVE_BUTTON
	ee_write_value(&ee_waveStatus[posNo], waveStatus);
#endif
#endif
}

void restoreMemory(unsigned char posNo)
{
	ee_write_value((unsigned char*)&ee_memoryPosition[posNo][0], (int16_t)ee_read_value(&ee_originalPosition[posNo][0]));
	ee_write_value((unsigned char*)&ee_memoryPosition[posNo][1], (int16_t)ee_read_value(&ee_originalPosition[posNo][1]));
	
#ifdef STORE_UBB
	ee_write_value(&ee_UBB[posNo], 0);
#endif
	
#ifdef STORE_MASSAGE
	ee_write_value(&ee_massage[posNo][0], 0);
	ee_write_value(&ee_massage[posNo][1], 0);
	ee_write_value(&ee_massage[posNo][2], 0);
#ifdef HAS_WAVE_BUTTON
	ee_write_value(&ee_waveStatus[posNo], 0);
#endif
#endif
}
/*
void resetMemory(unsigned char posNo)
{
	ee_write_value((unsigned char*)&ee_memoryPosition[posNo][0], (int16_t)ee_read_value(&flash_originalPosition[posNo][0]));
	ee_write_value((unsigned char*)&ee_memoryPosition[posNo][1], (int16_t)ee_read_value(&flash_originalPosition[posNo][1]));
	ee_write_value((unsigned char*)&ee_originalPosition[posNo][0], (int16_t)ee_read_value(&flash_originalPosition[posNo][0]));
	ee_write_value((unsigned char*)&ee_originalPosition[posNo][1], (int16_t)ee_read_value(&flash_originalPosition[posNo][1]));
#ifdef STORE_UBB
	ee_write_value(&ee_UBB[posNo], 0);
#endif
	
#ifdef STORE_MASSAGE
	ee_write_value(&ee_massage[posNo][0], 0);
	ee_write_value(&ee_massage[posNo][1], 0);
	ee_write_value(&ee_massage[posNo][2], 0);
#ifdef HAS_WAVE_BUTTON
	ee_write_value(&ee_waveStatus[posNo], 0);
#endif
#endif
}*/
void SteuerungFunkInit(void)
{
  unsigned char i;
  for(i = 0; i < 4; i++)
  {
    loadMemory(i);
#if defined STM8S003
    if(memPos[0] == 0 && memPos[1] == 0)
      restoreMemory(i);
#elif defined STM8S103
    if(memPos[0] == 0 && memPos[1] == 0)
      restoreMemory(i);
#endif
  }
  memPos[0] = MEM_OFF;
  memPos[1] = MEM_OFF;
}
void doSteuerung(unsigned long* RfButtons)
{
	union Ergomation_keys_t keys;
	static union Ergomation_keys_t lastKeys;
//	static unsigned char storePosKeyCount;
//	static unsigned char storeComfortKeyCount;
	static unsigned char nasKeyCount = 0;
	static unsigned char lastNasTaster;
//	static unsigned char lastMemKey = 0;
//	static unsigned char lastComfortMemKey = 0;
//	enum KeyState_t lastState;
	union MData_t mData;
	static union MData_t mDataCurrent;
	unsigned char i;
	unsigned char drive;
	int16_t lastMemPos[NUM_MEM_DRIVES];
	//static signed char memoryStopped = 0;
	//static enum {none, memKeyPressed, memKeyStored} blinkMode;
//	static enum state_t currentState;
	
//	static enum usageMode_t oldSyncUsageMode = unknown;
	
#ifdef USE_TIMECOUNT
	unsigned int lastTimePos1;
	unsigned int lastTimePos2;
	static signed char useTimer[2];
	
#if (NUM_DRIVES != 2)
#error todo...
#endif
	
	if (((driveState[0] == moveDown) || (driveState[0] == moveUp))
	 && (inRushTimer[0] == 0))
	{
		if ((pulseTime[0] >= MIN_PULSE_TIME)	// normal ist 16 oder 17....
		 || ((syncActive != 0) && (syncPulseTime[0] >= MIN_PULSE_TIME)))
		{
			useTimer[0] = -1;
		} else
		{
			useTimer[0] = 0;
		}
	}

	if (((driveState[1] == moveDown) || (driveState[1] == moveUp))
	 && (inRushTimer[1] == 0))
	{
		if ((pulseTime[1] >= MIN_PULSE_TIME)	// normal ist 16 oder 17....
		 || ((syncActive != 0) && (syncPulseTime[1] >= MIN_PULSE_TIME)))
		{
			useTimer[1] = -1;
		} else
		{
			useTimer[1] = 0;
		}
	}
#endif


#ifdef __ICCAVR__
	TIMSK0	=	0;
#else
	//	TODO	pr黤en welche IRQs tats鋍hlich abgeschaltet werden m黶sen...
	//__disable_irq();
#endif
	
        
        if (syncActive == 0)
	{
		keys.data = *RfButtons;//getRFbuttons();
	} else
	{
		keys.data = syncKeysTransmitted;
		keys.data |= syncKeysReceived;
	};
	if (btActive != 0)
          keys.data |= syncBtKeysReceived;
	
        /////////////////////////////////// moving /////////////////////////////////////
	
#if (NUM_DRIVES < 4)
	mData.data		=	0;
#endif

	mData.m1up		=	keys.m1up;
	mData.m1down	=	keys.m1down;
	mData.m2up		=	keys.m2up;
	mData.m2down	=	keys.m2down;
	
#if (NUM_DRIVES >= 3)
	mData.m3up		=	keys.m3up;
	mData.m3down	=	keys.m3down;
#endif
#if (NUM_DRIVES >= 4)
	mData.m4up		=	keys.m4up;
	mData.m4down	=	keys.m4down;
#endif
/*	
	if (keys.resetUp != 0)
	{
		mData.m1up	=	1;
		mData.m2up	=	1;
		mData.m3up	=	1;
		mData.m4up	=	1;
	};
*/	
	if (/*(keys.resetDown != 0)
	 || */(NAS_KEY == 0)
	 || (allFlatTimer != 0))
	{
		mData.m1down	=	1;
		mData.m2down	=	1;
		mData.m3down	=	1;
		mData.m4down	=	1;
	};
	/////////////////////////////////// memory /////////////////////////////////////
	
	memcpy(lastMemPos, memPos, sizeof(lastMemPos));
#ifdef USE_TIMECOUNT
	lastTimePos1 = timePos[0];
	lastTimePos2 = timePos[1];
#endif
#ifdef REPROGRAMMABLE_MEMORY_POSITIONS

#ifdef USE_TIMECOUNT
#error USE_TIMECOUNT wird nicht unterst姒涚─t!
#endif
#ifdef ZERO_G_ONLY
#error ZERO_G_ONLY wird nicht unterst姒涚─t!
#endif
	
	
	if (lastKeys.data != keys.data)
	{
		if ((keys.data != 0)
		 && ((getMemoryActive() != 0) || (allFlatTimer != 0) || (automaticMovementIsActive != 0)))
		{
			stopMemory();
			allFlatTimer = 0;
			memset((void *)syncTimeout, 0, sizeof(syncTimeout));
			stopAll = active;
			memoryStopped = -1;
		}
		
		if ((keys.data & (KEY_MEMORY2 | KEY_MEMORY3 | KEY_MEMORY4)) != 0)
		{
			memTimer = MEM_STORE_DELAY;	//	Wartezeit f姒涳拷das Speichern setzen...
			blinkMode = none;			
		} else
		if (keys.data != 0)
		{
			memTimer = 0;

#ifndef   NEW_TIMER_LED_STATUS
			blinkMode = none;
#endif
		};
	};
 
	
	if (((keys.data & (KEY_ZERO_G | KEY_ALLFLAT)) == (KEY_ZERO_G | KEY_ALLFLAT))
	 && ((lastKeys.data & (KEY_ZERO_G | KEY_ALLFLAT)) != (KEY_ZERO_G | KEY_ALLFLAT)))
	{
		memoryStopped = -1;
		memTimer = MEM_RESTORE_DELAY;
		blinkMode = none;
	}

	if (((keys.data & (KEY_ZERO_G | KEY_ALLFLAT)) != (KEY_ZERO_G | KEY_ALLFLAT))
	 && ((lastKeys.data & (KEY_ZERO_G | KEY_ALLFLAT)) == (KEY_ZERO_G | KEY_ALLFLAT)))
	{
		memoryStopped = -1;
		memTimer = 0;
#ifndef   NEW_TIMER_LED_STATUS
			blinkMode = none;
#endif
	}
	
	if (keys.data == 0)
	{
		memTimer = 0;
#ifndef   NEW_TIMER_LED_STATUS
			blinkMode = none;
#endif
	};
	
	if (memTimer == 1)
	{
		if ((keys.data & (KEY_ZERO_G | KEY_ALLFLAT)) == (KEY_ZERO_G | KEY_ALLFLAT))
		{
			restoreMemory(1);
			restoreMemory(2);
			restoreMemory(3);
		} else
		{
/*			
			if (keys.zeroG != 0)
			{
				storeMemory(0);
			};
*/
			if (keys.memory2 != 0)
			{
				storeMemory(1);
			};
			if (keys.memory3 != 0)
			{
				storeMemory(2);
			};
			if (keys.memory4 != 0)
			{
				storeMemory(3);
			};
		};
		memTimer = 0;
		blinkMode = memKeyStored;
		blinkState = 0x2F;	//	3 mal blinken...
/*	} else
	if ((memTimer < 150)
	 && (memTimer > 1))
	{
		blinkMode = memKeyPressed;
*/	};
	
	if ((keys.data == 0)
	 && (lastKeys.data != 0))
	{
		if (memoryStopped != 0)
		{
			// nichts tun...
		} else
		if (lastKeys.zeroG != 0)
		{
			loadMemory(0);
		} else
		if (lastKeys.memory4 != 0)
		{
			loadMemory(3);
		} else
		if (lastKeys.memory3 != 0)
		{
			loadMemory(2);
		} else
		if (lastKeys.memory2 != 0)
		{
			loadMemory(1);
		} else
			if(lastKeys.angleAdjust != 0)
			{

				memPos[0] = syncBtpulseCounter[0];
				memPos[1] = syncBtpulseCounter[1];
			}else
		if (lastKeys.allFlat != 0)
		{
			allFlatTimer = 255;
			allFlatTimeoutTimer = 4000;	//	ca. 40 sekunden...
			
			//massageTimer = 0;
			//memset(massage_status, 0, sizeof(massage_status));
		} else
		if (lastKeys.data != 0)
		{
			stopMemory();
			allFlatTimer = 0;
		};
	};
	
	if (keys.data == 0)
	{		
		memoryStopped = 0;
	};
	
	
#else	//	#ifdef REPROGRAMMABLE_MEMORY_POSITIONS	
	if (lastKeys.data == 0)
	{
		if ((keys.data != 0)
		 && ((getMemoryActive() != 0) || (allFlatTimer != 0) || (automaticMovementIsActive != 0)))
		{
			stopMemory();
			allFlatTimer = 0;
			memset((void *)syncTimeout, 0, sizeof(syncTimeout));
			stopAll = active;
		} else		
		if (keys.zeroG != 0)
		{
			loadMemory(0);
#ifdef USE_TIMECOUNT
			ee2memcpy((char*) timePos, (__eeprom char*)ee_timePosition[0], sizeof(timePos));
#endif
	#ifndef ZERO_G_ONLY
		} else
		if (keys.memory4 != 0)
		{
			loadMemory(3);
#ifdef USE_TIMECOUNT
			ee2memcpy((char*) timePos, (__eeprom char*)ee_timePosition[3], sizeof(timePos));
#endif
		} else
		if (keys.memory3 != 0)
		{
			loadMemory(2);
#ifdef USE_TIMECOUNT
			ee2memcpy((char*) timePos, (__eeprom char*)ee_timePosition[2], sizeof(timePos));
#endif
		} else
		if (keys.memory2 != 0)
		{
			loadMemory(1);
#ifdef USE_TIMECOUNT
			ee2memcpy((char*) timePos, (__eeprom char*)ee_timePosition[1], sizeof(timePos));
#endif
	#endif
		} else
			if(keys.angleAdjust != 0)
			{
				memPos[0] = syncBtpulseCounter[0];
				memPos[1] =  syncBtpulseCounter[1];
			}
		else if (keys.allFlat != 0)
		{
//			if (lastKeys.data == 0)
//			{
//				if (allFlatTimer == 0)
//				{
					allFlatTimer = 255;
#ifdef USE_TIMECOUNT
					allFlatTimeoutTimer = 3000;	//	ca. 30 sekunden...
#else
					allFlatTimeoutTimer = 4000;	//	ca. 40 sekunden...
#endif
					
					//massageTimer = 0;
		//			MASSAGE_1 = 0;
		//			MASSAGE_2 = 0;
					//memset(massage_status, 0, sizeof(massage_status));
//				} else
//				{
//					allFlatTimer = 0;
//				};
//			};
		} else
		if (keys.data != 0)
		{
			stopMemory();
			allFlatTimer = 0;
		};

	};
	
#endif	//	#ifdef REPROGRAMMABLE_MEMORY_POSITIONS
	
#ifdef LP_STYLE_FUNCTIONALITY
	if (ee_read_value(&ee_usageMode) != pressAndRelease)
	{
		if (keys.data == 0)
		{
			stopMemory();
			allFlatTimer = 0;
		};
	};
#endif
	
#ifdef RAVEN
	if (ravenM3active == 0)
	{
		memPos[2] = MEM_OFF;
	}
#endif


//	if(keys.angleAdjust != 0 && keys.angleAdjust == lastKeys.angleAdjust)
//	{
	//	memPos[0] = syncBtpulseCounter[0][0];
	//	memPos[1] =  syncBtpulseCounter[1][0];
	//}

	if ((memcmp(lastMemPos, memPos, sizeof(memPos)) != 0)
#ifdef USE_TIMECOUNT
	 || (lastTimePos1 != timePos[0])
	 || (lastTimePos2 != timePos[1])
#endif
		 )
	{
		mDataCurrent.data = 0;
	};
	
	for (i = 0; i < NUM_MEM_DRIVES; i++)
	{
		if (memPos[i] == MEM_OFF)
		{
			mDataCurrent.m1 = 0;
		} else
		{
			mData.m1 = 0;

			if (mDataCurrent.m1 == 0)
			{
#ifdef USE_TIMECOUNT
				if (useTimer[i] != 0)
				{
					if (timePos[i] < timerCountGetCounter(i) - TIME_DIFF)
						mData.m1down = 1;
					if (timePos[i] > timerCountGetCounter(i) + TIME_DIFF)
						mData.m1up = 1;
				} else
				{
					if (memPos[i] < getPulseCounter(i) - POS_DIFF)
						mData.m1down = 1;
					if (memPos[i] > getPulseCounter(i) + POS_DIFF)
						mData.m1up = 1;
				}
#else
				if (memPos[i] < getPulseCounter(i) - POS_DIFF)
					mData.m1down = 1;
				if (memPos[i] > getPulseCounter(i) + POS_DIFF)
					mData.m1up = 1;
#endif

				mDataCurrent.m1 = mData.m1;
				moveToTimer[i] = 255;
			} else
			{
				mData.m1 = mDataCurrent.m1;

#ifdef USE_TIMECOUNT
				if (useTimer[i] != 0)
				{
					if (timePos[i] >= timerCountGetCounter(i))
						mData.m1down = 0;
					if (timePos[i] <= timerCountGetCounter(i))
						mData.m1up = 0;
					moveToTimer[i] = ALL_FLAT_PULSE_TIMEOUT;
				} else
				{
					if (memPos[i] >= getPulseCounter(i))
						mData.m1down = 0;
					if (memPos[i] <= getPulseCounter(i))
						mData.m1up = 0;
					if ((pulseTime[i] < PULSE_TIMEOUT)
					 || (driveState[i] == waitUp)
					 || (driveState[i] == waitDown))
						moveToTimer[i] = ALL_FLAT_PULSE_TIMEOUT;
				}
#else
				if (memPos[i] >= getPulseCounter(i))
					mData.m1down = 0;
				if (memPos[i] <= getPulseCounter(i))
					mData.m1up = 0;
				if ((pulseTime[i] < PULSETIME_MAX_VALUE)
				 || (driveState[i] == waitUp)
				 || (driveState[i] == waitDown))
					moveToTimer[i] = ALL_FLAT_PULSE_TIMEOUT;
#endif
			}
			
			if (mData.m1 == 0)
			{
				memPos[i] = MEM_OFF;
			}
		}
		
		if (moveToTimer[i] == 0)
		{
			memPos[i ] = MEM_OFF;
		};


		mData.data >>= 2;
		mDataCurrent.data >>= 2;
	}

	mData.data <<= (2 * NUM_MEM_DRIVES);
	mDataCurrent.data <<= (2 * NUM_MEM_DRIVES);
        
        
        	if ((keys.data == 0)
	 && (stopAll != inactive))
	{
		stopAll = setInactive;
	};
	
	if (stopAll != inactive)
	{
		stopMemory();
		allFlatTimer = 0;
		memset((void *)syncTimeout, 0, sizeof(syncTimeout));
		
		mData.data = 0;
		keys.data = 0;
	};
	
	if (stopAll == setInactive)
		stopAll = inactive;
	
/////////////////////////////////// massage ////////////////////////////////////
        
 /////////////////////////////////// teach-mode /////////////////////////////////
	
	if (keys.data != 0)
		nasKeyCount = 0;

	if ((NAS_KEY == 0)
	 && (lastNasTaster != 0))
	{
		strgKeyTimer = KEY_TIMEOUT;
		
		if (getTeachMode())
			clrTeachMode();
	}

	if ((NAS_KEY != 0)
	 && (lastNasTaster == 0)
	 && (strgKeyTimer != 0))
	{
		nasKeyCount++;
		strgKeyTimer = KEY_TIMEOUT;

		
		switch (nasKeyCount)
		{
		  case 2:
			setTeachMode();
			break;
/*			
		  case 3:
			clrTeachMode();
			break;
		  case 4:
			UBB = 1;
			break;
		  case 5:
			nrf_genRandomAddress();
			UBB = 0;
			nasKeyCount = 0;
			break;
*/
		};
	};
	
	lastNasTaster = NAS_KEY;
/////////////////////////////////// misc... ////////////////////////////////////

	
	
	if (allFlatTimeoutTimer == 0)
	{
		allFlatTimer = 0;
	};
	
	if ((allFlatTimer != 0)
	 && ((getPulseCounter(0) != 0)
	  || (getPulseCounter(1) != 0)
#if (NUM_DRIVES >= 3)
	  || (getPulseCounter(2) != 0)
#endif
#if (NUM_DRIVES >= 4)
	  || (getPulseCounter(3) != 0)
#endif
#ifdef USE_TIMECOUNT
	  || (timerCountGetCounter(0) != TIMERCOUNTER_RESETVALUE)
	  || (timerCountGetCounter(1) != TIMERCOUNTER_RESETVALUE)
#endif
		))
	{
		allFlatTimer = ALL_FLAT_PULSE_TIMEOUT;
	};

	if (((keys.ubb != 0) && (lastKeys.ubb == 0))
)//	 || ((keys.m1up != 0) && (keys.m1down != 0) && ((lastKeys.m1up == 0) || (lastKeys.m1down == 0))))
	{
		//UBB = ~UBB;
		UBB_toggle();
	};
	
	lastKeys.data = keys.data;

	if (strgKeyTimer == 0)
	{
//		storePosKeyCount = 0;
//		storeComfortKeyCount = 0;
//		updateLEDs(none);
		if (nasKeyCount == 4)
		{
			//UBB = 0;
			UBB_disable();
		}
		nasKeyCount = 0;
	};
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        for (drive = 0; drive < NUM_DRIVES; drive++)
	{
		if ((mData.m1up != 0) && (mData.m1down == 0))
		{
			setPWMup(drive,255);
			syncTimeout[drive] = SYNC_TIMEOUT;
		} else
		if ((mData.m1up == 0) && (mData.m1down != 0))
		{
			setPWMdown(drive,255);
			syncTimeout[drive] = SYNC_TIMEOUT;
		} else
		{
			//setPWMdown(0,0);
#ifndef NO_SYNC
			if ((syncActive == 0)
			 || (syncTimeout[drive] == 0)                       //同步超时1.5S
                           || (pulseTime[drive] >= PULSETIME_MAX_VALUE)     //驱动器到两端，电流为0
//			 || (stopAll != inactive)
#ifdef USE_TIMECOUNT
			 || ((useTimer[drive] != 0)
			  && !((driveDirection[drive] == moving_down) && (timeCounterSync[drive] < (timerCountGetCounter(drive) - SYNC_OFFSET)))
			  && !((driveDirection[drive] == moving_up) && (timeCounterSync[drive] > (timerCountGetCounter(drive) + SYNC_OFFSET))))
			 || ((useTimer[drive] == 0)
			  && !((driveDirection[drive] == moving_down) && (pulseCounterSync[drive] < (getPulseCounter(drive) - SYNC_OFFSET)))
			  && !((driveDirection[drive] == moving_up) && (pulseCounterSync[drive] > (getPulseCounter(drive) + SYNC_OFFSET)))))
#else
			 ||	(!((driveDirection[drive] == moving_down) && (pulseCounterSync[drive] < (getPulseCounter(drive) - SYNC_OFFSET)))
			  && !((driveDirection[drive] == moving_up) && (pulseCounterSync[drive] > (getPulseCounter(drive) + SYNC_OFFSET)))))
#endif
#endif
			{
				setPWMdown(drive,0);
			}
		};
		
		mData.data >>= 2;
	}
	
	mData.data = 0;
  
}


#define MASSAGE_PRESCALER	1

#define MASSAGE_SHIFT		8
#define MASSAGE_STEP		(MASSAGE_PWM_MAX / 120)

void strgTimer_10ms()
{
	unsigned int i;
	static char prescaler = 0;
	
#ifdef SOFTSTART_SOFTSTOP_MASSAGE
	static uint_fast8_t massagePrescaler = 0;
	static uint_fast16_t massageValue[2] = {0,0};
	if (massagePrescaler == 0)
	{
		massagePrescaler = MASSAGE_PRESCALER;

		for (i = 0; i < 2; i++)
		{
			if (massageValue[i] < newMassageValue[i])
			{
				massageValue[i] += massageValue[i] >> MASSAGE_SHIFT;
//				massageValue[i] += massageValue[i] >> (massageValue[i] >> MASSAGE_SHIFT);
				massageValue[i] += MASSAGE_STEP;
				if (massageValue[i] > newMassageValue[i])
					massageValue[i] = newMassageValue[i];
			}
			if (massageValue[i] > newMassageValue[i])
			{
				massageValue[i] -= massageValue[i] >> MASSAGE_SHIFT;
//				massageValue[i] -= massageValue[i] >> (massageValue[i] >> MASSAGE_SHIFT);
				massageValue[i] -= MASSAGE_STEP;
				if ((massageValue[i] > MASSAGE_PWM_MAX)
				 || (massageValue[i] < newMassageValue[i]))
					massageValue[i] = newMassageValue[i];
			}
		}
	}
	massagePrescaler--;

	setMassage1(massageValue[0]);
	setMassage2(massageValue[1]);
#endif
	
#ifdef HAS_WAVE_BUTTON
	uint_fast16_t waveStep = wave_step[waveStatus];

	if (waveStep == 0)
	{
		waveValue = -0xffff;
	} else
	{
		waveValue += waveStep;
		if (waveValue > 0xffff)
		{
			waveValue = -0xffff;
		}
	}
#endif

	for (i = 0; i < NUM_DRIVES; i++)
	{
//		if (onOffOnDelay[i] != 0)
//			onOffOnDelay[i]--;
		
		if (syncTimeout[i] != 0)
			syncTimeout[i]--;
	};
	
/*	for (i = 0; i < 2; i++)
	{
		if (massageStartTimer[i] != 0)
			massageStartTimer[i]--;
	};
	*/
	if (strgKeyTimer != 0)
		strgKeyTimer--;
	
	if (allFlatTimer != 0)
		allFlatTimer--;
	
	if (allFlatTimeoutTimer != 0)
		allFlatTimeoutTimer--;

	for (i = 0; i < NUM_MEM_DRIVES; i++)
	{
		if (moveToTimer[i] != 0)
			moveToTimer[i]--;
	}
	
//	if (inRushTimer != 0)
//		inRushTimer--;
	
//	if (uesaTimer != 0)
//		uesaTimer--;
	
//	if (massageTimer != 0)
//		massageTimer--;

#ifdef TOGGLE_LEDS
	if (ledToggleCount != 0)
	{
		if (ledToggleTimer != 0)
		{
			ledToggleTimer--;
		} else
		{
			ledToggleCount--;
			
//			if (ledToggleCount != 0)
//			{
				ledToggleTimer	=	LED_TOGGLE_TIME;
//				switchLEDs(ledToggleCode, SW_CMD_LED_TOGGLE);
//			} else
//			{
//				ledToggleTimer	=	LED_TOGGLE_TIME;
//				switchLEDs(ledToggleCode, SW_CMD_LED_OFF);
//			};

		};
	};
#endif

	
	if (prescaler == 0)
	{
		prescaler = 10;

		if (memTimer > 1)
			memTimer--;

		if (feedbackTimer != 0)
			feedbackTimer --;
		
#ifdef REPROGRAMMABLE_MEMORY_POSITIONS
		if (blinkState != 0)
			blinkState--;
#endif
//		if (keyChangedTimer != 0xFF)
//			keyChangedTimer++;
	};
	
	prescaler--;

}
