#ifndef _STEUERUNG_H_
#define _STEUERUNG_H_


#define	GO_TIMEOUT					200		///<	10Hz Timer -> ca. 20 Sekunden...
#define GO_SHORT_TIMEOUT			100		///<	max. Zeit die die Go-Taste gedrückt werden darf...
#define GO_MEMORY_TIMEOUT			50		///<	min. Zeit zum aktivieren der Speicher-Funktion
#define GO_MEMORY_STORED_TIMEOUT	8		///<	schnelles blinken beim speichern.
#define GO_SYS_PROT_REV_TIMEOUT		5		///<	ca. 1/2 Sekunden zurück fahren...
#define HEAT_TIMEOUT		12000	///<	ca. 20 Minuten

#define SYSPROT_MIN_VALUE	0x1800	///<	untere Schwelle zum auslösen des Systemschutzes -> ca. 1V...
#define SYSPROT_MAX_VALUE	0x6700	///<	obere Schwelle zum auslösen des Systemschutzes -> ca. 4V...

#define	SYNC_OFFSET			20

#define FEEDBACKMASK			0x01
#define	FEEDBACK_COEFFICIENT	(FEEDBACKMASK << 1)
#define GEN_FEEDBACK_VALUE(x)	((FEEDBACK_COEFFICIENT * x) + 1)

#ifndef NUM_MEM_DRIVES
#define NUM_MEM_DRIVES 2
#endif
void SteuerungFunkInit(void);
void doSteuerung(unsigned long* RfButtons);
void strgTimer_10ms();

void stopMemory(void);
uint8_t getMemoryActive(void);

enum StopAll_t
{
	inactive	=	0,
	active		=	-1,
	setInactive	=	1
};

enum usageMode_t
{
	pressAndRelease	=	0,
	pressAndHold	=	-1,
	unknown			=	1
};

//extern __eeprom volatile enum usageMode_t ee_usageMode;

extern unsigned int heatTimer;
extern unsigned char strgGoTimer;
extern volatile uint32_t massageTimer;
extern volatile unsigned char allFlatTimer;
extern int16_t memPos[];
//#define MEM_OFF	0x8000
#define MEM_OFF	S16_MIN
extern unsigned char massage_status[2];
//extern __eeprom int16_t ee_memoryPosition[4][NUM_MEM_DRIVES];
//extern __eeprom int16_t ee_originalPosition[4][NUM_MEM_DRIVES];
extern enum StopAll_t stopAll;
extern signed char automaticMovementIsActive;

#ifdef ELEGANCE_T
void strgSyncMassage(unsigned char programm, unsigned char intensity, unsigned char periode);
extern unsigned char massagePeriode;
extern unsigned char massageIntensity;
extern unsigned char massageProgramm;
#endif

#ifdef HAS_WAVE_BUTTON
extern uint_fast8_t waveStatus;
extern int_fast32_t waveValue;
#endif



#endif