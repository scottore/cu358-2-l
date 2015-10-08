////////////////////////////////////////////////////////////////////////////////
//
// Definitionen...
//

union Ergomation_keys_t
{
	struct
	{
		unsigned char m1up:1;			//	Bit 0	0x00 00 00 01 - identisch zu Elegance-N
		unsigned char m1down:1;			//	Bit 1	0x00 00 00 02 - identisch zu Elegance-N
		unsigned char m2up:1;			//	Bit 2	0x00 00 00 04 - identisch zu Elegance-N
		unsigned char m2down:1;			//	Bit 3	0x00 00 00 08 - identisch zu Elegance-N
		unsigned char m3up:1;			//	Bit 4	0x00 00 00 10 - identisch zu Elegance-N
		unsigned char m3down:1;			//	Bit 5	0x00 00 00 20 - identisch zu Elegance-N
		unsigned char m4up:1;			//	Bit 6	0x00 00 00 40 - identisch zu Elegance-N
		unsigned char m4down:1;			//	Bit 7	0x00 00 00 80 - identisch zu Elegance-N

		unsigned char massageAll:1;		//	Bit 8	0x00 00 01 00
		unsigned char massageTimer:1;	//	Bit 9	0x00 00 02 00
		unsigned char massageFeet:1;	//	Bit 10	0x00 00 04 00
		unsigned char massageHead:1;	//	Bit 11	0x00 00 08 00
		unsigned char zeroG:1;			//	Bit 12	0x00 00 10 00 - identisch zu Elegance-N memory 1
		unsigned char memory2:1;		//	Bit 13	0x00 00 20 00 - identisch zu Elegance-N
		unsigned char memory3:1;		//	Bit 14	0x00 00 40 00 - identisch zu Elegance-N
		unsigned char memory4:1;		//	Bit 15	0x00 00 80 00 - identisch zu Elegance-N

		unsigned char storePosition:1;	//	Bit 16	0x00 01 00 00 - identisch zu Elegance-N
		unsigned char ubb:1;			//	Bit 17	0x00 02 00 00 - identisch zu Elegance-N
		unsigned char _lamp:1;			//	Bit 18	0x00 04 00 00 - identisch zu Elegance-N
		unsigned char intensity1:1;		//	Bit 19	0x00 08 00 00
		unsigned char intensity2:1;		//	Bit 20	0x00 10 00 00
		unsigned char intensity3:1;		//	Bit 21	0x00 20 00 00
		unsigned char period:1;			//	Bit 22	0x00 40 00 00
		unsigned char massageHeadMinus:1;//	Bit 23	0x00 80 00 00

		unsigned char massageFeetMinus:1;//	Bit 24	0x01 00 00 00
		unsigned char massageHead2:1;	//	Bit 25	0x02 00 00 00
		unsigned char massageFeet2:1;	//	Bit 26	0x04 00 00 00
		unsigned char allFlat:1;		//	Bit 27	0x08 00 00 00
		unsigned char massageWave:1;	//	Bit	28	0x10 00 00 00
		unsigned char angleAdjust:1;			//	Bit	29	0x20 00 00 00
		unsigned char _f2:1;			//	Bit 30	0x40 00 00 00
		unsigned char _f3:1;			//	Bit 31	0x80 00 00 00
	};
	unsigned long data;
};

//#define	MASSAGE_BUTTONS		0x00780F00u
/*
#define	MASSAGE_BUTTONS			0x13F80F00u
#define	KEY_ZERO_G				0x00001000u
#define KEY_MEMORY2				0x00002000u
#define KEY_MEMORY3				0x00004000u
#define KEY_MEMORY4				0x00008000u
#define	KEY_STORE_POSITION		0x00010000u
#define	UBB_BUTTON				0x00020000u
#define	KEY_MASSAGE_HEAD_MINUS	0x00800000u
#define	KEY_MASSAGE_FEET_MINUS	0x01000000u
#define KEY_ALLFLAT				0x08000000u
*/
union Elegance_T_LEDs_t
{
	struct
	{
		unsigned char m1:1;
		unsigned char m2:1;
		unsigned char m3:1;
		unsigned char m4:1;
		unsigned char mReset:1;
		unsigned char program:1;
		unsigned char intensity:1;
		unsigned char period:1;
		
		unsigned char licht1:1;
		unsigned char licht2:1;
		unsigned char storePosition:1;
		unsigned char storeComfort:1;
		unsigned char memory1:1;
		unsigned char memory2:1;
		unsigned char memory3:1;
		unsigned char memory4:1;
		
		unsigned char memoryStore:1;
		unsigned char f1:1;
		unsigned char f2:1;
		unsigned char f3:1;
		unsigned char lamp:1;
		unsigned char ubb:1;
		unsigned char __dummy1:2;
		
		unsigned char __dummy2:5;
		unsigned char memoryStored:1;
		unsigned char function:1;
		unsigned char continuousMode:1;
		
		unsigned char display;
	};
	unsigned long long data;
};

/*
#define	ELEGANCE_T_LED_M1				0x00000001
#define	ELEGANCE_T_LED_M2				0x00000002
#define	ELEGANCE_T_LED_M3				0x00000004
#define	ELEGANCE_T_LED_M4				0x00000008
#define	ELEGANCE_T_LED_M_RESET			0x00000010
#define	ELEGANCE_T_LED_PROGRAMM			0x00000020
#define	ELEGANCE_T_LED_INTENSITY		0x00000040
#define	ELEGANCE_T_LED_PERIOD			0x00000080

#define	ELEGANCE_T_LED_LICHT1			0x00000100
#define	ELEGANCE_T_LED_LICHT2			0x00000200
#define	ELEGANCE_T_LED_STORE_POS		0x00000400
#define	ELEGANCE_T_LED_STORE_COMFORT	0x00000800
#define	ELEGANCE_T_LED_MEMORY1			0x00001000
#define	ELEGANCE_T_LED_MEMORY2			0x00002000
#define	ELEGANCE_T_LED_MEMORY3			0x00004000
#define	ELEGANCE_T_LED_MEMORY4			0x00008000

#define	ELEGANCE_T_LED_MEMORY_STORE		0x00010000
#define	ELEGANCE_T_LED_F1				0x00020000
#define	ELEGANCE_T_LED_F2				0x00040000
#define	ELEGANCE_T_LED_F3				0x00080000
#define	ELEGANCE_T_LED_LAMP				0x00100000
#define	ELEGANCE_T_LED_UBB				0x00200000
*/

#define	ELEGANCE_T_LED_FUNCTION			0x40000000
#define	ELEGANCE_T_LED_CONTINUOUSMODE	0x80000000
#define	ELEGANCE_T_LED_BACKLIGHT		0x7FFFFFFF



