////////////////////////////////////////////////////////////////////////////////
//
// Definitionen...
//

#define	UBB_BUTTON		    0x00020000u

#ifdef RF_TOUCH
#define	MASSAGE_BUTTONS	0x00780000u

#define	RF_TOUCH_T_LED_M1				           0x00000001
#define	RF_TOUCH_T_LED_M2				           0x00000002
#define	RF_TOUCH_T_LED_M3				           0x00000004
#define	RF_TOUCH_T_LED_M4   		 		       0x00000008
#define	RF_TOUCH_T_LED_SOCKET	 		       0x00000200
#define	RF_TOUCH_T_LED_MEMORY1			       0x00001000
#define	RF_TOUCH_T_LED_MEMORY2		    	   0x00002000
#define	RF_TOUCH_T_LED_MEMORY3    			   0x00004000
#define	RF_TOUCH_T_LED_MEMORY4    			   0x00008000
#define	RF_TOUCH_T_LED_UBB             	0x00020000
#define	RF_TOUCH_T_LED_MEMORY_STORE		   0x00010000
#define	RF_TOUCH_T_LED_LAMP	        	   0x00040000
#define	RF_TOUCH_T_LED_MASSAGE_PROGRAM 	0x00080000
#define	RF_TOUCH_T_LED_STOP_MASSAGE 	   0x00400000
#define	RF_TOUCH_T_LED_LOCK	        	   0x04000000
#define	RF_TOUCH_T_LED_SYNC			          0x10000000
#define	RF_TOUCH_T_LED_M_RESET_DOWN		   0x20000000
#define	RF_TOUCH_T_LED_M_RESET_UP		     0x40000000

union RF_Touch_T_keys_t
{
	struct
	{
		unsigned char m1up:1;			        //	Bit 0  - 0x00000001
		unsigned char m1down:1;			      //	Bit 1  - 0x00000002
		unsigned char m2up:1;			        //	Bit 2  - 0x00000004
		unsigned char m2down:1;			      //	Bit 3  - 0x00000008
		unsigned char m3up:1;			        //	Bit 4  - 0x00000010
		unsigned char m3down:1;			      //	Bit 5  - 0x00000020
		unsigned char m4up:1;			        //	Bit 6  - 0x00000040
		unsigned char m4down:1;			      //	Bit 7  - 0x00000080
		unsigned char sync:1;   		      //	Bit 8  - 0x00000100
		unsigned char socket:1; 		      //	Bit 9  - 0x00000200
		unsigned char m5up:1;           //	Bit 10 - 0x00000400
		unsigned char m5down:1;	        //	Bit 11 - 0x00000800
		unsigned char memory1:1;		      //	Bit 12 - 0x00001000
		unsigned char memory2:1;		      //	Bit 13 - 0x00002000
		unsigned char memory3:1;		      //	Bit 14 - 0x00004000
		unsigned char memory4:1;		      //	Bit 15 - 0x00008000
		unsigned char storePosition:1;	 //	Bit 16 - 0x00010000
		unsigned char ubb:1;			         //	Bit 17 - 0x00020000
		unsigned char lamp:1;			        //	Bit 18 - 0x00040000
		unsigned char massage1:1;       //	Bit 19 - 0x00080000
		unsigned char massage2:1;       //	Bit 20 - 0x00100000
		unsigned char massage3:1;       //	Bit 21 - 0x00200000    
		unsigned char stopMassage:1;    //	Bit 22 - 0x00400000
  unsigned char __dummy1:1;       // Bit 23 - 0x00800000
  unsigned char __dummy2:1;       // Bit 24 - 0x01000000
  unsigned char __dummy3:1;       // Bit 25 - 0x02000000  
  unsigned char __dummy4:1;       // Bit 26 - 0x04000000
		unsigned char safetyChild:1;    //	Bit 27 - 0x08000000 
  unsigned char __dummy5:4;       //  Bit 28-31
	};
	unsigned long data;
};

union RF_TOUCH_T_LEDs_t
{
	struct
	{
		unsigned char m1:1;
		unsigned char m2:1;
		unsigned char m3:1;
		unsigned char m4:1;
		unsigned char __dummy1:5;
		unsigned char socket:1;
		unsigned char __dummy2:2;
  unsigned char memory1:1;
  unsigned char memory2:1;
  unsigned char memory3:1;
  unsigned char storePosition:1;    
		unsigned char ubb:1;
		unsigned char lamp:1;
		unsigned char massage1:1;
		unsigned char massage2:1;
		unsigned char massage3:1;
		unsigned char stopMassage1:1;
		unsigned char __dummy3:3;
  unsigned char safetyChild:1;
  unsigned char __dummy4:1;
		unsigned char sync:1;
		unsigned char resetOut:1;
		unsigned char resetIn:1;
		unsigned char conMode:1;        
	};
	unsigned long long data;
};

#else
#define	MASSAGE_BUTTONS	0x08780C00u

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
#define	ELEGANCE_T_LED_FUNCTION			0x40000000
#define	ELEGANCE_T_LED_CONTINUOUSMODE	0x80000000

union Elegance_T_keys_t
{
	struct
	{
		unsigned char m1up:1;			//	Bit 0 - identisch zu Elegance-N
		unsigned char m1down:1;			//	Bit 1 - identisch zu Elegance-N
		unsigned char m2up:1;			//	Bit 2 - identisch zu Elegance-N
		unsigned char m2down:1;			//	Bit 3 - identisch zu Elegance-N
		unsigned char m3up:1;			//	Bit 4 - identisch zu Elegance-N
		unsigned char m3down:1;			//	Bit 5 - identisch zu Elegance-N
		unsigned char m4up:1;			//	Bit 6 - identisch zu Elegance-N
		unsigned char m4down:1;			//	Bit 7 - identisch zu Elegance-N

		unsigned char resetUp:1;		//	Bit 8
		unsigned char resetDown:1;		//	Bit 9
		unsigned char programMinus:1;	//	Bit 10
		unsigned char programPlus:1;	//	Bit 11
		unsigned char memory1:1;		//	Bit 12 - identisch zu Elegance-N
		unsigned char memory2:1;		//	Bit 13 - identisch zu Elegance-N
		unsigned char memory3:1;		//	Bit 14 - identisch zu Elegance-N
		unsigned char memory4:1;		//	Bit 15 - identisch zu Elegance-N

		unsigned char storePosition:1;	//	Bit 16 - identisch zu Elegance-N
		unsigned char ubb:1;			//	Bit 17 - identisch zu Elegance-N
		unsigned char lamp:1;			//	Bit 18 - identisch zu Elegance-N
		unsigned char intensityMinus:1;
		unsigned char intensityPlus:1;
		unsigned char periodMinus:1;
		unsigned char periodPlus:1;
		unsigned char licht1minus:1;

		unsigned char licht1plus:1;
		unsigned char licht2minus:1;
		unsigned char licht2plus:1;
		unsigned char off:1;
		unsigned char storeComfort:1;
		unsigned char f1:1;
		unsigned char f2:1;
		unsigned char f3:1;
	};
	unsigned long data;
};

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
		
		unsigned char __dummy2:6;
		unsigned char function:1;
		unsigned char continuousMode:1;
		
		unsigned char display;
	};
	unsigned long long data;
};
#endif
