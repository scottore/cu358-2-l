#ifndef _MISC_H_
#define _MISC_H_
#include "stm8s_conf.h"
#define NUM_DRIVES 2

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

/***************************
AD IO¿Úºê¶¨Òå
***************************/


#define GPIO_AIN0_PORT        GPIOB
#define GPIO_AIN0_PIN         GPIO_PIN_0
#define GPIO_AIN1_PORT        GPIOB
#define GPIO_AIN1_PIN         GPIO_PIN_1
#define GPIO_AIN2_PORT        GPIOB
#define GPIO_AIN2_PIN         GPIO_PIN_2
#define UBB_PORT              GPIOD
#define UBB_PIN               GPIO_PIN_7

#define M1_OUT_PORT           GPIOB
#define M1_OUT_PIN            GPIO_PIN_6
#define M1_IN_PORT            GPIOB
#define M1_IN_PIN             GPIO_PIN_7
#define M2_OUT_PORT           GPIOB
#define M2_OUT_PIN            GPIO_PIN_3
#define M2_IN_PORT            GPIOF
#define M2_IN_PIN             GPIO_PIN_4

#define NAS_PORT              GPIOD
#define NAS_PIN               GPIO_PIN_6
#define OC_OUT_PORT          GPIOC
#define OC_OUT_PIN           GPIO_PIN_1

#define TXPort  GPIOD
#define TXPin   GPIO_PIN_5
#define RXPort  GPIOD
#define RXPin   GPIO_PIN_6

#define gpio_Ain0_init()   do{GPIO_AIN0_PORT->DDR &=~GPIO_AIN0_PIN; \
                          GPIO_AIN0_PORT->CR1 &=~GPIO_AIN0_PIN;}while(0)
#define gpio_Ain1_init()   do{GPIO_AIN1_PORT->DDR &=~GPIO_AIN1_PIN; \
                          GPIO_AIN1_PORT->CR1 &=~GPIO_AIN1_PIN;}while(0)
#define gpio_Ain2_init()   do{GPIO_AIN2_PORT->DDR &=~GPIO_AIN2_PIN; \
                          GPIO_AIN2_PORT->CR1 &=~GPIO_AIN2_PIN;}while(0)

#define gpio_UBB_init()    do{UBB_PORT->DDR |= UBB_PIN; \
                         UBB_PORT->CR1 |= UBB_PIN;}while(0)
#define UBB_ON()  UBB_PORT->ODR |= UBB_PIN
#define UBB_OFF()  UBB_PORT->ODR &= ~UBB_PIN
#define UBB_TOOGLE()  UBB_PORT->ODR ^= UBB_PIN
#define UBB_get()   (UBB_PORT->ODR & UBB_PIN)?1:0
#define UBB_set(x)	if (x) {UBB_ON();} else {UBB_OFF();}
#define UBB_toggle()   UBB_TOOGLE()
#define UBB_disable()   UBB_OFF()

#define gpio_Oc1ptr_init()    do{OC_OUT_PORT->DDR |= OC_OUT_PIN; \
                         OC_OUT_PORT->CR1 |= OC_OUT_PIN;OC_OUT_PORT->ODR |= OC_OUT_PIN;}while(0)
                           
#define gpio_M1up_init()    do{M1_OUT_PORT->DDR |= M1_OUT_PIN; \
                         M1_OUT_PORT->CR1 |= M1_OUT_PIN;M1_OUT_PORT->ODR &= ~M1_OUT_PIN;}while(0)
#define gpio_M1down_init()    do{M1_IN_PORT->DDR |= M1_IN_PIN; \
                         M1_IN_PORT->CR1 |= M1_IN_PIN;M1_IN_PORT->ODR &= ~M1_IN_PIN;}while(0)
#define gpio_M2up_init()    do{M2_OUT_PORT->DDR |= M2_OUT_PIN; \
                         M2_OUT_PORT->CR1 |= M2_OUT_PIN;M2_OUT_PORT->ODR &= ~M2_OUT_PIN;}while(0)
#define gpio_M2down_init()    do{M2_IN_PORT->DDR |= M2_IN_PIN; \
                         M2_IN_PORT->CR1 |= M2_IN_PIN;M2_IN_PORT->ODR &= ~M2_IN_PIN;}while(0)

#define	setM1up(x) do{if(x) M1_OUT_PORT->ODR |= M1_OUT_PIN; \
                      else M1_OUT_PORT->ODR &= ~M1_OUT_PIN;}while(0)
#define	setM2up(x) do{if(x) M2_OUT_PORT->ODR |= M2_OUT_PIN; \
                      else M2_OUT_PORT->ODR &= ~M2_OUT_PIN;}while(0)
#define	setM1down(x) do{if(x) M1_IN_PORT->ODR |= M1_IN_PIN; \
                      else M1_IN_PORT->ODR &= ~M1_IN_PIN;}while(0)
#define	setM2down(x) do{if(x) M2_IN_PORT->ODR |= M2_IN_PIN; \
                      else M2_IN_PORT->ODR &= ~M2_IN_PIN;}while(0)
                        

#define NAS_INIT()    do{NAS_PORT->DDR &= ~NAS_PIN; \
                         NAS_PORT->CR1 &= ~NAS_PIN;}while(0);
#define NAS_KEY   ((uint8_t)(NAS_PORT->IDR & (uint8_t)NAS_PIN)?TRUE:FALSE)

#define	MASSAGE_BUTTONS			        0x000F8013u
#define	KEY_ZERO_G				0x00100000u
#define KEY_MEMORY2				0x00200000u
#define KEY_MEMORY3				0x00400000u
#define KEY_MEMORY4				0x00800000u
#define	KEY_STORE_POSITION		        0x00000100u
#define	UBB_BUTTON				0x00000200u
#define	KEY_MASSAGE_HEAD_MINUS               	0x00000800u
#define	KEY_MASSAGE_FEET_MINUS	                0x00000001u
#define KEY_ALLFLAT				0x00000008u

/*
#define	MASSAGE_BUTTONS			        0x13F80F00u
#define	KEY_ZERO_G				0x00001000u
#define KEY_MEMORY2				0x00002000u
#define KEY_MEMORY3				0x00004000u
#define KEY_MEMORY4				0x00008000u
#define	KEY_STORE_POSITION		        0x00010000u
#define	UBB_BUTTON				0x00020000u
#define	KEY_MASSAGE_HEAD_MINUS               	0x00800000u
#define	KEY_MASSAGE_FEET_MINUS	                0x01000000u
#define KEY_ALLFLAT				0x08000000u
 */                   
//#define	setM2up(x) do{if(x) GPIO_WriteHigh(M2_OUT_PORT, M2_OUT_PIN);else GPIO_WriteLow(M2_OUT_PORT, M2_OUT_PIN);}while(0)	
//#define	setM1down(x) do{if(x) GPIO_WriteHigh(M1_IN_PORT, M1_IN_PIN);else GPIO_WriteLow(M1_IN_PORT, M1_IN_PIN);}while(0)
//#define	setM2down(x) do{if(x) GPIO_WriteHigh(M2_IN_PORT, M2_IN_PIN);else GPIO_WriteLow(M2_IN_PORT, M2_IN_PIN);}while(0)	

#define	MASSAGE_PWM_MAX				21739	//	ca. 92Hz @ 2MHz / 21739
void Nas_10ms_Appl(void);

#endif