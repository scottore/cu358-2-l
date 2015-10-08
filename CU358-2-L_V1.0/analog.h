#ifndef _ANALOG_H_
#define _ANALOG_H_

#define Channel_0       0x00
#define Channel_1       0x01
#define Channel_2       0x02
#define Channel_3       0x03
#define Channel_4       0x04
#define Channel_5       0x05
#define Channel_6       0x06
#define Channel_7       0x07
#define Channel_8       0x08
#define Channel_9       0x09
#define Channel_10      0x0A
#define Channel_11      0x0B
#define Channel_12      0x0C
#define Channel_13      0x0D
#define Channel_14      0x0E
#define Channel_15      0x0F
#define Channel_Mask    0x0F
extern int16_t current[NUM_DRIVES + 1];
void ADC_Start(uint8_t channel);
void ADC_Stop(void);
void ADC_Data_Callback(void);
void ADC_SetCurrent(void);
#endif