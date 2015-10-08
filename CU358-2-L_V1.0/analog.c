#include "stm8s_conf.h"
#include "relais.h"
#include "analog.h"
#include "misc.h"

const uint8_t channelAD[NUM_DRIVES + 1] = {Channel_0,Channel_1,Channel_2};
static int16_t currentTemp[NUM_DRIVES + 1][10];
int16_t current[NUM_DRIVES + 1];

void ADC_Data_Callback(void)
{
  uint8_t i;
  uint8_t temph,templ;
  static uint8_t channel = 0;

  if(channel < NUM_DRIVES + 1)
  {
    for(i = 0;i < 10;i++)
    {
      /* Read LSB first */
      templ = *(uint8_t*)(uint16_t)((uint16_t)ADC1_BaseAddress + (uint8_t)(i << 1) + 1);
      /* Then read MSB */
      temph = *(uint8_t*)(uint16_t)((uint16_t)ADC1_BaseAddress + (uint8_t)(i << 1));

      currentTemp[channel][i] = (uint16_t)(templ | (uint16_t)(temph << (uint8_t)8));
    }
    channel ++;
    ADC_Start(channel);
  }
  else
  {
    channel = 0;
    ADC_Stop();
  }
}
void ADC_SetCurrent(void)
{
  uint8_t i, Index;
  for(Index = 0;Index  < NUM_DRIVES + 1;Index ++)
  {
    current[Index] = 0;
    for(i = 2;i < 10;i++)
      current[Index] += currentTemp[Index][i];
    current[Index] = current[Index] >> 3;
  }
}

/*********************************************
�������ܣ�ADC��ͨ��ת����ʼ��
���������Channel��Channel_0����Channel_15
���������AD��ֵ
*********************************************/
void ADC_Stop(void)
{
  ADC1->CR1 &= ~BIT(0);//�ر�AD
}
void ADC_Start(uint8_t channel)
{  
    ADC1->CSR &= ~Channel_Mask;
    ADC1->CSR |= channelAD[channel];
    ADC1->CSR |= BIT(5); //ѡ��ͨ��,�ж�ʹ��
  
    ADC1->CR1 |= BIT(1);//����ת��ģʽ

    ADC1->CR2 |= BIT(3);//�����Ҷ���
    ADC1->CR3 |= BIT(7);//ʹ�ܻ���
    
    ADC1->CR1 |= BIT(0);//��AD����
}
