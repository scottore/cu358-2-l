#include "stm8s_conf.h"
int16_t eeReadTempBuffer;
unsigned char*  ee_read_bytes(  unsigned char * ee_addr, unsigned char * data, unsigned int dataLength)
{
  unsigned int i;
  for(i = 0;i < dataLength;i++)
    data[i] = ee_addr[i];
  return data;
}
void ee_write_bytes(unsigned char* ee_addr, const unsigned char * data, unsigned int dataLength)
{
  unsigned int i;
  
  FLASH->CR1 &= (uint8_t)(~FLASH_CR1_FIX);
  FLASH->CR1 |= (uint8_t)FLASH_PROGRAMTIME_TPROG;
  asm("nop");
  asm("nop");
  /* Define FLASH programming time */
 // FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG);
  /* Unlock Data memory */
  FLASH->DUKR = FLASH_RASS_KEY2; /* Warning: keys are reversed on data memory !!! */
  FLASH->DUKR = FLASH_RASS_KEY1;
  asm("nop");
  asm("nop");
  //FLASH_Unlock(FLASH_MEMTYPE_DATA);
  for(i = 0;i < dataLength;i++)
    ee_addr[i] = data[i];
}