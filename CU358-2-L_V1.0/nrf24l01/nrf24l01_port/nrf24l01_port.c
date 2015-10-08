
#include "nrf24l01_port.h"
#include "rf-protocol.h"
#define	KEY_TIMEOUT				150		//	ca. 2000ms...

//volatile uint8_t NAS_KEY;
volatile uint8_t strgKeyTimer = 0;
void nrf24l01_port_init(void)
{
  NAS_INIT();
  
  NRF24L01_PWR_PORT->DDR |= NRF24L01_PWR_PIN;
  NRF24L01_PWR_PORT->CR1 |= NRF24L01_PWR_PIN;
  NRF24L01_IRQ_PORT->DDR &= ~NRF24L01_IRQ_PIN;
  NRF24L01_IRQ_PORT->CR1 &= ~NRF24L01_IRQ_PIN;
  NRF24L01_CE_PORT->DDR |= NRF24L01_CE_PIN;
  NRF24L01_CE_PORT->CR1 |= NRF24L01_CE_PIN;
  NRF24L01_CSN_PORT->DDR |= NRF24L01_CSN_PIN;
  NRF24L01_CSN_PORT->CR1 |= NRF24L01_CSN_PIN;
  NRF24L01_MOSI_PORT->DDR |= NRF24L01_MISO_PIN;
  NRF24L01_MOSI_PORT->CR1 |= NRF24L01_MISO_PIN;
  NRF24L01_SCK_PORT->DDR |= NRF24L01_SCK_PIN;
  NRF24L01_SCK_PORT->CR1 |= NRF24L01_SCK_PIN;
  NRF24L01_MISO_PORT->DDR &= ~NRF24L01_MISO_PIN;
  NRF24L01_MISO_PORT->CR1 |= NRF24L01_MISO_PIN;

  SPI->CR1 |= BIT(4) | BIT(2);//主设备，高位先发，8分频
    
  SPI->CR2 |= BIT(1)|BIT(0);//nss软件管理  
  SPI->CR2 &=~(BIT(2)|BIT(7));//双线单向模式、全双工
   /* CRC configuration */
  SPI->CRCPR = (uint8_t)0x01;
  
  SPI->CR1 |= BIT(6); //使能SPI BIT(6)

}

/**
 * @brief This function send one byte over the SPI interface (to the radio IC).
 *
 * This function is used by the nrf24l01 library.
 * It allows the library to access the SPI interface.
 */
unsigned char SPI_Transmit(unsigned char ch)
{
  SPI->DR = ch; /* Write in the DR register the data to be sent*/
  while(!(SPI->SR & (uint8_t)SPI_FLAG_RXNE));
 // while (!SPI_GetFlagStatus(SPI_FLAG_RXNE));
  return ((uint8_t)SPI->DR); 
 // return SPI_ReceiveData();
}
uint8_t get_nas_value(void)
{
  return NAS_KEY;
}


