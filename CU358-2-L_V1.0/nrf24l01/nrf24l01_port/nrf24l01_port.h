#ifndef _NRF24L01_PORT_H_
#define _NRF24L01_PORT_H_

#include "stm8s_conf.h"
#include "misc.h"
#define NRF24L01_PWR_PORT GPIOC
#define NRF24L01_PWR_PIN  GPIO_PIN_1

#define NRF24L01_IRQ_PORT GPIOD
#define NRF24L01_IRQ_PIN  GPIO_PIN_0
#define NRF24L01_IRQ_EXTI EXTI_PORT_GPIOD

#define NRF24L01_CE_PORT  GPIOC
#define NRF24L01_CE_PIN   GPIO_PIN_3
#define NRF24L01_CSN_PORT GPIOC
#define NRF24L01_CSN_PIN  GPIO_PIN_4

#define NRF24L01_SCK_PORT GPIOC
#define NRF24L01_SCK_PIN  GPIO_PIN_5
#define NRF24L01_MOSI_PORT GPIOC
#define NRF24L01_MOSI_PIN  GPIO_PIN_6
#define NRF24L01_MISO_PORT  GPIOC
#define NRF24L01_MISO_PIN   GPIO_PIN_7


/**
 * @brief This function allows the nrf24l01 library to set the CE pin to high level.
 */
#define nrf24l01_setCE_high()  NRF24L01_CE_PORT->ODR |= NRF24L01_CE_PIN
//#define nrf24l01_setCE_high()  GPIO_WriteHigh(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
/**
 * @brief This function allows the nrf24l01 library to set the CE pin to high low.
 */
#define nrf24l01_setCE_low()  NRF24L01_CE_PORT->ODR &= ~NRF24L01_CE_PIN
//#define nrf24l01_setCE_low()  GPIO_WriteLow(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
/**
 * @brief This function allows the nrf24l01 library to set the CSN pin to high level.
 */
#define nrf24l01_setCSN_high() NRF24L01_CSN_PORT->ODR |= NRF24L01_CSN_PIN
//#define nrf24l01_setCSN_high() GPIO_WriteHigh(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)
/**
 * @brief This function allows the nrf24l01 library to set the CSN pin to high low.
 */
#define nrf24l01_setCSN_low() NRF24L01_CSN_PORT->ODR &= ~NRF24L01_CSN_PIN
//#define nrf24l01_setCSN_low() GPIO_WriteLow(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)
/**
 * @brief This function allows the nrf24l01 library the get the state of the interrupt pin.
 */
#define nrf24l01_getIRQ()  (BitStatus)(NRF24L01_IRQ_PORT->IDR & (uint8_t)NRF24L01_IRQ_PIN)
//#define nrf24l01_getIRQ()  GPIO_ReadInputPin(NRF24L01_IRQ_PORT, NRF24L01_IRQ_PIN)


extern volatile uint8_t strgKeyTimer;

void nrf24l01_port_init(void);

uint8_t get_nas_value(void);

#endif