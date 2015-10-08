#ifdef __ICCAVR__
#include <ioavr.h>
#include <inavr.h>

#elif  defined(STM8S003) || defined(STM8S103)
#include "stm8s_conf.h"
#include "position.h"
#else
#define	__tinyflash const
#include "stm32f0xx_gpio.h"
#include "hal.h"
#include "pulse.h"
#endif

#include "misc.h"
#include "relais.h"
#include "relais_port.h"

void move_up(uint8_t driver, uint8_t pwm, uint8_t max_current)
{
   setPWMup(driver,pwm);
}
void move_down(uint8_t driver, uint8_t pwm, uint8_t max_current)
{
  setPWMdown(driver,pwm);
}
void move_var(uint8_t driver, uint16_t var, uint8_t max_current)
{
  
}