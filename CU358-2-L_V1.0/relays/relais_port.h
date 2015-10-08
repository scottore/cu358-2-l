#ifndef _RELAIS_PORT_H_
#define _RELAIS_PORT_H_

void move_up(uint8_t driver, uint8_t pwm, uint8_t max_current);
void move_down(uint8_t driver, uint8_t pwm, uint8_t max_current);
void move_var(uint8_t driver, uint16_t var, uint8_t max_current);

#endif