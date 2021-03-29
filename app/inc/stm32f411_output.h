#ifndef __STM32F411_OUTPUT_H__
#define __STM32F411_OUTPUT_H__

#include "gpio_driver.h"

#define LED_green 12
#define LED_blue  15

void led_pin_init();

void delay(uint16_t timeout);

void Led_on(uint16_t led_pin);

void Led_off(uint16_t led_pin);





#endif
