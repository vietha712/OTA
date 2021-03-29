#include "stm32f411_output.h"
#include <stdint.h>

void led_pin_init(){
  GPIOD_CLK();

  gpio_config Led_Green_conf;
  Led_Green_conf.pin = LED_green;
  Led_Green_conf.mode = OUTPUT_MODE;
  Led_Green_conf.output_type = OUTPUT_PUSHPULL;
  Led_Green_conf.output_speed = MEDIUM_SPEED;
  Led_Green_conf.pupd = NO_PUPD;
  gpio_init(GPIOD, &Led_Green_conf);
}

void Led_on(uint16_t led_pin) {
  gpio_set_reset(GPIOD, led_pin, SET);
}

void Led_off(uint16_t led_pin) {
  gpio_set_reset(GPIOD, led_pin, RESET);
}

void delay(uint16_t  timeout)
{
    unsigned int t1, t2;
    for (t1 = 0; t1 < timeout; t1++)
    {
        for (t2 = 0; t2 < 0xFFF; t2++)
        {
          asm(" nop");
        }
    }
}
