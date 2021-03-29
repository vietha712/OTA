#include "gpio_driver.h"
#include "stm32f411_output.h"

int main(void){
  led_pin_init();

  while(1)
  {
    Led_on(LED_green);
    delay(0xff);
    Led_off(LED_green);
    delay(0xff);
  }
}
