// #include <atmel_start.h>
#include <hal_init.h>
#include "usart.h"
#include "subbus.h"
#include "control.h"
#include "spi.h"
#include "rtc_timer.h"
#include "i2c.h"
#include "commands.h"
#include "serial_num.h"

static void configure_ports(void) {
	// Dis/Enable pins as needed
	// GPIO on PA14
	gpio_set_pin_level(SD_CS, true);
	gpio_set_pin_direction(SD_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(SD_CS, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA27
  gpio_set_pin_direction(SD, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(SD, GPIO_PULL_OFF);
  gpio_set_pin_function(SD, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB08
  gpio_set_pin_direction(SPR7, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(SPR7, GPIO_PULL_OFF);
  gpio_set_pin_function(SPR7, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20
  gpio_set_pin_direction(SPR29, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(SPR29, GPIO_PULL_OFF);
  gpio_set_pin_function(SPR29, GPIO_PIN_FUNCTION_OFF);

  // gpio_set_pin_level(PMOD1, false);
  gpio_set_pin_direction(PMOD1, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(PMOD1, GPIO_PULL_OFF);
  gpio_set_pin_function(PMOD1, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA21
  gpio_set_pin_direction(PMOD2, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(PMOD2, GPIO_PULL_OFF);
  gpio_set_pin_function(PMOD2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18
	gpio_set_pin_direction(PMOD3, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PMOD3,	GPIO_PULL_OFF);
	gpio_set_pin_function(PMOD3, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA28
	gpio_set_pin_direction(PMOD4, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PMOD4,	GPIO_PULL_OFF);
	gpio_set_pin_function(PMOD4, GPIO_PIN_FUNCTION_OFF);
  
  // GPIO on PB11
	gpio_set_pin_direction(PMOD6, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PMOD6,	GPIO_PULL_OFF);
	gpio_set_pin_function(PMOD6, GPIO_PIN_FUNCTION_OFF);
  
  // GPIO on PB10
	gpio_set_pin_direction(PMOD8, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PMOD8,	GPIO_PULL_OFF);
	gpio_set_pin_function(PMOD8, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB02
  gpio_set_pin_level(START, true);
  gpio_set_pin_direction(START, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(START, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB09
  gpio_set_pin_level(DAC_CS, true);
  gpio_set_pin_direction(DAC_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(DAC_CS, GPIO_PIN_FUNCTION_OFF);
}

int main(void)
{
  /* Initializes MCU, drivers and middleware */
  init_mcu();
  configure_ports();
  if (subbus_add_driver(&sb_base)
   || subbus_add_driver(&sb_fail_sw)
   || subbus_add_driver(&sb_board_desc)
   || subbus_add_driver(&sb_control)
   || subbus_add_driver(&sb_spi)
   || subbus_add_driver(&sb_i2c_j4)
   || subbus_add_driver(&sb_i2c_j6)
   || subbus_add_driver(&sb_rtc)
      #ifdef uDACS_B
        || subbus_add_driver(&sb_ps_spi)
        || subbus_add_driver(&sb_cmd)
      #endif
      ) {
    while (true) ; // some driver is misconfigured.
  }
  subbus_reset();
  // uart_init();
  while (1) {
    // poll_control();
    subbus_poll();
    #if SUBBUS_INTERRUPTS
      if (subbus_intr_req)
        intr_service();
    #endif
    // possibly check for watchdog features
  }
}
