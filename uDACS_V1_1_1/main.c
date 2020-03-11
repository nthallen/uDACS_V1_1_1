#include <atmel_start.h>
#include "usart.h"
#include "subbus.h"
#include "control.h"
#include "spi.h"
#include "spi_PS.h"

int main(void) {
  /* Initializes MCU, drivers and middle-ware */
  atmel_start_init();
  if (subbus_add_driver(&sb_base) ||
      subbus_add_driver(&sb_fail_sw) ||
      subbus_add_driver(&sb_board_desc) ||
      subbus_add_driver(&sb_spi) ||
	  subbus_add_driver(&sb_matlab_test) ||
	  subbus_add_driver(&sb_PS_spi)) {
    while (true) ; // some driver is mis-configured.
  }
  
  subbus_reset();
  uart_init();

  bool toggle = false;
  while (1) {
	toggle = !toggle;  
	gpio_set_pin_level(SPR7, toggle);
    poll_control();
    subbus_poll();
    #if SUBBUS_INTERRUPTS
      if (subbus_intr_req)
        intr_service();
    #endif
    // possibly check for watchdog features
  }
}
