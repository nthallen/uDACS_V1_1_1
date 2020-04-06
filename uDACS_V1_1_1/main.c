#include <atmel_start.h>
#include "usart.h"
#include "subbus.h"
#include "control.h"
#include "spi.h"
#include "rtc_timer.h"

int main(void)
{
  /* Initializes MCU, drivers and middleware */
  atmel_start_init();
  if (subbus_add_driver(&sb_base) ||
      subbus_add_driver(&sb_fail_sw) ||
      subbus_add_driver(&sb_board_desc) ||
      subbus_add_driver(&sb_spi) ||
      subbus_add_driver(&sb_rtc)) {
    while (true) ; // some driver is misconfigured.
  }
  subbus_reset();
  uart_init();
  while (1) {
    poll_control();
    subbus_poll();
    #if SUBBUS_INTERRUPTS
      if (subbus_intr_req)
        intr_service();
    #endif
    // possibly check for watchdog features
  }
}
