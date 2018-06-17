#include <atmel_start.h>
#include "usart.h"
// #include "subbus.h"
#include "control.h"
// #include "spi.h"
// #include "commands.h"

int main(void)
{
  /* Initializes MCU, drivers and middleware */
  atmel_start_init();
  subbus_t subbus;
#if USE_SUBBUS
#if USE_SUBBUS_DRIVERS
  subbus.add_driver(new subbus_fail_sw);
#endif
  subbus.reset();
#endif
  uart_init();
  Control ctrl(&subbus);
  // spi_init();
  // commands_init();
  while (1) {
    ctrl.poll();
    // poll_spi();
    // poll_commands();
    #if SUBBUS_INTERRUPTS
      if (subbus_intr_req)
        intr_service();
    #endif
  }
}
