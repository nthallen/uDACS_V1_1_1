#include <peripheral_clk_config.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include <hal_ext_irq.h>
#include "spi_ps.h"

struct spi_m_async_descriptor PS_SPI;

void PS_SPI_PORT_init(void) {
  gpio_set_pin_direction(PMOD7, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(PMOD7, GPIO_PULL_OFF);
  gpio_set_pin_function(PMOD7, PINMUX_PA16C_SERCOM1_PAD0);
  gpio_set_pin_level(PMOD5, false);

  gpio_set_pin_direction(PMOD5, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(PMOD5, PINMUX_PA17C_SERCOM1_PAD1);
  gpio_set_pin_level(PMOD1, false);
  gpio_set_pin_direction(PMOD1, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(PMOD1, PINMUX_PA19C_SERCOM1_PAD3);
}

void PS_SPI_CLOCK_init(void) {
  _pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
  _gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

void PS_SPI_init(void) {
  PS_SPI_CLOCK_init();
  spi_m_async_init(&PS_SPI, SERCOM1);
  PS_SPI_PORT_init();
}
