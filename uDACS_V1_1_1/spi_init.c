#include <peripheral_clk_config.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include <hal_ext_irq.h>
#include "spi.h"
#include "serial_num.h"

struct spi_m_async_descriptor AD_SPI;

static void EXTERNAL_IRQ_0_init(void) {
  _gclk_enable_channel(EIC_GCLK_ID, CONF_GCLK_EIC_SRC);
  gpio_set_pin_direction(DRDY, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(DRDY, GPIO_PULL_OFF);
  gpio_set_pin_function(DRDY, PINMUX_PB03A_EIC_EXTINT3);
  ext_irq_init();
}

void AD_SPI_PORT_init(void) {
  gpio_set_pin_level(AD_MOSI, false);
  gpio_set_pin_direction(AD_MOSI, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(AD_MOSI, PINMUX_PA08C_SERCOM0_PAD0);
  gpio_set_pin_level(AD_SCLK, false);
  gpio_set_pin_direction(AD_SCLK, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(AD_SCLK, PINMUX_PA09C_SERCOM0_PAD1);
  gpio_set_pin_direction(AD_MISO, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(AD_MISO, GPIO_PULL_OFF);
  gpio_set_pin_function(AD_MISO, PINMUX_PA10C_SERCOM0_PAD2);
	gpio_set_pin_direction(ADC_ALERT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(ADC_ALERT,	GPIO_PULL_OFF);
	gpio_set_pin_function(ADC_ALERT, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_level(ADC_CS, true);
	gpio_set_pin_direction(ADC_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC_CS, GPIO_PIN_FUNCTION_OFF);
}

void AD_SPI_CLOCK_init(void) {
  _pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
  _gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

void AD_SPI_init(void) {
  EXTERNAL_IRQ_0_init();
  AD_SPI_CLOCK_init();
  spi_m_async_init(&AD_SPI, SERCOM0);
  AD_SPI_PORT_init();
}
