#include <peripheral_clk_config.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include <hal_ext_irq.h>
#include "spi_ps.h"

#ifdef uDACS_B

struct spi_m_async_descriptor PS_SPI;

void PS_SPI_PORT_init(void) {
  gpio_set_pin_direction(PS_MISO, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(PS_MISO, GPIO_PULL_OFF);
  gpio_set_pin_function(PS_MISO, PINMUX_PA16C_SERCOM1_PAD0);

  gpio_set_pin_level(PS_SCLK, false);
  gpio_set_pin_direction(PS_SCLK, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(PS_SCLK, PINMUX_PA17C_SERCOM1_PAD1);

  gpio_set_pin_level(PS_MOSI, false);
  gpio_set_pin_direction(PS_MOSI, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(PS_MOSI, PINMUX_PA19C_SERCOM1_PAD3);

  gpio_set_pin_level(EEP3_CS, true);
  gpio_set_pin_direction(EEP3_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(EEP3_CS, GPIO_PIN_FUNCTION_OFF);

  gpio_set_pin_level(EEP2_CS, true);
  gpio_set_pin_direction(EEP2_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(EEP2_CS, GPIO_PIN_FUNCTION_OFF);

  gpio_set_pin_level(EEP1_CS, true);
  gpio_set_pin_direction(EEP1_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(EEP1_CS, GPIO_PIN_FUNCTION_OFF);

  gpio_set_pin_level(ADC3_CS, true);
  gpio_set_pin_direction(ADC3_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(ADC3_CS, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB11
  gpio_set_pin_level(ADC2_CS, true);
  gpio_set_pin_direction(ADC2_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(ADC2_CS, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB10
  gpio_set_pin_level(ADC1_CS, true);
  gpio_set_pin_direction(ADC1_CS, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(ADC1_CS, GPIO_PIN_FUNCTION_OFF);
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

#endif // uDACS_B