/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "i2c.h"	// was "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>
#include <hpl_gpio.h>
#include "uDACS_pins.h"

#include <hpl_rtc_base.h>

struct i2c_m_async_desc UC_I2C;

void UC_I2C_PORT_init(void)
{
	gpio_set_pin_pull_mode(PMOD7, GPIO_PULL_OFF);
	gpio_set_pin_function(PMOD7, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_pull_mode(PMOD5, GPIO_PULL_OFF);
	gpio_set_pin_function(PMOD5, PINMUX_PA17C_SERCOM1_PAD1);
}

void UC_I2C_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
	_gclk_enable_channel(SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC);
}

void UC_I2C_init(void)
{
	UC_I2C_CLOCK_init();
	i2c_m_async_init(&UC_I2C, SERCOM1);
	UC_I2C_PORT_init();
}
