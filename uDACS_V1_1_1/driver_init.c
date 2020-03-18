/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>
#include <hpl_rtc_base.h>

struct timer_descriptor TIMER_0;
struct spi_m_async_descriptor SPI_AD_DA;
struct spi_m_async_descriptor SPI_PR_SN;
struct spi_m_async_descriptor SPI_SD;
struct usart_sync_descriptor USART_0;

void EXTERNAL_IRQ_0_init(void) {
	_gclk_enable_channel(EIC_GCLK_ID, CONF_GCLK_EIC_SRC);
	gpio_set_pin_direction(DRDY, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(DRDY, GPIO_PULL_OFF);
	gpio_set_pin_function(DRDY, PINMUX_PB03A_EIC_EXTINT3);
	ext_irq_init();
}

void SPI_AD_DA_PORT_init(void){
	gpio_set_pin_level(AD_MOSI, false);
	gpio_set_pin_direction(AD_MOSI, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(AD_MOSI, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_level(AD_SCLK, false);
	gpio_set_pin_direction(AD_SCLK, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(AD_SCLK, PINMUX_PA09C_SERCOM0_PAD1);

	gpio_set_pin_direction(AD_MISO, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(AD_MISO, GPIO_PULL_OFF);
	gpio_set_pin_function(AD_MISO, PINMUX_PA10C_SERCOM0_PAD2);
}

void SPI_AD_DA_CLOCK_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

void SPI_AD_DA_init(void) {
	SPI_AD_DA_CLOCK_init();
	spi_m_async_init(&SPI_AD_DA, SERCOM0);
	SPI_AD_DA_PORT_init();
}

void SPI_PR_SN_PORT_init(void) {
	gpio_set_pin_direction(PS_MISO, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PS_MISO, GPIO_PULL_OFF);
	gpio_set_pin_function(PS_MISO, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_level(PS_SCLK, false);
	gpio_set_pin_direction(PS_SCLK, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(PS_SCLK, PINMUX_PA17C_SERCOM1_PAD1);

	gpio_set_pin_level(PS_MOSI, false);
	gpio_set_pin_direction(PS_MOSI, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(PS_MOSI, PINMUX_PA19C_SERCOM1_PAD3);
}

void SPI_PR_SN_CLOCK_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

void SPI_PR_SN_init(void) {
	SPI_PR_SN_CLOCK_init();
	spi_m_async_init(&SPI_PR_SN, SERCOM1);
	SPI_PR_SN_PORT_init();
}

void SPI_SD_PORT_init(void) {
	gpio_set_pin_level(SD_MOSI, false);
	gpio_set_pin_direction(SD_MOSI, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(SD_MOSI, PINMUX_PA12D_SERCOM4_PAD0);

	gpio_set_pin_level(SD_MSCLK, false);
	gpio_set_pin_direction(SD_MSCLK, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(SD_MSCLK, PINMUX_PA13D_SERCOM4_PAD1);

	gpio_set_pin_direction(SD_MISO, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(SD_MISO, GPIO_PULL_OFF);
	gpio_set_pin_function(SD_MISO, PINMUX_PA15D_SERCOM4_PAD3);
}

void SPI_SD_CLOCK_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void SPI_SD_init(void) {
	SPI_SD_CLOCK_init();
	spi_m_async_init(&SPI_SD, SERCOM4);
	SPI_SD_PORT_init();
}

void USART_0_PORT_init(void) {
	gpio_set_pin_function(PB22, PINMUX_PB22D_SERCOM5_PAD2);
	gpio_set_pin_function(PB23, PINMUX_PB23D_SERCOM5_PAD3);
}

void USART_0_CLOCK_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

void USART_0_init(void) {
	USART_0_CLOCK_init();
	usart_sync_init(&USART_0, SERCOM5, (void *)NULL);
	USART_0_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
	timer_init(&TIMER_0, RTC, _rtc_get_timer());
}

void system_init(void) {
	init_mcu();

	// GPIO on PA01
	gpio_set_pin_direction(ADC_Alert, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(ADC_Alert, GPIO_PULL_OFF);
	gpio_set_pin_function(ADC_Alert, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA11
	gpio_set_pin_level(ADC_CS, true);
	gpio_set_pin_direction(ADC_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14
	gpio_set_pin_level(SD_CS, true);
	gpio_set_pin_direction(SD_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(SD_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20
	gpio_set_pin_level(Pmp_CNTL_2, false);
	gpio_set_pin_direction(Pmp_CNTL_2, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(Pmp_CNTL_2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21
	gpio_set_pin_level(EEP2_CS, true);
	gpio_set_pin_direction(EEP2_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(EEP2_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA22
	gpio_set_pin_direction(Pmp_STAT_1, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(Pmp_STAT_1, GPIO_PULL_UP);
	gpio_set_pin_function(Pmp_STAT_1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23
	gpio_set_pin_direction(Pmp_STAT_2, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(Pmp_STAT_2, GPIO_PULL_OFF);
	gpio_set_pin_function(Pmp_STAT_2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27
	gpio_set_pin_direction(SD_Sns, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(SD_Sns, GPIO_PULL_UP);
	gpio_set_pin_function(SD_Sns, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA28
	gpio_set_pin_level(EEP1_CS, true);
	gpio_set_pin_direction(EEP1_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(EEP1_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB02
	gpio_set_pin_level(START, true);
	gpio_set_pin_direction(START, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(START, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08
	gpio_set_pin_level(Pmp_CNTL_1, false);
	gpio_set_pin_direction(Pmp_CNTL_1, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(Pmp_CNTL_1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09
	gpio_set_pin_level(DAC_CS, true);
	gpio_set_pin_direction(DAC_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(DAC_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10
	gpio_set_pin_level(ADC1_CS, true);
	gpio_set_pin_direction(ADC1_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC1_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11
	gpio_set_pin_level(ADC2_CS, true);
	gpio_set_pin_direction(ADC2_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC2_CS, GPIO_PIN_FUNCTION_OFF);

	EXTERNAL_IRQ_0_init();
	SPI_AD_DA_init();
	SPI_PR_SN_init();
	SPI_SD_init();
	USART_0_init();
	TIMER_0_init();
}
