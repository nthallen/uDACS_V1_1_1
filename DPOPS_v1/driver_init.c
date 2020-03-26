/* *****************************************************************************
 * DCOTSS DPOPS uDACs RevB Duct Sensor monitor code
 *
 *    Pin Mapping definitions and initialization.
 *    micro-controller (Atmel SAMD21) Hardware Peripherals initialization.
 * 
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

// SPI Port connected to On-Board Octal 24 bit A/D and Quad 16 bit D/A
// One A/D channel used for monitoring MKS Pressure Sensor on A0
// One D/A channel used to bias the A0- input of the A/D A0 so the
// single ended output of the MKS will still be in range.
//
void spi_ad_da_port_init(void){
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

void spi_ad_da_clock_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

void spi_ad_da_init(void) {
	spi_ad_da_clock_init();
	spi_m_async_init(&SPI_AD_DA, SERCOM0);
	spi_ad_da_port_init();
}

// SPI Port Connected to PMOD connected.  Used for Controlling 2
// Honeywell RSC series Pressure Sensors in the DPOPS Application
//
void spi_pr_sn_port_init(void) {
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

void spi_pr_sn_clock_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

void spi_pr_sn_init(void) {
	spi_pr_sn_clock_init();
	spi_m_async_init(&SPI_PR_SN, SERCOM1);
	spi_pr_sn_port_init();
}

// SPI Port Connected to On-Board SDcard Slot for local storage
// Not yet used in the DPOPS applications
//
void spi_sd_port_init(void) {
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

void spi_sd_clock_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void spi_sd_init(void) {
	spi_sd_clock_init();
	spi_m_async_init(&SPI_SD, SERCOM4);
	spi_sd_port_init();
}

// USART connected to On-Board FTDI chip USART/USB
// Used for communicating with DPOPS Host (Raspberry PI-4)
//
void usart_0_port_init(void) {
	gpio_set_pin_function(PB22, PINMUX_PB22D_SERCOM5_PAD2);
	gpio_set_pin_function(PB23, PINMUX_PB23D_SERCOM5_PAD3);
}

void usart_0_clock_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

void USART_0_init(void) {
	usart_0_clock_init();
	usart_sync_init(&USART_0, SERCOM5, (void *)NULL);
	usart_0_port_init();
}

// Timer used for generating Main Loop While(1) state clock period
// This timer increments a global count variable that can be used
// by any State Machines to establish fixed delays / timing.
// 
static void TIMER_0_init(void) {
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
	timer_init(&TIMER_0, RTC, _rtc_get_timer());
}

// Initialize all general purpose digital I/O pins as well as all 
// dedicated hardware resources used
//
void system_init(void) {
	init_mcu();

	// GPIO on PA01 - ADC had an issue?  Not used
	gpio_set_pin_direction(ADC_Alert, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(ADC_Alert, GPIO_PULL_OFF);
	gpio_set_pin_function(ADC_Alert, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA11 - Chip Select for On-board A/D
	gpio_set_pin_level(ADC_CS, true);
	gpio_set_pin_direction(ADC_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14 - Chip Select for On-board SDcard
	gpio_set_pin_level(SD_CS, true);
	gpio_set_pin_direction(SD_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(SD_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20 - On/Off Control for DPOPS Bypass Pump
	gpio_set_pin_level(PMP_CNTL_2, false);
	gpio_set_pin_direction(PMP_CNTL_2, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(PMP_CNTL_2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21 - EEPROM Chip Select for Honeywell Diff Pressure
	gpio_set_pin_level(EEP2_CS, true);
	gpio_set_pin_direction(EEP2_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(EEP2_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA22 - On/Off Status of DPOPS Pump
	gpio_set_pin_direction(PMP_STAT_1, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PMP_STAT_1, GPIO_PULL_UP);
	gpio_set_pin_function(PMP_STAT_1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23 - On/Off Status of DPOPS Bypass Pump
	gpio_set_pin_direction(PMP_STAT_2, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(PMP_STAT_2, GPIO_PULL_OFF);
	gpio_set_pin_function(PMP_STAT_2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27 - Presence signal of SDcard in SDsocket
	gpio_set_pin_direction(SD_Sns, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(SD_Sns, GPIO_PULL_UP);
	gpio_set_pin_function(SD_Sns, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA28 - EEPROM Chip Select for Honeywell Abs pressure
	gpio_set_pin_level(EEP1_CS, true);
	gpio_set_pin_direction(EEP1_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(EEP1_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB02 - Start Convert Command for On-Board A/D
	gpio_set_pin_level(START, true);
	gpio_set_pin_direction(START, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(START, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08 - On/Off control of DPOPS pump
	gpio_set_pin_level(PMP_CNTL_1, false);
	gpio_set_pin_direction(PMP_CNTL_1, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(PMP_CNTL_1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09 - Chip Select for On-board D/A
	gpio_set_pin_level(DAC_CS, true);
	gpio_set_pin_direction(DAC_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(DAC_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10 - Chip Select for A/D in Honeywell Abs Press
	gpio_set_pin_level(ADC1_CS, true);
	gpio_set_pin_direction(ADC1_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC1_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11 - Chip Select for A/D in Honeywell Diff Press
	gpio_set_pin_level(ADC2_CS, true);
	gpio_set_pin_direction(ADC2_CS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(ADC2_CS, GPIO_PIN_FUNCTION_OFF);

	EXTERNAL_IRQ_0_init();
	spi_ad_da_init();
	spi_pr_sn_init();
	spi_sd_init();
	USART_0_init();
	TIMER_0_init();
}
