/************************************************************************/
/* \file spi.c                                                          */
/************************************************************************/
#include <peripheral_clk_config.h>
#include <hal_spi_m_async.h>
#include <hal_gpio.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include "atmel_start_pins.h"
#include "spi.h"
#include "subbus.h"

static volatile bool AD_SPI_txfr_complete = true;
static bool spi_enabled = SPI_ENABLE_DEFAULT;

void spi_enable(bool value) {
  spi_enabled = value;
}

static inline void chip_select(uint8_t pin) {
  gpio_set_pin_level(pin, false);
}
static inline void chip_deselect(uint8_t pin) {
  gpio_set_pin_level(pin, true);
}

static void complete_cb_AD_SPI(const struct spi_m_async_descriptor *const io_descr) {
  AD_SPI_txfr_complete = true;
}

static uint8_t spi_read_data[MAX_SPI_READ_LENGTH];

static void start_spi_transfer(uint8_t pin, uint8_t const *txbuf, int length) {
  assert(length <= MAX_SPI_READ_LENGTH,__FILE__,__LINE__);
  chip_select(pin);
  AD_SPI_txfr_complete = false;
  spi_m_async_transfer(&AD_SPI, txbuf, spi_read_data, length);
}

#if 0
enum adc_state_t {adc_init, adc_init_tx,
           adc_ain0_wait, adc_ain0_tx,
           adc_ain2_wait, adc_ain2_tx,
           adc_temp_wait, adc_temp_tx};
typedef struct {
  bool enabled;
  enum adc_state_t state;
  uint8_t cs_pin;
  uint16_t AIN0_addr;
  uint16_t AIN2_addr;
  uint16_t TEMP_addr;
  uint16_t poll_count;
} adc_poll_def;

static adc_poll_def adc_u2 = {AD_SPI_U2_ENABLED, adc_init, CS0, 0x10, 0x11, 0x19};
static adc_poll_def adc_u3 = {AD_SPI_U3_ENABLED, adc_init, CS1, 0x12, 0x13, 0x1A};

/**
 * poll_adc() is only called when AD_SPI_txfr_complete is non-zero
 * @return true if we are relinquishing the SPI bus
 */
static bool poll_adc(adc_poll_def *adc) {
  uint16_t value;
  if (!adc->enabled) return true;
  switch (adc->state) {
    case adc_init:
      start_spi_transfer(adc->cs_pin, CONVERT_AIN0, 4);
      adc->state = adc_init_tx;
      return false;
    case adc_init_tx:
      chip_deselect(adc->cs_pin);
      adc->poll_count = 0;
      adc->state = adc_ain0_wait;
      return true;
    case adc_ain0_wait:
      chip_select(adc->cs_pin);
      if (gpio_get_pin_level(MISO)) {
        chip_deselect(adc->cs_pin);
        if (++adc->poll_count <= ADC_CONVERT_TIMEOUT) {
          return true;
        } // Otherwise just go ahead to the next step
      }
      start_spi_transfer(adc->cs_pin, CONVERT_AIN2, 4);
      adc->state = adc_ain0_tx;
      return false;
    case adc_ain0_tx:
      chip_deselect(adc->cs_pin);
      value = (spi_read_data[0] << 8) + spi_read_data[1];
      subbus_cache_update(&sb_spi, adc->AIN0_addr, value);
      adc->poll_count = 0;
      adc->state = adc_ain2_wait;
      return true;
    case adc_ain2_wait:
      chip_select(adc->cs_pin);
      if (gpio_get_pin_level(MISO)) {
        chip_deselect(adc->cs_pin);
        if (++adc->poll_count <= ADC_CONVERT_TIMEOUT) {
          return true;
        } // Otherwise just go ahead to the next step
      }
      start_spi_transfer(adc->cs_pin, CONVERT_TEMP, 4);
      adc->state = adc_ain2_tx;
      return false;
    case adc_ain2_tx:
      chip_deselect(adc->cs_pin);
      value = (spi_read_data[0] << 8) + spi_read_data[1];
      subbus_cache_update(&sb_spi, adc->AIN2_addr, value);
      adc->poll_count = 0;
      adc->state = adc_temp_wait;
      return true;
    case adc_temp_wait:
      chip_select(adc->cs_pin);
      if (gpio_get_pin_level(MISO)) {
        chip_deselect(adc->cs_pin);
        if (++adc->poll_count <= ADC_CONVERT_TIMEOUT) {
          return true;
        } // Otherwise just go ahead to the next step
      }
      start_spi_transfer(adc->cs_pin, CONVERT_AIN0, 4);
      adc->state = adc_temp_tx;
      return false;
    case adc_temp_tx:
      chip_deselect(adc->cs_pin);
      value = (spi_read_data[0] << 8) + spi_read_data[1];
      subbus_cache_update(&sb_spi, adc->TEMP_addr, value);
      adc->poll_count = 0;
      adc->state = adc_ain0_wait;
      return true;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true;
}
#endif

enum dac_state_t {dac_init, dac_tx, dac_idle};
typedef struct {
  bool enabled;
  enum dac_state_t state;
  uint8_t cs_pin;
  uint16_t addr[4];
  uint16_t current;
} dac_poll_def;

static dac_poll_def dac_u13 = {
  SPI_DAC_U13_ENABLED, dac_idle, DAC_CS,
  {0x10, 0x11, 0x12, 0x13}, 0
};
static uint8_t DACREFENABLE[3] = {0x38, 0x00, 0x01};
static uint8_t DACupdate[3];

static bool dac_vref_enabled = false;
/**
 * Only called when SPI bus is free
 * @return true if we have relinquished the bus and cleared our chip select
 */
static bool poll_dac(void) {
  uint16_t value;
  if (!dac_u13.enabled) return true;
  switch (dac_u13.state) {
    case dac_init: // Need to send the internal reference enable signal
      start_spi_transfer(dac_u13.cs_pin, DACREFENABLE, 3);
      dac_vref_enabled = true;
      dac_u13.state = dac_tx;
      return false;
    case dac_tx:
      chip_deselect(dac_u13.cs_pin);
      dac_u13.state = dac_vref_enabled ? dac_idle : dac_init;
      return true;
    case dac_idle:
      if (subbus_cache_iswritten(&sb_spi, dac_u13.addr[dac_u13.current], &value)) {
        DACupdate[0] = 0x18+dac_u13.current;
        DACupdate[1] = (value>>8) & 0xFF;
        DACupdate[2] = value & 0xFF;
        start_spi_transfer(dac_u13.cs_pin, DACupdate, 3);
        subbus_cache_update(&sb_spi, dac_u13.addr[dac_u13.current], value);
        dac_u13.current = (dac_u13.current + 1) & 0x3;
        dac_u13.state = dac_tx;
        return false;
      } else {
        dac_u13.current = (dac_u13.current + 1) & 0x3;
        return true;
      }
    default:
      assert(false,__FILE__,__LINE__);
  }
  return true;
}

enum spi_state_t {spi_adc, spi_dac};
static enum spi_state_t spi_state = spi_dac;

void spi_poll(void) {
  enum spi_state_t input_state = spi_state;
  if (!spi_enabled) return;
  while (AD_SPI_txfr_complete) {
    switch (spi_state) {
      #if 0
      case spi_adc:
        if (poll_adc()) {
          spi_state = spi_dac;
        }
        break;
      #endif
      case spi_dac:
        if (poll_dac()) {
          spi_state = spi_dac;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
    if (spi_state == input_state) break;
  }
}

static void spi_reset(void) {
  if (!sb_spi.initialized) {
    // This type of initialization should not be repeated
    // AD_SPI_init() is called by system_init()
    spi_m_async_register_callback(&AD_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_AD_SPI);
    spi_m_async_enable(&AD_SPI);
    sb_spi.initialized = true;
  }
  // What should reset do? Write zeros to all DACS? For now, do nothing.
}

/**
 * This file should include a memory map. The current one is In Evernote.
 * 0x10-0x13 RW: DAC Flow Setpoints
 */
static subbus_cache_word_t spi_cache[SPI_HIGH_ADDR-SPI_BASE_ADDR+1] = {
  // AD5664 DAC outputs
  { 0, 0, true,  false, true,  false, false }, // Offset 0: RW: DAC Setpoint 0
  { 0, 0, true,  false, true,  false, false }, // Offset 1: RW: DAC Setpoint 1
  { 0, 0, true,  false, true,  false, false }, // Offset 2: RW: DAC Setpoint 2
  { 0, 0, true,  false, true,  false, false }, // Offset 3: RW: DAC Setpoint 3
};

subbus_driver_t sb_spi = {
  SPI_BASE_ADDR, SPI_HIGH_ADDR, // address range
  spi_cache,
  spi_reset,
  spi_poll,
  false
};
