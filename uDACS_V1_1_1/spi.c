/************************************************************************/
/* \file spi.c                                                          */
/************************************************************************/
#include <peripheral_clk_config.h>
#include <hal_spi_m_async.h>
#include <hal_gpio.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include <hal_ext_irq.h>
#include "uDACS_pins.h"
#include "spi.h"
#include "subbus.h"
#include "rtc_timer.h"

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
static void adc_update_regs();
// We will use modes 1 (AD5664) and 3 (AD7770). Initialize this
// to mode 0 so we will initialize whichever goes first.
static enum spi_transfer_mode spi_current_transfer_mode = SPI_MODE_0;

static void start_spi_transfer(uint8_t pin, uint8_t const *txbuf, int length, enum spi_transfer_mode mode) {
  assert(length <= MAX_SPI_READ_LENGTH,__FILE__,__LINE__);
  if (spi_current_transfer_mode != mode) {
    spi_m_async_disable(&AD_SPI);
    spi_m_async_set_mode(&AD_SPI, mode);
    spi_current_transfer_mode = mode;
    spi_m_async_enable(&AD_SPI);
  }
  chip_select(pin);
  AD_SPI_txfr_complete = false;
  spi_m_async_transfer(&AD_SPI, txbuf, spi_read_data, length);
}

static uint8_t adc_sd_read_output[32] = {
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00,
  0x80, 0x00, 0x80, 0x00
};

static uint8_t adc_read_gen_errs[8] = {
  0x13, 0x80, // read back Regs on SDO
  0xD9, 0x00, // Read GEN_ERR_REG_1 0x59+R
  0xDB, 0x00, // Read GEN_ERR_REG_2 0x5B+R
  0x13, 0x90, // read back ADC on SDO
};

static uint8_t adc_rw_regs[6] = {
  0x13, 0x80, // read back Regs on SDO
  0x80, 0x00, // Reg register 0
  0x13, 0x90, // read back ADC on SDO
};

enum ad7770_state_t {ad7770_init, ad7770_init_tx,
           ad7770_read, ad7770_read_tx,
           ad7770_read_errs, ad7770_read_errs_tx, ad7770_check_sb,
           ad7770_regq, ad7770_regq_tx};
enum adc_regs_state_t {adc_regs_ready, adc_regs_frozen, adc_regs_diverted};
enum adc_readback_mode_t {adc_unknown_mode, adc_reg_mode, adc_sd_mode};
static enum adc_readback_mode_t adc_mode = adc_unknown_mode;
static int DRDY_observed;

typedef struct {
  uint8_t msg[2];
} ad7770_init_word;

/************************************************************************/
/* Initialization commands:                                             */
/* Note that if the ADC is int sigma-delta readback mode, then the      */
/* first byte could be part of the sigma-delta readback error header    */
/* rather than the response from the registers.                         */
/************************************************************************/
#define N_AD7770_INIT 10
ad7770_init_word ad7770_init_codes[N_AD7770_INIT] = {
  { { 0x13, 0x80 } }, // readback regs on SDO
  { { 0xDB, 0x00 } }, // R: GEN_ERR_REG_2 to clear reset detected
  { { 0x11, 0x34 } }, // GENERAL_USER_CONFIG_1: Power up internal reference output
  { { 0x14, 0.00 } }, // Data Output Format Register: Status Header
  { { 0x15, 0x40 } }, // Internal reference
  { { 0x60, 0x0F } }, // SRC N MSB
  { { 0x61, 0xA0 } }, // SRC N LSB
  { { 0x64, 0x01 } }, // SRC LOAD
  { { 0x64, 0x00 } }, // SRC LOAD
  { { 0x13, 0x90 } }  // read back ADC on SDO
};

static struct {
  bool enabled;
  enum ad7770_state_t state;
  enum adc_regs_state_t regs_state;
  uint8_t cs_pin;
  uint16_t n_init;
  uint16_t status;
  uint16_t poll_count;
  uint8_t hdr[8];
  uint32_t ain[8];
} stage = {SPI_AD7770_ENABLED, ad7770_init, adc_regs_ready, ADC_CS, 0, 0, 0};

/**
 * poll_adc() is only called when AD_SPI_txfr_complete is non-zero
 * @return true if we are relinquishing the SPI bus
 */
static bool poll_adc() {
  uint16_t value;
  if (!stage.enabled) return true;
  switch (stage.state) {
    case ad7770_init:
      while (stage.n_init >= N_AD7770_INIT) ;
      start_spi_transfer(stage.cs_pin, (&ad7770_init_codes[stage.n_init].msg[0]), 2, SPI_MODE_3);
      stage.state = ad7770_init_tx;
      return false;
    case ad7770_init_tx:
      chip_deselect(stage.cs_pin);
      if (adc_mode == adc_reg_mode && spi_read_data[0] != 0x20) {
        stage.status |= 0x200;
      }
      if (ad7770_init_codes[stage.n_init].msg[0] == 0x13) {
        adc_mode = (ad7770_init_codes[stage.n_init].msg[1] & 0x10) ?
          adc_sd_mode : adc_reg_mode;
      }
      // check the readback for basic formatting. Set bits in a status register
      stage.poll_count = 0;
      stage.state =
        (++stage.n_init >= N_AD7770_INIT) ? ad7770_read : ad7770_init;
      return true;
    case ad7770_read:
      if (DRDY_observed == 0) {
        subbus_cache_word_t *cw = &sb_spi.cache[POLL_COUNT_ADDR-SPI_BASE_ADDR];
        if (cw->was_read && cw->cache == 0) {
          stage.state = ad7770_check_sb;
        }
        return true;
      }
      start_spi_transfer(stage.cs_pin, adc_sd_read_output, 32, SPI_MODE_3);
      stage.state = ad7770_read_tx;
      return false;
    case ad7770_read_tx:
      ext_irq_disable(DRDY);
      stage.status &= ~0x01FF;
      if (DRDY_observed > 1)
        stage.status |= 0x100;
      DRDY_observed = 0;
      ext_irq_enable(DRDY);
      chip_deselect(stage.cs_pin);
      // check the readback for basic formatting. Set bits in stage.status register
      // Update stage.ain,
      for (int i = 0; i < 8; ++i) {
        int offset = 4*i;
        uint8_t hdr = spi_read_data[offset];
        if (((hdr&0x70)>>4) != i) {
          stage.status |= 1<<i;
        }
        stage.hdr[i] = hdr;
        stage.ain[i] =
          (((uint32_t)spi_read_data[offset+1])<<16) +
          (((uint32_t)spi_read_data[offset+2])<<8) +
          spi_read_data[offset+3];
      }
      ++stage.poll_count;
      if (stage.regs_state == adc_regs_ready) {
        adc_update_regs();
      } else {
        stage.regs_state = adc_regs_diverted;
      }
      stage.state = ad7770_check_sb;
      return true;
    case ad7770_check_sb: // Check here for subbus reads or writes
      if (subbus_cache_was_read(&sb_spi, GEN_ERRS_ADDR)) {
        stage.state = ad7770_read_errs;
        return false;
      } else if (subbus_cache_was_read(&sb_spi,REG_QUERY_ADDR)) {
        uint16_t addrval = sb_spi.cache[REG_QUERY_ADDR-SPI_BASE_ADDR].cache;
        uint8_t addr = (addrval & 0x7F) + ((addrval&0x80) ? 1 : 0);
        if (addr > 0x64) addr = 0;
        adc_rw_regs[2] = addr | 0x80; // Read only
        adc_rw_regs[3] = 0; // Does not matter
        stage.state = ad7770_regq;
        return false;
      } else if (subbus_cache_iswritten(&sb_spi,REG_QUERY_ADDR, &value)) {
        adc_rw_regs[2] = value&0xFF;
        adc_rw_regs[3] = (value>>8)&0xFF;
        stage.state = ad7770_regq;
        return false;
      } else {
        stage.state = ad7770_read;
      }
      return true;
    case ad7770_read_errs:
      start_spi_transfer(stage.cs_pin, adc_read_gen_errs, 8, SPI_MODE_3);
      stage.state = ad7770_read_errs_tx;
      return false;
    case ad7770_read_errs_tx:
      { uint16_t errs = spi_read_data[3] + (spi_read_data[5]<<8);
        subbus_cache_update(&sb_spi, GEN_ERRS_ADDR, errs);
      }
      stage.state = ad7770_read;
      return true;
    case ad7770_regq:
      start_spi_transfer(stage.cs_pin, adc_rw_regs, 8, SPI_MODE_3);
      stage.state = ad7770_regq_tx;
      return false;
    case ad7770_regq_tx:
      value = adc_rw_regs[2];
      value += (value&0x80) ? (spi_read_data[3]<<8) : (adc_rw_regs[3]<<8);
      subbus_cache_update(&sb_spi, REG_QUERY_ADDR, value);
      stage.state = ad7770_read;
      return true;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true;
}

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
      start_spi_transfer(dac_u13.cs_pin, DACREFENABLE, 3, SPI_MODE_1);
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
        start_spi_transfer(dac_u13.cs_pin, DACupdate, 3, SPI_MODE_1);
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
      case spi_adc:
        if (poll_adc()) {
          spi_state = spi_dac;
        }
        #ifdef RTC_USE_MAX_DURATION_REFERENCE
        // rtc_max_state_duration_ref_value = stage.state;
        #endif
        break;
      case spi_dac:
        if (poll_dac()) {
          spi_state = spi_adc;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
    if (spi_state == input_state) break;
  }
}

static void DRDY_handler(void) {
  ++DRDY_observed;
}

static void spi_reset(void) {
  if (!sb_spi.initialized) {
    AD_SPI_init();
    spi_m_async_register_callback(&AD_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_AD_SPI);
    spi_m_async_enable(&AD_SPI);
    ext_irq_register(DRDY, DRDY_handler);
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
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x00: RW: DAC Setpoint 0
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x01: RW: DAC Setpoint 1
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x02: RW: DAC Setpoint 2
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x03: RW: DAC Setpoint 3
  { 0, 0, true,  false, false,  false,  true }, // Offset 0x04: R:  ADC Status Register
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x05: RW: ADC AIN[0] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x06: RW: ADC AIN[0] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x07: RW: ADC AIN[1] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x08: RW: ADC AIN[1] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x09: RW: ADC AIN[2] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x0A: RW: ADC AIN[2] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x0B: RW: ADC AIN[3] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x0C: RW: ADC AIN[3] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x0D: RW: ADC AIN[4] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x0E: RW: ADC AIN[4] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x0F: RW: ADC AIN[5] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x10: RW: ADC AIN[5] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x11: RW: ADC AIN[6] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x12: RW: ADC AIN[6] HDR+MSB
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x13: RW: ADC AIN[7] LSW
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x14: RW: ADC AIN[7] HDR+MSB
  { 0, 0, true,  false, false,  false,  true }, // Offset 0x15: R:  ADC Poll Count
  { 0, 0, true,  false, false,  false, false }, // Offset 0x16: R:  ADC General Errors
  { 0, 0, true,  false,  true,  false, false }, // Offset 0x17: RW: ADC Register Query
};

static void adc_update_regs() {
  spi_cache[0x4].cache = stage.status;
  for (int i = 0; i < 8; ++i) {
    spi_cache[0x5+2*i].cache = (uint16_t)(stage.ain[i] & 0xFFFF);
    spi_cache[0x6+2*i].cache = ((uint16_t)((stage.ain[i] & 0xFF0000)>>16)) |
       (((uint16_t)stage.hdr[i]) << 8);
  }
  subbus_cache_update(&sb_spi, POLL_COUNT_ADDR, stage.poll_count);
}

void spi_action(uint16_t offset) {
  uint16_t last_poll_count;
  switch (offset) {
    case 0x04: // Status register
      // freeze updates to counts, poll_count
      stage.regs_state = adc_regs_frozen;
      break;
    case 0x15: // Poll Count
      // release updates to counts, poll_count
      last_poll_count = spi_cache[0x15].cache;
      if (last_poll_count <= stage.poll_count) {
        stage.poll_count -= last_poll_count;
      } else {
        stage.poll_count = 0;
      }
      spi_cache[0x15].cache = 0;
      switch (stage.regs_state) {
        case adc_regs_ready:
        case adc_regs_frozen:
        default:
          break;
        case adc_regs_diverted:
          adc_update_regs();
          break;
      }
      stage.regs_state = adc_regs_ready;
      break;
    default:
      assert(false,__FILE__,__LINE__);
  }
}

subbus_driver_t sb_spi = {
  SPI_BASE_ADDR, SPI_HIGH_ADDR, // address range
  spi_cache,
  spi_reset,
  spi_poll,
  spi_action,
  false
};
