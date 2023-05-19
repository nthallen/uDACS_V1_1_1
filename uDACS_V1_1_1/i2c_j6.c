/************************************************************************/
/* 2:16 PM 4/24/2023	file i2c_j6.c  
	
	uDACS I2C interface to J6 and on-board MS8607 PTRH measurement
	On uDACS RevB I2C bus on J6.5(SCL), J6.7(SDA) is shared with on board MS8607
	I2C ADDR: 0x76h (P&T), 0x40h (RH)
	
	NOTE: Needs RTC timer module for delays

/************************************************************************/

/* Need? ***

#include <peripheral_clk_config.h>
#include <hal_gpio.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
// #include <hal_ext_irq.h>  // Need?? ***
 */

#include "driver_temp.h" // Need?? ***
#include "uDACS_pins.h"
#include "i2c_ms8607.h"
#include "subbus.h"
#include "rtc_timer.h"
 
#define pow2(X) (float)(1<<X)

static bool i2c_j6_enabled = I2C_J6_ENABLE_DEFAULT;
static bool i2c_ms8607_enabled = I2C_MS8607_ENABLE_DEFAULT;
static bool i2c_p6_enabled = I2C_P6_ENABLE_DEFAULT;

// Need? ***
static struct io_descriptor *PM_I2C_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t pm_ov_status = 0;
#define PM_SLAVE_ADDR 0x67
#define PM_OVERFLOW 1
#define PM_UNDERFLOW 2

// i2c functions

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&PM_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(PM_I2C_io, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&PM_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(PM_I2C_io, ibuf, nbytes);
}

void i2c_enable(bool value) {
  i2c_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  if (sb_cache_was_read(i2c_cache, I2C_STATUS_OFFSET)) {
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, 0);
  }
  if (I2C_error >= -7 && I2C_error <= -2) {
    uint16_t val = i2c_cache[I2C_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_error));
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, val);
  }
	//  Need diffeerent pins here. Not DADC ***
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(DADC_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(DADC_I2C.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void i2c_reset() {
  if (!sb_i2c.initialized) {
    // I2C_init(); // Called from driver_init
    i2c_m_async_get_io_descriptor(&PM_I2C, &PM_I2C_io);
    i2c_m_async_enable(&PM_I2C);
    i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_async_error);
    i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);
    i2c_m_async_register_callback(&PM_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);

    sb_i2c.initialized = true;
  }
}
//  End of I2C functions

// I think we can use an array here, not a struct. 

typedef struct {
    uint8_t cmd[3];	// cmd - Coefficient read commands
} ms8607_prom_read;

ms8607_prom_read msp_read_coef[8] = {
  { {0xA0, 0x00, 0x00} }, // Read CRC & Manuf info (?)
  { {0xA2, 0x00, 0x00} }, // Read Coeff C1
  { {0xA4, 0x00, 0x00} }, // Read Coeff C2
  { {0xA6, 0x00, 0x00} }, // Read Coeff C3
  { {0xA8, 0x00, 0x00} }, // Read Coeff C4
  { {0xAA, 0x00, 0x00} }, // Read Coeff C5
  { {0xAC, 0x00, 0x00} }, // Read Coeff C6 & RH CRC
  { {0xAE, 0x00, 0x00} }  // Unused 
};
static uint8_t coef_num = 0;
static uint8_t msp_ibuf[I2C_J6_MAX_READ_LENGTH];
static uint8_t msrh_ibuf[I2C_J6_MAX_READ_LENGTH];

//  *** May not need the array. May just need command, and read back 4 bytes.
/* static uint8_t ms8607_adc_read[4] = {
  MSP_ADC_READ, 		// Send ADC Read command
  0x00, 0x00, 0x00 	// read back ADC on SDO
};
 */
static uint8_t msp_adc_read = MSP_ADC_READ;

static uint8_t msp_reset_cmd = MSP_RESET;
static uint8_t msp_conv_D1_osr;
static uint8_t msp_conv_D2_osr;
static uint8_t msrh_meas_D3 = MSRH_MEAS_D3;

// Need to add RH values to cache ***

/* These addresses belong to the I2C_J6 module
 * 0x60 R:  PL: 16b Compensated Pressure LSW
 * 0x61 R:  PM: 16b Compensated Pressure MSB
 * 0x62 R:  TL: 16b Compensated Temperature LSW
 * 0x63 R:  TM: 16b Compensated Temperature MSB
 * 0x64 R:  C1: 16b Pressure sensitivity | SENST1
 * 0x65 R:  C2: 16b Pressure offset | OFFT1
 * 0x66 R:  C3: 16b Temperature coeff. of pressure sensitivity | TCS
 * 0x67 R:  C4: 16b Temperature coefficient of pressure offset | TCO
 * 0x68 R:  C5: 16b Reference temperature | TREF
 * 0x69 R:  C6: 16b Temperature coefficient of the temperature | TEMPSENS
 * 0x6A R:  D1L:16b Raw Pressure LSW
 * 0x6B R:  D1M:16b Raw Pressure MSB
 * 0x6C R:  D2L:16b Raw Temperature LSW
 * 0x6D R:  D2M:16b Raw Temperature MSB
 * 0x6E R:  OSR:16b OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096, 5:8192)
 */
static subbus_cache_word_t i2c_j6_cache[I2C_J6_HIGH_ADDR-I2C_J6_BASE_ADDR+1] = {
  { 0, 0, true,  false, false,  false, false }, // Offset 0x00: R: Compensated Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x01: R: Compensated Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x02: R: Compensated Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x03: R: Compensated Temperature MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x04: R: C1: Pressure sensitivity | SENST1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x05: R: C2: Pressure offset | OFFT1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x06: R: C3: Temperature coefficient of pressure sensitivity | TCS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x07: R: C4: Temperature coefficient of pressure offset | TCO
  { 0, 0, true,  false, false,  false, false }, // Offset 0x08: R: C5: Reference temperature | TREF
  { 0, 0, true,  false, false,  false, false }, // Offset 0x09: R: C6: Temperature coefficient of the temperature | TEMPSENS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0A: R: Raw Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0B: R: Raw Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0C: R: Raw Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0D: R: Raw Temperature MSB
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x0E: RW: OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096)
// .cache	.wvalue	.readable	.was_read	.writable	.written	.dynamic
};

// For P and T
// 
//
// start_spi_transfer to start P conversion then -check
//   MSP_CONV_D1 + (2 * MSP_OSR_OFFS) // OFFS can change for different OSR. 
//		would be on cache.
//
// set endtime -check
// delay until done then start_spi_transfer to read P addr 0x00 -check

// store read P results AND start_spi_transfer to start T conversion
// set new endtime
// delay until done then start_spi_transfer to read T addr 0x00
// store read T results go back to Read P : give up the BUS


/*	Need to add  RH states
 ****************************************************
 *	MS8607 Driver State Machine
 *	ms8607_init - Reset ms8607 to initialize
 *	ms8607_readcal - Read Calibration data (6)
 *	ms8607_convp - Send Convert Pressure Command
 *	ms8607_readp - Read Pressure
 *	ms8607_convt - Send Convert Temperature Command
 *	ms8607_readt - Read Temperature
 */
enum ms8607_state_t {
        ms8607_init, ms8607_init_tx, ms8607_init_delay,
        ms8607_readcal, ms8607_readcal_tx,
        ms8607_convp, ms8607_convp_tx, ms8607_convp_delay,
        ms8607_readp, ms8607_readp_tx,
        ms8607_convt, ms8607_convt_tx, ms8607_convt_delay,
        ms8607_readt, ms8607_readt_tx};

typedef struct {
  bool enabled;
  enum ms8607_state_t state;
  uint32_t D1;		// Raw Pressure
  uint32_t D2;		// Raw Temperature
  uint16_t cal[8];
  float P; 	// Compensated Pressure
  float T; 	// Compensated Temperature
  uint32_t endtime;
  uint32_t delay;
//  uint16_t current;
} ms8607_poll_def;

static ms8607_poll_def ms8607 = {
    I2C_MS8607_ENABLED, ms8607_init
//	, 0, 0
//	, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
//	, 0, 0
};


/**
 * poll_ms8607() is only called when I2C_txfr_complete = true
 *    and I2C bus is free
 * @return true if we are relinquishing the SPI bus
 */
static bool poll_ms8607() {
  // uint32_t delay = 0x00000000;
  float dT; 	// difference between actual and measured temperature
  float OFF; 	// offset at actual temperature
  float SENS; 	// sensitivity at actual temperature
  if (!I2C_MS8607_ENABLED) return true;
  switch (ms8607.state) {
    case ms8607_init:
//      start_spi_transfer(ms8607.cs_pin, &msp_reset_cmd, 1, SPI_MODE_0);
			i2c_write(MSP_I2C_ADDR, &msp_reset_cmd, 1);
      ms8607.state = ms8607_init_tx;
      return false;

    case ms8607_init_tx:
      ms8607.endtime = rtc_current_count + ( 3 * RTC_COUNTS_PER_MSEC ); // >2.8mS
      ms8607.state = ms8607_init_delay;
      return false;

    case ms8607_init_delay:
      if ( rtc_current_count <= ms8607.endtime ) return false;
      // for (int j=0; j < 5500; ++j);  // 5500 about 3ms delay
      // chip_deselect(ms8607.cs_pin);	// Just don't need with i2c
      ms8607.state = ms8607_readcal;
      return true;

    case ms8607_readcal:
      // need to send 7 / receive 12 bytes to get
      // 1 x 16b CRC & Manuf info (TBD)
      // 6 x 16b coefficients
//      start_spi_transfer(ms8607.cs_pin, (&msp_read_coef[coef_num].cmd[0]), 3, SPI_MODE_0);
			i2c_write(MSP_I2C_ADDR, (&msp_read_coef[coef_num].cmd[0]), 1);	// maybe use array for cmd instead of struct?
			i2c_read(MSP_I2C_ADDR, &msp_ibuf, 2);
			ms8607.state = ms8607_readcal_tx;
      return false;

    case ms8607_readcal_tx:
      // read coeff from msp_ibuf into cache
      // possible check here for 0xFE on every third byte
      ms8607.cal[coef_num] = ( // update ms8607 struct
         (((uint16_t)msp_ibuf[1])<<8)
          | ((uint16_t)msp_ibuf[2]));
      if (coef_num > 0) i2c_j6_cache[coef_num + 3].cache = ms8607.cal[coef_num]; // update cache
//      chip_deselect(ms8607.cs_pin);
      ms8607.state = ms8607_readp;
      if (coef_num++ < 7) ms8607.state = ms8607_readcal;
      return true;

    // return loop here
    case ms8607_convp:
      msp_conv_D1_osr = MSP_CONV_D1 + (2 * i2c_j6_cache[0x0E].cache); // Update CONV_D1 cmd with OSR offset
//      start_spi_transfer(ms8607.cs_pin, &msp_conv_D1_osr, 1, SPI_MODE_0); // Send Convert D1 (P)
			i2c_write(MSP_I2C_ADDR, &msp_conv_D1_osr, 1); // Send Convert D1 (P)
			
			//      *** NOt sure this needs to be here in the loop ***
      //  ADC OSR=256 	560us ~1ms
      //  ADC OSR=512 	1.10ms ~2ms
      //  ADC OSR=1024	2.17ms ~3ms
      //  ADC OSR=2056	4.32ms ~5ms
      //  ADC OSR=4096	8.61ms ~9ms
      //  ADC OSR=8192	17.2ms ~18ms
      switch (i2c_j6_cache[0x0E].cache) {
        case 0:	ms8607.delay = 1 * RTC_COUNTS_PER_MSEC ; break; // 1mS
        case 1:	ms8607.delay = 2 * RTC_COUNTS_PER_MSEC ; break; // 2mS
        case 2:	ms8607.delay = 3 * RTC_COUNTS_PER_MSEC ; break; // 3mS
        case 3:	ms8607.delay = 5 * RTC_COUNTS_PER_MSEC ; break; // 5mS
        case 4:	ms8607.delay = 9 * RTC_COUNTS_PER_MSEC ; break; // 9mS
        default: ms8607.delay = 18 * RTC_COUNTS_PER_MSEC ; break; // 18mS : case 5 or default
      }
      ms8607.state = ms8607_convp_tx;
      return false;

    case ms8607_convp_tx:
      ms8607.endtime = rtc_current_count + ms8607.delay ;
      ms8607.state = ms8607_convp_delay;
      return false;

    case ms8607_convp_delay:
  	  if ( rtc_current_count <= ms8607.endtime ) return false;
      // for (int j=0; j < 16500; ++j); // 16500 about 10ms delay
//      chip_deselect(ms8607.cs_pin);
      ms8607.state = ms8607_readp;
      return true;

    case ms8607_readp:
//      start_spi_transfer(ms8607.cs_pin, ms8607_adc_read, 4, SPI_MODE_0);
			i2c_write(MSP_I2C_ADDR, msp_adc_read, 1);	
			i2c_read(MSP_I2C_ADDR, &msp_ibuf, 3);
      ms8607.state = ms8607_readp_tx;
      return false;

    case ms8607_readp_tx:
      i2c_j6_cache[0x0A].cache = (  // read P LSW from msp_ibuf and update cache
         (((uint16_t)msp_ibuf[2])<<8)
        | ((uint16_t)msp_ibuf[3]));
      i2c_j6_cache[0x0B].cache = (  // read P MSB from msp_ibuf and update cache
          ((uint16_t)msp_ibuf[1]));
      ms8607.D1 = (((uint32_t)i2c_j6_cache[0x0B].cache)<<16)
        | ((uint32_t)i2c_j6_cache[0x0A].cache);  // Update ms8607.D1 for P calculation
//      chip_deselect(ms8607.cs_pin);
      ms8607.state = ms8607_convt;
      return true;

    case ms8607_convt:
      msp_conv_D2_osr = MSP_CONV_D2 + (2 * i2c_j6_cache[0x0E].cache); // Update CONV_D1 cmd with OSR offset
//      start_spi_transfer(ms8607.cs_pin, &msp_conv_D2_osr, 1, SPI_MODE_0); // Send Convert D2 (T)
			i2c_write(MSP_I2C_ADDR, &msp_conv_D2_osr, 1); // Send Convert D2 (T)
      ms8607.state = ms8607_convt_tx;
      return false;

    case ms8607_convt_tx:
      ms8607.endtime = rtc_current_count + ms8607.delay ;
      ms8607.state = ms8607_convt_delay;
      return false;

    case ms8607_convt_delay:
  	  if ( rtc_current_count <= ms8607.endtime ) return false;
      // for (int j=0; j < 16500; ++j); // 16500 about 10ms delay
//      chip_deselect(ms8607.cs_pin);
      ms8607.state = ms8607_readt;
      return true;

    case ms8607_readt:
//      start_spi_transfer(ms8607.cs_pin, ms8607_adc_read, 4, SPI_MODE_0);
			i2c_write(MSP_I2C_ADDR, msp_adc_read, 1);	
			i2c_read(MSP_I2C_ADDR, &msp_ibuf, 3);
      ms8607.state = ms8607_readt_tx;
      return false;

    case ms8607_readt_tx:
      i2c_j6_cache[0x0C].cache = (
         (((uint16_t)msp_ibuf[2])<<8)
        | ((uint16_t)msp_ibuf[3])); // read T LSW from msp_ibuf and update cache
      i2c_j6_cache[0x0D].cache = (
          ((uint16_t)msp_ibuf[1])); // read T MSB from msp_ibuf and update cache
      ms8607.D2 = ((uint32_t)i2c_j6_cache[0x0D].cache)<<16
        | ((uint32_t)i2c_j6_cache[0x0C].cache);	// Update ms8607.D2 for T calculation

//	Insert RH readings here ***

      // Perform Compensation calculations here and update cache
      dT = ((float)ms8607.D2) - ((float)(ms8607.cal[5]) * pow2(8));
      OFF = ((float)(ms8607.cal[2]) * pow2(17)) + (dT * ((float)(ms8607.cal[4]))) / pow2(6);
      SENS = ((float)(ms8607.cal[1]) * pow2(16)) + (dT * ((float)(ms8607.cal[3]))) / pow2(7);
      ms8607.T = ( 2000 + ((dT * (float)(ms8607.cal[6])) / pow2(23))) / 100;  // degC
      ms8607.P = ((((float)(ms8607.D1) * SENS) / pow2(21)) - OFF) / pow2(15) / 100; // mBar

      sb_cache_update32(i2c_j6_cache, 0, &ms8607.P);	// Update cache P
      sb_cache_update32(i2c_j6_cache, 2, &ms8607.T);	// Update cache T

//      chip_deselect(ms8607.cs_pin);
      ms8607.state = ms8607_convp;	// return to perform next P reading
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
   return true;
}


/**
 * Only called when I2C bus is free
 * @return true if we have relinquished the bus 
 */
static bool poll_i2c_p6(void) {
  if (!i2c_j6_enabled) return true;
//	to service whatever is connected to J6 I2C bus on P6
  return true;
}

// Main poll loop

enum i2c_state_t {i2c_ms8607, i2c_p6 };
static enum i2c_state_t i2c_state = i2c_ms8607;

void i2c_j6_poll(void) {
	// cycle between ms8607 and J6
  if (!i2c_enabled) return;
	  if (true) {	//  do we need condition here? 
    switch (i2c_state) {
      case i2c_ms8607:
        if (poll_ms8607()) {
          i2c_state = i2c_p6;
        }
        break;
      case i2c_p6:
        if (poll_i2c_p6()) {
          i2c_state = i2c_ms8607;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
  }
}

subbus_driver_t sb_i2c_j6 = {
  I2C_J6_BASE_ADDR, I2C_J6_HIGH_ADDR, // address range
  i2c_j6_cache,
  i2c_j6_reset,
  i2c_j6_poll,
  0, // Dynamic function
  false // initialized
};
