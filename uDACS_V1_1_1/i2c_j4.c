/************************************************************************/
/* 2:16 PM 4/24/2023	file i2c_j4.c  
	
	uDACS I2C interface to J4 and off-board MS5607 + SHT25 PTRH measurements
	I2C ADDR: 0x77h (P&T), 0x40h (RH)
	
	NOTE: Needs RTC timer module for delays

 ************************************************************************/
#include <hpl_pm_base.h>
#include <peripheral_clk_config.h>
#include <hpl_gclk_base.h>
#include "uDACS_pins.h"
#include "i2c.h"
#include "subbus.h"
#include "rtc_timer.h"
 
#define pow2(X) (float)(1<<X)

#define PM_SLAVE_ADDR 0x67
#define PM_OVERFLOW 1
#define PM_UNDERFLOW 2

struct i2c_m_async_desc UC_I2C;

static bool i2c_j4_enabled = I2C_J4_ENABLE_DEFAULT;

// Need? ***
static struct io_descriptor *UC_I2C_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t pm_ov_status = 0;

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

// I think we can use an array here, not a struct. 

typedef struct {
    uint8_t cmd[1];	// cmd - Coefficient read commands
} ms5607_prom_read;

static ms5607_prom_read msp_read_coef[8] = {
  { { 0xA0 } }, // Read CRC & Manuf info (?)
  { { 0xA2 } }, // Read Coeff C1
  { { 0xA4 } }, // Read Coeff C2
  { { 0xA6 } }, // Read Coeff C3
  { { 0xA8 } }, // Read Coeff C4
  { { 0xAA } }, // Read Coeff C5
  { { 0xAC } }, // Read Coeff C6 & RH CRC
  { { 0xAE } }  // Unused 
};
static uint8_t coef_num = 0;
static uint8_t msp_ibuf[I2C_J4_MAX_READ_LENGTH];
static uint8_t sht_ibuf[I2C_J4_MAX_READ_LENGTH];

static uint8_t msp_adc_read[1] = { MSP_ADC_READ };
static uint8_t msp_reset_cmd[1] = { MSP_RESET };
static uint8_t msp_conv_D1_osr[1];
static uint8_t msp_conv_D2_osr[1];
static uint8_t sht_meas_t[1] = { SHT_MEAS_T };
static uint8_t sht_meas_rh[1] = { SHT_MEAS_RH };

// Need to add RH values to cache ***

/* These addresses belong to the I2C_J4 module
 * 0x80 R:  ST: 16b I2C Status
 * 0x81 R:  PL: 16b Compensated Pressure LSW
 * 0x82 R:  PM: 16b Compensated Pressure MSB
 * 0x83 R:  TL: 16b Compensated Temperature LSW
 * 0x84 R:  TM: 16b Compensated Temperature MSB
 * 0x85 R:  C1: 16b Pressure sensitivity | SENST1
 * 0x86 R:  C2: 16b Pressure offset | OFFT1
 * 0x87 R:  C3: 16b Temperature coeff. of pressure sensitivity | TCS
 * 0x88 R:  C4: 16b Temperature coefficient of pressure offset | TCO
 * 0x89 R:  C5: 16b Reference temperature | TREF
 * 0x8A R:  C6: 16b Temperature coefficient of the temperature | TEMPSENS
 * 0x8B R:  D1L:16b Raw Pressure LSW
 * 0x8C R:  D1M:16b Raw Pressure MSB
 * 0x8D R:  D2L:16b Raw Temperature LSW
 * 0x8E R:  D2M:16b Raw Temperature MSB
 * 0x8F R:  OSR:16b OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096, 5:8192)
 * 0x90 R:  RH: 16b SHT25 Relative Humidity measurement in %
 * 0x91 R:   T: 16b SHT25 Temperature measurement in degC
 * 0x92 RW:UREG:16b SHT25 8-bit User Register (Place holder; Not implemented)
 */
static subbus_cache_word_t i2c_j4_cache[I2C_J4_HIGH_ADDR-I2C_J4_BASE_ADDR+1] = {
  { 0, 0, true,  false, false,  false, false }, // Offset 0x00: R: 16b I2C Status
  { 0, 0, true,  false, false,  false, false }, // Offset 0x01: R: Compensated Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x02: R: Compensated Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x03: R: Compensated Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x04: R: Compensated Temperature MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x05: R: C1: Pressure sensitivity | SENST1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x06: R: C2: Pressure offset | OFFT1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x07: R: C3: Temperature coefficient of pressure sensitivity | TCS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x08: R: C4: Temperature coefficient of pressure offset | TCO
  { 0, 0, true,  false, false,  false, false }, // Offset 0x09: R: C5: Reference temperature | TREF
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0A: R: C6: Temperature coefficient of the temperature | TEMPSENS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0B: R: Raw Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0C: R: Raw Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0D: R: Raw Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0E: R: Raw Temperature MSB
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x0F: RW: OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096)
  { 0, 0, true,  false, false,  false, false }, // Offset 0x10: R: SHT25 Relative Humidity
  { 0, 0, true,  false, false,  false, false }, // Offset 0x11: R: SHT25 Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x12: R: SHT25 Temperature MSW
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x13: RW: SHT25 User Register (Place holder; Not implemented)
// .cache	.wvalue	.readable	.was_read	.writable	.written	.dynamic
};

/*	Need to add  RH states
 ****************************************************
 *	MS5607 Driver State Machine
 *	ms5607_init - Reset ms5607 to initialize
 *	ms5607_readcal - Read Calibration data (6)
 *	ms5607_convp - Send Convert Pressure Command
 *	ms5607_readp - Read Pressure
 *	ms5607_convt - Send Convert Temperature Command
 *	ms5607_readt - Read Temperature
 */
enum ms5607_state_t {
        ms5607_init, ms5607_init_tx, ms5607_init_delay,
        ms5607_readcal, ms5607_readcal_tx, ms5607_readcal_cache,
        ms5607_convp, ms5607_convp_tx, ms5607_convp_delay,
        ms5607_readp, ms5607_readp_tx, ms5607_readp_cache,
        ms5607_convt, ms5607_convt_tx, ms5607_convt_delay,
        ms5607_readt, ms5607_readt_tx, ms5607_readt_cache,
        };

typedef struct {
  bool enabled;
  enum ms5607_state_t state;
  uint32_t D1;		// Raw Pressure
  uint32_t D2;		// Raw Temperature
  uint16_t cal[8];
  float P; 	// Compensated Pressure
  float T; 	// Compensated Temperature
  uint32_t endtime;
  uint32_t delay;
//  uint16_t current;
} ms5607_poll_def;

static ms5607_poll_def ms5607 = {
    I2C_MS5607_ENABLED, ms5607_init
//	, 0, 0
//	, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
//	, 0, 0
};


/**
 * poll_ms5607() is only called when I2C_txfr_complete = true
 *    and I2C bus is free
 * return true if we are relinquishing the I2C bus
 */
static bool poll_ms5607() {
  // uint32_t delay = 0x00000000;
  float dT; 	// difference between actual and measured temperature
  float OFF; 	// offset at actual temperature
  float SENS; 	// sensitivity at actual temperature
  if (!I2C_MS5607_ENABLED || !I2C_txfr_complete ) return true;
  switch (ms5607.state) {
    // Reset ms5607
    case ms5607_init:
			i2c_write(MSP_I2C_ADDR, msp_reset_cmd, 1);  // Reset MS5607 ~ 2.8mSec
      ms5607.state = ms5607_init_tx;
      return false;

    case ms5607_init_tx:
      ms5607.endtime = rtc_current_count + ( 3 * RTC_COUNTS_PER_MSEC ); 
      ms5607.state = ms5607_init_delay;
      return false;

    case ms5607_init_delay:
      if ( rtc_current_count <= ms5607.endtime ) return false;
      ms5607.state = ms5607_readcal;
      return true;

    case ms5607_readcal:
      // need to send 7 / receive 12 bytes to get
      // 1 x 16b CRC & Manuf info (TBD)
      // 6 x 16b coefficients
			i2c_write(MSP_I2C_ADDR, msp_read_coef[coef_num].cmd, 1);	// maybe use array for cmd instead of struct?
			ms5607.state = ms5607_readcal_tx;
      return false;

    case ms5607_readcal_tx:
      i2c_read(MSP_I2C_ADDR, msp_ibuf, 2);
			ms5607.state = ms5607_readcal_cache;
      return false;
      
    case ms5607_readcal_cache:
      // place coeff from msp_ibuf into cache
      ms5607.cal[coef_num] = ( // update ms5607 struct
         (((uint16_t)msp_ibuf[0])<<8)
          | ((uint16_t)msp_ibuf[1]));
      if (coef_num > 0) i2c_j4_cache[coef_num + 4].cache = ms5607.cal[coef_num]; // update cache
      ms5607.state = ms5607_convp;
      if (++coef_num < 7) ms5607.state = ms5607_readcal;
      return true;

    // return loop here
    case ms5607_convp:
      msp_conv_D1_osr[0] = MSP_CONV_D1 + (2 * i2c_j4_cache[0x0F].cache); // Update CONV_D1 cmd with OSR offset
			i2c_write(MSP_I2C_ADDR, msp_conv_D1_osr, 1); // Send Convert D1 (P)
			
			//      *** NOt sure this needs to be here in the loop ***
      //  ADC OSR=256 	560us ~1ms
      //  ADC OSR=512 	1.10ms ~2ms
      //  ADC OSR=1024	2.17ms ~3ms
      //  ADC OSR=2056	4.32ms ~5ms
      //  ADC OSR=4096	8.61ms ~9ms
      //  ADC OSR=8192	17.2ms ~18ms
      switch (i2c_j4_cache[0x0F].cache) {
        case 0:	ms5607.delay = 1 * RTC_COUNTS_PER_MSEC ; break; // 1mS
        case 1:	ms5607.delay = 2 * RTC_COUNTS_PER_MSEC ; break; // 2mS
        case 2:	ms5607.delay = 3 * RTC_COUNTS_PER_MSEC ; break; // 3mS
        case 3:	ms5607.delay = 5 * RTC_COUNTS_PER_MSEC ; break; // 5mS
        case 4:	ms5607.delay = 9 * RTC_COUNTS_PER_MSEC ; break; // 9mS
        default: ms5607.delay = 18 * RTC_COUNTS_PER_MSEC ; break; // 18mS : case 5 or default
      }
      ms5607.state = ms5607_convp_tx;
      return false;

    case ms5607_convp_tx:
      ms5607.endtime = rtc_current_count + ms5607.delay ;
      ms5607.state = ms5607_convp_delay;
      return false;

    case ms5607_convp_delay:
  	  if ( rtc_current_count <= ms5607.endtime ) return false;
      ms5607.state = ms5607_readp;
      return true;

    case ms5607_readp:
	  i2c_write(MSP_I2C_ADDR, msp_adc_read, 1);	
      ms5607.state = ms5607_readp_tx;
      return false;

    case ms5607_readp_tx:
			i2c_read(MSP_I2C_ADDR, msp_ibuf, 3);
      ms5607.state = ms5607_readp_cache;
      return false;

    case ms5607_readp_cache:
      i2c_j4_cache[0x0B].cache = (  // read P LSW from msp_ibuf and update cache
         (((uint16_t)msp_ibuf[1])<<8)
        | ((uint16_t)msp_ibuf[2]));
      i2c_j4_cache[0x0C].cache = (  // read P MSB from msp_ibuf and update cache
          ((uint16_t)msp_ibuf[0]));
      ms5607.D1 = (((uint32_t)i2c_j4_cache[0x0C].cache)<<16)
        | ((uint32_t)i2c_j4_cache[0x0B].cache);  // Update ms5607.D1 for P calculation
      ms5607.state = ms5607_convt;
      return true;

    case ms5607_convt:
      msp_conv_D2_osr[0] = MSP_CONV_D2 + (2 * i2c_j4_cache[0x0F].cache); // Update CONV_D1 cmd with OSR offset
			i2c_write(MSP_I2C_ADDR, msp_conv_D2_osr, 1); // Send Convert D2 (T)
      ms5607.state = ms5607_convt_tx;
      return false;

    case ms5607_convt_tx:
      ms5607.endtime = rtc_current_count + ms5607.delay ;
      ms5607.state = ms5607_convt_delay;
      return false;

    case ms5607_convt_delay:
  	  if ( rtc_current_count <= ms5607.endtime ) return false;
      ms5607.state = ms5607_readt;
      return true;

    case ms5607_readt:
			i2c_write(MSP_I2C_ADDR, msp_adc_read, 1);	
      ms5607.state = ms5607_readt_tx;
      return false;

    case ms5607_readt_tx:
			i2c_read(MSP_I2C_ADDR, msp_ibuf, 3);
      ms5607.state = ms5607_readt_cache;
      return false;

    case ms5607_readt_cache:
      i2c_j4_cache[0x0D].cache = (
         (((uint16_t)msp_ibuf[1])<<8)
        | ((uint16_t)msp_ibuf[2])); // read T LSW from msp_ibuf and update cache
      i2c_j4_cache[0x0E].cache = (
          ((uint16_t)msp_ibuf[0])); // read T MSB from msp_ibuf and update cache
      ms5607.D2 = ((uint32_t)i2c_j4_cache[0x0E].cache)<<16
        | ((uint32_t)i2c_j4_cache[0x0D].cache);	// Update ms5607.D2 for T calculation

      // Perform Compensation calculations here and update cache
      dT = ((float)ms5607.D2) - ((float)(ms5607.cal[5]) * pow2(8));
      OFF = ((float)(ms5607.cal[2]) * pow2(17)) + (dT * ((float)(ms5607.cal[4]))) / pow2(6);
      SENS = ((float)(ms5607.cal[1]) * pow2(16)) + (dT * ((float)(ms5607.cal[3]))) / pow2(7);
      ms5607.T = ( 2000 + ((dT * (float)(ms5607.cal[6])) / pow2(23))) / 100;  // degC
      ms5607.P = ((((float)(ms5607.D1) * SENS) / pow2(21)) - OFF) / pow2(15) / 100; // mBar

      sb_cache_update32(i2c_j4_cache, 1, &ms5607.P);	// Update cache P
      sb_cache_update32(i2c_j4_cache, 3, &ms5607.T);	// Update cache T

      ms5607.state = ms5607_convp;	// return to perform next P reading
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
   return true;
}


/****************************************************
 *	SHT25 Driver State Machine
 *	sht25_init - Reset sht25 to initialize
 *	sht25_convt - Send Measure Temperature Command
 *	sht25_readt - Read Temperature
 *	sht25_convrh - Send Measure Relative Humidity Command
 *	sht25_readrh - Read Relative Humidity
 */
enum sht25_state_t {
        sht25_init, sht25_init_tx, sht25_init_delay,
        sht25_convt, sht25_convt_tx, sht25_convt_delay,
        sht25_readt, sht25_readt_cache,
        sht25_convrh, sht25_convrh_tx, sht25_convrh_delay,
        sht25_readrh, sht25_readrh_cache
        };

typedef struct {
  bool enabled;
  enum sht25_state_t state;
  uint16_t RH; 	//  Relative Humidity
  float T; 	//  Temperature
  uint32_t endtime;
//  uint32_t delay;
//  uint16_t current;
} sht25_poll_def;

static sht25_poll_def sht25 = {
    I2C_SHT25_ENABLED, sht25_init
//	, 0, 0
//	, 0, 0
};

/**
 * poll_sht25() is only called when I2C_txfr_complete = true
 *    and I2C bus is free
 * return true if we are relinquishing the I2C bus
 */
static bool poll_sht25(void) {
  if (!I2C_SHT25_ENABLED || !I2C_txfr_complete ) return true;
  switch (sht25.state) {
  //  Reset SHT25
    case sht25_init:
			i2c_write(SHT_I2C_ADDR, msp_reset_cmd, 1);
      sht25.state = sht25_init_tx;
      return false;

    case sht25_init_tx:
      sht25.endtime = rtc_current_count + ( 15 * RTC_COUNTS_PER_MSEC ); // Reset < 15mS
      sht25.state = sht25_init_delay;
      return false;

    case sht25_init_delay:
      if ( rtc_current_count <= sht25.endtime ) return false;
      sht25.state = sht25_convt;
      return true;

  //  Process SHT25
    case sht25_convt:
	  i2c_write(SHT_I2C_ADDR, sht_meas_t, 1);	
      sht25.state = sht25_convt_tx;
      return false;

    case sht25_convt_tx:
      sht25.endtime = rtc_current_count + ( 85 * RTC_COUNTS_PER_MSEC ); // Tmax delay 85mS
      sht25.state = sht25_convt_delay;
      return false;

    case sht25_convt_delay:
  	  if ( rtc_current_count <= sht25.endtime ) return false;
      sht25.state = sht25_readt;
      return true;

    case sht25_readt:
			i2c_read(SHT_I2C_ADDR, sht_ibuf, 3);  // 3rd byte is checksum
      sht25.state = sht25_readt_cache;
      return false;

    case sht25_readt_cache:
      sht25.T = -46.85 + (175.72) *   // convert to 14b (16b) and calculate T
        ((((uint16_t)sht_ibuf[0])<<8) | ((uint16_t)(sht_ibuf[1] & 0xFC))) / pow2(16);
      sb_cache_update32(i2c_j4_cache, 0x11, &sht25.T);	// Update cache T
      sht25.state = sht25_convrh;
      return true;

    case sht25_convrh:
	  i2c_write(SHT_I2C_ADDR, sht_meas_rh, 1);	
      sht25.state = sht25_convrh_tx;
      return false;

    case sht25_convrh_tx:
      sht25.endtime = rtc_current_count + ( 29 * RTC_COUNTS_PER_MSEC ); // RHmax delay 29mS
      sht25.state = sht25_convrh_delay;
      return false;

    case sht25_convrh_delay:
  	  if ( rtc_current_count <= sht25.endtime ) return false;
      sht25.state = sht25_readrh;
      return true;

    case sht25_readrh:
			i2c_read(SHT_I2C_ADDR, sht_ibuf, 3);  // 3rd byte is checksum
      sht25.state = sht25_readrh_cache;
      return false;

    case sht25_readrh_cache:
      sht25.RH = -600 + 12500 *   // convert to 12b (16b) and Calculate '% * 100'
        ((((uint16_t)sht_ibuf[0])<<8) | ((uint16_t)(sht_ibuf[1] & 0xFC))) / pow2(16);
      i2c_j4_cache[0x10].cache = sht25.RH;  // update cache
      sht25.state = sht25_convt;
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
  return true;
}

// i2c functions

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&UC_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(UC_I2C_io, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&UC_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(UC_I2C_io, ibuf, nbytes);
}

void i2c_j4_enable(bool value) {
  i2c_j4_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  if (sb_cache_was_read(i2c_j4_cache, I2C_J4_STATUS_OFFSET)) {
    sb_cache_update(i2c_j4_cache, I2C_J4_STATUS_OFFSET, 0);
  }
  if (I2C_error >= -7 && I2C_error <= -2) {
    uint16_t val = i2c_j4_cache[I2C_J4_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_error));
    sb_cache_update(i2c_j4_cache, I2C_J4_STATUS_OFFSET, val);
  }
	//  Need diffeerent pins here. Not DADC ***
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(UC_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(UC_I2C.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void i2c_j4_reset() {
  if (!sb_i2c_j4.initialized) {
    UC_I2C_init();
    i2c_m_async_get_io_descriptor(&UC_I2C, &UC_I2C_io);
    i2c_m_async_enable(&UC_I2C);
    i2c_m_async_register_callback(&UC_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_async_error);
    i2c_m_async_register_callback(&UC_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);
    i2c_m_async_register_callback(&UC_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);

    sb_i2c_j4.initialized = true;
  }
}
//  End of I2C functions

//	UC_I2C Driver
void UC_I2C_PORT_init(void)
{
	gpio_set_pin_pull_mode(UC_SDA, GPIO_PULL_OFF);
	gpio_set_pin_function(UC_SDA, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(UC_SCL, GPIO_PULL_OFF);
	gpio_set_pin_function(UC_SCL, PINMUX_PA23C_SERCOM3_PAD1);
}

void UC_I2C_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
	_gclk_enable_channel(SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC);
}

void UC_I2C_init(void)
{
	UC_I2C_CLOCK_init();
	i2c_m_async_init(&UC_I2C, SERCOM3);
	UC_I2C_PORT_init();
}
//	End of UC_I2C Driver

// Main poll loop

enum i2c_state_t {i2c_ms5607, i2c_sht25 };
static enum i2c_state_t i2c_state = i2c_ms5607;

void i2c_j4_poll(void) {
	// cycle between ms5607 and J4
  if (!i2c_j4_enabled) return;
	  if (true) {	//  do we need condition here? 
    switch (i2c_state) {
      case i2c_ms5607:
        if (poll_ms5607()) {
          i2c_state = i2c_sht25;
        }
        break;
      case i2c_sht25:
        if (poll_sht25()) {
          i2c_state = i2c_ms5607;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
  }
}

subbus_driver_t sb_i2c_j4 = {
  I2C_J4_BASE_ADDR, I2C_J4_HIGH_ADDR, // address range
  i2c_j4_cache,
  i2c_j4_reset,
  i2c_j4_poll,
  0, // Dynamic function
  false // initialized
};
