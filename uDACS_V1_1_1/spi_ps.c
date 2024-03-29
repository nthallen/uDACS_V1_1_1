
#include <hal_spi_m_async.h>
#include <hal_gpio.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include "uDACS_pins.h"
#include "subbus.h"
#include "spi_ps.h"

#ifdef uDACS_B

#define N_PSENSORS 3

/* *************************************************************************
 * CCTIT Checksum and Checksum Table with 0x1021 seed
*/
static const uint16_t gu16Crc16Table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

// Pressure sensors can operate both full or half duplex
// Predefine buffers that hold "to-write" bytes
//               and collect "being-read" bytes
//
static uint8_t PS_xfr_Rbuf[PS_SPI_MAX_XFR_LENGTH] = {
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

static uint8_t PS_xfr_Wbuf[PS_SPI_MAX_XFR_LENGTH] = {
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

static volatile uint8_t PS_SPI_txfr_complete = true;   // async transfer complete?
static bool ps_spi_enabled = PS_SPI_ENABLE_DEFAULT;    // PS SPI comms enabled?

// Declare EEPROM structure - holds EEPROM Contents locally
typedef struct {
//  uint8_t       PartNumber[PART_NO_SIZE];
//  uint8_t       SerialNumber[SERIAL_NO_SIZE];
  float         Pressure_Range;
  float         Pressure_Min;
//  uint8_t       PressureUnit[PRESSURE_UNIT_SIZE];
//  uint8_t       PressureRef[PRESSURE_REF_SIZE];
  uint8_t       ADC_Config[ADC_CONFIG_SIZE];
  float         Off_Coeff[DEGREE_POLYNOMIAL];
  float         Spn_Coeff[DEGREE_POLYNOMIAL];
  float         Shp_Coeff[DEGREE_POLYNOMIAL];
  uint16_t      CheckSum;
  float         temperature;
  int32_t       pressure_raw;
  float         pressure;
  int16_t       temperature_raw;
  uint8_t       EE_CS;
  uint8_t       AD_CS;
} EEPROM_t;
static EEPROM_t proms[N_PSENSORS], *prom_p;

typedef struct __attribute__((__packed__)) {
  uint8_t       PartNumber[PN_SIZE];
  uint8_t       SerialNumber[SN_SIZE];
  uint8_t       PressureUnit[UNITS_SIZE];
  uint8_t       PressureRef[REF_SIZE];
  float         Pressure_Range;
  float         Pressure_Min;
} sensor_specs_t;
static sensor_specs_t sensors[N_PSENSORS];
static void sensor_cfg_init(void);

// Pressure sensors require two SPI modes:
//    MODE 0 when reading from EEPROM
//    MODE 3 when reading Pressure or Temperature values
//
static enum spi_transfer_mode PS_spi_current_transfer_mode = SPI_MODE_0;

/* **************************************************************************
 * Base Functions for PS_SPI transfers
 *
 */
static inline void ps_chip_select(uint8_t pin) {
  gpio_set_pin_level(pin, false);
}
static inline void ps_chip_deselect(uint8_t pin) {
  gpio_set_pin_level(pin, true);
}

// SPI Hardware Done Interrupt callback function
static void complete_cb_ps_spi(const struct spi_m_async_descriptor *const io_descr) {
  PS_SPI_txfr_complete = true;
}

// Kick off SPI Transfer, non-blocking
static void ps_start_spi_transfer(uint8_t pin, int length) {
  ps_chip_select(pin);
  PS_SPI_txfr_complete = false;
  spi_m_async_transfer(&PS_SPI, PS_xfr_Wbuf, PS_xfr_Rbuf, length);
}

/* **************************************************************************
 * Pressure Sensor "poll" State Machine supporting functions
 */

// EEPROM spans a 9 Bit address space, upper bit is embedded in Read cmd
//
static inline void prom_cmd_addr(uint16_t addr) {
  PS_xfr_Wbuf[0] = (addr <= 0xFF ? 0x03 : 0x0B);
  PS_xfr_Wbuf[1] = (uint8_t)addr;
}

// transfer uint8_t block of length m to/from EEPROM
//
void txfr_block(uint8_t pin, int addr, uint8_t numBytes) {
  prom_cmd_addr(addr);
  ps_start_spi_transfer(pin, numBytes);
}

// 4 EEPROM bytes convert to float
//
float bytes2float32(int addr) {
  unsigned long dat32 = (PS_xfr_Rbuf[addr+3] << 24) + (PS_xfr_Rbuf[addr+2] << 16)
                      + (PS_xfr_Rbuf[addr+1] <<  8) + (PS_xfr_Rbuf[addr]);
  float val = *((float*)&dat32);
  return(val);
}

// sort PS_xfr_Rbuf Manufacturer ID info into Local EEPROM Structure
//
 void sort_PS_EEPROM_info(int prom_num) {
   EEPROM_t *prom_p = &proms[prom_num];
   sensor_specs_t *sensor = &sensors[prom_num];

   int ii;            // member index
   int jj;            // buffer index 1st read from EEPROM is 2 writes delayed

   for(ii=0, jj=2; ii<PART_NO_SIZE;   ii++) { sensor->PartNumber[ii]   = PS_xfr_Rbuf[jj++]; }
   sensor->PartNumber[ii] = '\0'; // Add terminating NUL
   for(ii=0; ii<SERIAL_NO_SIZE; ii++) { sensor->SerialNumber[ii] = PS_xfr_Rbuf[jj++]; }
   sensor->SerialNumber[ii] = '\0'; // Add terminating NUL

   prom_p->Pressure_Range = bytes2float32(jj);
   sensor->Pressure_Range = prom_p->Pressure_Range;
   jj = jj+EEPROM_FLOAT_SIZE;

   prom_p->Pressure_Min = bytes2float32(jj);
   sensor->Pressure_Min = prom_p->Pressure_Min;
   jj = jj+EEPROM_FLOAT_SIZE;

   for(ii=0; ii<PRESSURE_UNIT_SIZE; ii++) { sensor->PressureUnit[ii] = PS_xfr_Rbuf[jj++]; }
   sensor->PressureUnit[ii] = '\0';
   for(ii=0; ii<PRESSURE_REF_SIZE;  ii++) { sensor->PressureRef[ii]  = PS_xfr_Rbuf[jj++]; }
   sensor->PressureRef[ii] = '\0';

   // if (which_prom)
   //   sensor_cfg_init();
}

// sort PS_xfr_Rbuf Polynomial coefficients into Local EEPROM Structure
//
void sort_PS_EEPROM_coeffs(uint8_t poly, EEPROM_t *prom_p) {
  uint8_t jj = 2;            // index into PS_xfr_Rbuf, 1st 2 writes return junk per Vendor spec.
  float temp[DEGREE_POLYNOMIAL];    // temporarily hold the coefficients
  for(uint8_t ii=0; ii<DEGREE_POLYNOMIAL; ii++) {
    temp[ii] = bytes2float32(jj);
    jj = jj+EEPROM_FLOAT_SIZE;
  }
  switch(poly) {
    case 1:
      for(uint8_t ii=0; ii<DEGREE_POLYNOMIAL; ii++) { prom_p->Off_Coeff[ii] = temp[ii]; }
      break;
    case 2:
      for(uint8_t ii=0; ii<DEGREE_POLYNOMIAL; ii++) { prom_p->Spn_Coeff[ii] = temp[ii]; }
      break;
    case 3:
      for(uint8_t ii=0; ii<DEGREE_POLYNOMIAL; ii++) { prom_p->Shp_Coeff[ii] = temp[ii]; }
      break;
  }
}

// Select which coefficients are being retrieved from EEPROM
//
void get_coef(uint8_t poly, uint8_t pin) {
  switch(poly) {
    case 1:
      txfr_block(pin, OFF_COEFF_EEADDR, (EEPROM_FLOAT_SIZE*DEGREE_POLYNOMIAL)+2);
      break;
    case 2:
      txfr_block(pin, SPAN_COEFF_EEADDR, (EEPROM_FLOAT_SIZE*DEGREE_POLYNOMIAL)+2);
      break;
    case 3:
      txfr_block(pin, SHAPE_COEFF_EEADDR, (EEPROM_FLOAT_SIZE*DEGREE_POLYNOMIAL)+2);
      break;
  }
}

// calculate compensated pressure, using temperature data and PROM coefficients
//
void use_coef(EEPROM_t *sensor) {
  float t1 =(float)sensor->temperature_raw;
  float t2 = t1*t1;
  float t3 = t2*t1;
  // first correct for offset using 3rd order poly and raw temperature data
  float x = sensor->Off_Coeff[3] * t3;
  float y = sensor->Off_Coeff[2] * t2;
  float z = sensor->Off_Coeff[1] * t1;
  sensor->pressure = sensor->pressure_raw - (x + y + z + sensor->Off_Coeff[0]);

  // then correct for Gain using 3rd order poly and raw temperature data
  x = sensor->Spn_Coeff[3] * t3;
  y = sensor->Spn_Coeff[2] * t2;
  z = sensor->Spn_Coeff[1] * t1;
  sensor->pressure = sensor->pressure / (x + y + z + sensor->Spn_Coeff[0]);

  // then correct for nonlinearity using 3rd order poly shape coefficients
  x = (sensor->Shp_Coeff[3] * sensor->pressure * sensor->pressure * sensor->pressure);
  y = (sensor->Shp_Coeff[2] * sensor->pressure * sensor->pressure);
  z = (sensor->Shp_Coeff[1] * sensor->pressure);
  sensor->pressure = x + y + z + sensor->Shp_Coeff[0];

  // Finally normalize to full scale range (max and min from EEPROM)
  sensor->pressure = (sensor->pressure * sensor->Pressure_Range) + sensor->Pressure_Min;
}

// Define read / write-able Host Cache Interface
static subbus_cache_word_t ps_spi_cache[PS_SPI_HIGH_ADDR - PS_SPI_BASE_ADDR+1] = {
  { 0, 0, true,  false,  false,  false, false },  // R,   PS_SPI_STATUS

  { 0, 0, true,  false,  false,  false, false },  // Sensor Config FIFO Len
  { 0, 0, true,  false,  false,  false, true  },  // Sensor Config FIFO

  { 0, 0, true,  false,  false,  false, false },  // LSW, Float, Temperature,  Pressure Sensor 1
  { 0, 0, true,  false,  false,  false, false },  // MSW, Float, Temperature,  Pressure Sensor 1
  { 0, 0, true,  false,  false,  false, false },  // LSW, Float, Pressure,  Pressure Sensor 1
  { 0, 0, true,  false,  false,  false, false },  // MSW, Float, Pressure,  Pressure Sensor 1

  { 0, 0, true,  false,  false,  false, false },  // LSW, Float, Temperature,  Pressure Sensor 2
  { 0, 0, true,  false,  false,  false, false },  // MSW, Float, Temperature,  Pressure Sensor 2
  { 0, 0, true,  false,  false,  false, false },  // LSW, Float, Pressure,  Pressure Sensor 2
  { 0, 0, true,  false,  false,  false, false },  // MSW, Float, Pressure,  Pressure Sensor 2

  { 0, 0, true,  false,  false,  false, false },  // LSW, Float, Temperature,  Pressure Sensor 3
  { 0, 0, true,  false,  false,  false, false },  // MSW, Float, Temperature,  Pressure Sensor 3
  { 0, 0, true,  false,  false,  false, false },  // LSW, Float, Pressure,  Pressure Sensor 3
  { 0, 0, true,  false,  false,  false, false },  // MSW, Float, Pressure,  Pressure Sensor 3
};
//  0,    0,        true,       false,     true,      false,     false,
// .cache   .wvalue   .readable   .was_read  .writable  .written   .dynamic

void ps_cache_update(EEPROM_t *sensor, uint8_t prom_num){
  uint8_t indx = (prom_num*PS_NUM_WORDS_PER_CHANNEL)+PS_SPI_T1_OFFSET;

  sb_cache_update32(ps_spi_cache, indx, &prom_p->temperature);
  sb_cache_update32(ps_spi_cache, indx+2, &prom_p->pressure);
}

/* *****************************************************************************
 * Honeywell RSC series Pressure Sensor Driver State machine - Poll Function.
 * Main loop = state clock, i.e. evaluate inputs, act, decide next state
 *             each time Main loop calls driver's poll function
 *
 */

// Define states and conditional variables for Pressure Sensor State Machine (ps_sm)
//
enum ps_sm_t { read_part_info,    sort_part_info,
         read_poly_coeffs,  sort_coeffs,
         read_config_AD,    sort_config_AD,
         read_checksum,    sort_checksum,
         calc_checksum_read,  calc_checksum_calc,
         bad_checksum,
         reset_ADs,      delay_AD_reset,
         wr_AD_config,    wait_AD_config,
         rd_t_set_p,      sort_t,        wait_pressure,
         rd_p_set_t,      sort_p,        wait_temperature }

ps_sm = read_part_info;                // initial state

uint8_t  prom_num       = 0;            // 0 = Sensor/EEPROM-1, 1 = Sensor/EEPROM-2, 2 = Sensor/EEPROM-3
uint8_t  pin_cs         = EEP1_CS;          // 1 of 4 Sensor Chip Selects, start with EEPROM1
uint8_t  poly_type      = 1;            // which polynomial, 1 = offset, 2 = span, 3 = shape
uint16_t check_count    = 0;            // track # bytes read from EEPROM for CRC calculation
uint16_t CRC16_Computed = 0xFFFF;          // computed checksum on EEPROM - initialized to prescribed start

// for absolute delay time tracking
uint32_t timer_ps_sm    = 0;            // hold time read from real time rtc_current_count variable
uint32_t AD_reset_count = 0;            // elapsed time tracking to assure AD_RESET command completed
uint32_t wait_cnvt_cnt  = 0;            // elapsed time tracking to assure AD conversion is completed

void ps_spi_poll(void) {
  if ( !ps_spi_enabled || !PS_SPI_txfr_complete ) {
    return;
  }
  switch(ps_sm) {
    case read_part_info:            // Read Sensor's ASCII Manufacturer's Info
      prom_p = &proms[prom_num]; // (prom_num == 1) ? &prom_1 : &prom_2;
      pin_cs = prom_p->EE_CS; // (prom_num == 0) ? EEP1_CS : EEP2_CS;
      txfr_block(pin_cs, PARTNUMBER_EEADDR, RESERVED_EEADDR-PARTNUMBER_EEADDR+2);
      ps_sm = sort_part_info;
      break;

    case sort_part_info:            // Sort SPI read-buffer into local structure
      ps_chip_deselect(pin_cs);
      sort_PS_EEPROM_info(prom_num);
      if (prom_num >= N_PSENSORS-1)
        sensor_cfg_init();
      poly_type = 1;
      ps_sm = read_poly_coeffs;
      break;

    case read_poly_coeffs:                      // Read current type Polynomial coefficients from EEPROM
      get_coef(poly_type, pin_cs);
      ps_sm = sort_coeffs;
      break;

    case sort_coeffs:              // Sort SPI read-buffer into local structure
      ps_chip_deselect(pin_cs);
      sort_PS_EEPROM_coeffs(poly_type, prom_p);
      poly_type++;
      ps_sm = (poly_type < 4) ? read_poly_coeffs : read_config_AD;  // all poly types read in?
      break;

    case read_config_AD:            // Read EEPROM's ADC default Config
      txfr_block(pin_cs, ADCCONFIG_EEADDR, ADC_CONFIG_SIZE+2);
      ps_sm = sort_config_AD;
      break;

    case sort_config_AD:            // Sort SPI read-buffer into local structure
      ps_chip_deselect(pin_cs);
      for(uint8_t ii=0; ii<4; ii++) {  prom_p->ADC_Config[ii] = PS_xfr_Rbuf[ii*2+2]; } // 4 Registers
      ps_sm = read_checksum;
      break;

    case read_checksum:              // Read stored EEPROM Checksum value
      txfr_block(pin_cs, CHECKSUM_EEADDR, 2+2);
      ps_sm = sort_checksum;
      break;

    case sort_checksum:              // Sort SPI read-buffer into local structure
      ps_chip_deselect(pin_cs);
      prom_p->CheckSum = (PS_xfr_Rbuf[3] << 8) + PS_xfr_Rbuf[2];
      check_count = 0;            // bytes read from EEPROM for CRC calculation
      CRC16_Computed = 0xFFFF;        // computed checksum on EEPROM - initial value
      ps_sm = calc_checksum_read;
      break;

    case calc_checksum_read:          // Read 1 byte at a time from  EEPROM, all of EEPROM
      txfr_block(pin_cs, check_count, 1+2);
      ps_sm = calc_checksum_calc;
      break;

    case calc_checksum_calc:          // Generate running CRC calculation
      ps_chip_deselect(pin_cs);
      uint16_t itable = PS_xfr_Rbuf[2] ^ (CRC16_Computed >> 8);           // Get CRC table index
      CRC16_Computed = gu16Crc16Table[itable] ^ (CRC16_Computed << 8);    // Calculate the new CRC value
      check_count++;
      if (check_count < MAX_EEPROM_SIZE-2) {        // last address (excluding last 2) in EEPROM?
        ps_sm = calc_checksum_read;                // no, get next address
      } else {
        // The the initialized bit in the status word
        sb_cache_update(ps_spi_cache, PS_SPI_STATUS_OFFSET,
          ps_spi_cache[PS_SPI_STATUS_OFFSET].cache | (1<<(prom_num*2)));
        if (CRC16_Computed == prom_p->CheckSum) {  // checksum matches?
          // Good checksum, so update the CRC_OK bit
          sb_cache_update(ps_spi_cache, PS_SPI_STATUS_OFFSET,
             ps_spi_cache[PS_SPI_STATUS_OFFSET].cache | (1<<(prom_num*2+1)));
        }
//          ps_sm = bad_checksum;                  // no, fail
//        } else {
        if (prom_num < N_PSENSORS - 1) {              // last EEPROM tested?
          check_count = 0;                    // no, do it all on next EEPROM
          ++prom_num;
          ps_sm = read_part_info;
        } else {                      // EEPROMS done and passed! On to AD's
          spi_m_async_disable(&PS_SPI);            // AD's work in MODE_1
          spi_m_async_set_mode(&PS_SPI, SPI_MODE_1);
          PS_spi_current_transfer_mode = SPI_MODE_1;
          spi_m_async_enable(&PS_SPI);
          prom_num = 0;
          ps_sm = reset_ADs;
        }
      }
      break;

    case bad_checksum:
      ps_sm = bad_checksum;                // what to do ???????
      break;

    case reset_ADs:
      prom_p = &proms[prom_num]; // (prom_num == 1) ? &prom_1 : &prom_2;
      pin_cs = prom_p->AD_CS; // (prom_num == 0) ? ADC1_CS : ADC2_CS;
      PS_xfr_Wbuf[0] = RESET_AD;
      ps_start_spi_transfer(pin_cs, 1);
      timer_ps_sm = rtc_current_count;
      ps_sm = delay_AD_reset;
      break;

    case delay_AD_reset:
      ps_chip_deselect(pin_cs);
      AD_reset_count = rtc_current_count - timer_ps_sm;
      ps_sm = (AD_reset_count < AD_RESET_DELAY) ? delay_AD_reset : wr_AD_config;
      break;

    case wr_AD_config:
      PS_xfr_Wbuf[0] = WR_AD_REG_ALL;        // write_all_ADC_configuration_registers command
      PS_xfr_Wbuf[1] = prom_p->ADC_Config[0];    // must write EEPROM stored value to reg 0
      PS_xfr_Wbuf[2] = PS_AD_MODE_T_20;      // write mode reg to 330 Hz, Normal mode, convert Temperature
      PS_xfr_Wbuf[3] = prom_p->ADC_Config[2];      // must write EEPROM stored value to reg 2
      PS_xfr_Wbuf[4] = prom_p->ADC_Config[3];    // must write EEPROM stored value to reg 3
      ps_start_spi_transfer(pin_cs, 5);      // 5 bytes to transfer
      ps_sm = wait_AD_config;
      break;

    case wait_AD_config:
      ps_chip_deselect(pin_cs);
      if (prom_num < N_PSENSORS-1) {
        ++prom_num;
        ps_sm = reset_ADs;
      } else {
        prom_num = 0;
        PS_xfr_Wbuf[0] = WR_AD_REG_MODE;            // Command = Write_AD_Mode_Register
        PS_xfr_Wbuf[2] = PS_START_CNV;        // Command = Start_Convert
        ps_sm = rd_t_set_p;
      }
      break;

    case rd_t_set_p:
      prom_p = &proms[prom_num]; // (prom_num == 1) ? &prom_1 : &prom_2;  // select which sensor
      pin_cs = prom_p->AD_CS; // (prom_num == 0) ? ADC1_CS : ADC2_CS;
      PS_xfr_Wbuf[1] = PS_AD_MODE_P_20;          // Mode -> convert Pressure
      ps_start_spi_transfer(pin_cs, 3);        // send the 3 bytes, read previous conversion.
      timer_ps_sm = rtc_current_count;            // get time of start convert command (1ms resolution)
      ps_sm = sort_t;
      break;

    case sort_t:
      ps_chip_deselect(pin_cs);
      prom_p->temperature_raw = ((PS_xfr_Rbuf[0] << 8) + PS_xfr_Rbuf[1]);
      prom_p->temperature_raw /= 4;  // /4 after shift to preserve sign
      prom_p->temperature = T_GAIN * (float)prom_p->temperature_raw;
      ps_sm = wait_pressure;
      //    gpio_set_pin_level(BPMP_CNTL, true);           // pulse every state clock
      //    for(uint8_t ii=0; ii<5; ii++) {gpio_set_pin_level(BPMP_CNTL, true);}
      //    gpio_set_pin_level(BPMP_CNTL, false);
      break;

    case wait_pressure:
      wait_cnvt_cnt = rtc_current_count - timer_ps_sm;
      ps_sm = ( wait_cnvt_cnt < WAIT_CONVERSION_20 ) ? wait_pressure : rd_p_set_t;
      break;

    case rd_p_set_t:
      PS_xfr_Wbuf[1] = PS_AD_MODE_T_20;                // Mode payload - convert temperature
      ps_start_spi_transfer(pin_cs, 3);                // send the 3 bytes, read previous conversion.
      timer_ps_sm = rtc_current_count;                    // get time of start convert command issued in transfer
      ps_sm = sort_p;
      break;

    case sort_p:
      ps_chip_deselect(pin_cs);
      prom_p->pressure_raw = ((PS_xfr_Rbuf[0] << 16) + (PS_xfr_Rbuf[1] << 8) + PS_xfr_Rbuf[2]);
      prom_p->pressure_raw = (prom_p->pressure_raw << 8)/256;      // shift and /256 to preserve sign
      use_coef(prom_p);
      ps_cache_update(prom_p, prom_num);
      ps_sm = wait_temperature;
      //    gpio_set_pin_level(BPMP_CNTL, true);           // pulse every state clock
      //    for(uint8_t ii=0; ii<10; ii++) {gpio_set_pin_level(BPMP_CNTL, true);}
      //    gpio_set_pin_level(BPMP_CNTL, false);
      break;

    case wait_temperature:
      wait_cnvt_cnt = rtc_current_count - timer_ps_sm;
      if ( wait_cnvt_cnt < WAIT_CONVERSION_20 ) {
        ps_sm = wait_temperature;
      } else {
        ps_sm = rd_t_set_p;
        if (++prom_num == N_PSENSORS)
          prom_num = 0;
        // prom_num = prom_num ? 0 : 1;
      }
      break;

    default:
      ps_sm = read_part_info;
      break;
  }
}

static struct sensor_cfg_t {
  uint16_t *data;
  int cp;
  int nc;
} sensor_cfg;

static void sensor_cfg_init(void) {
  sensor_cfg.data = (uint16_t*)&sensors;
  sensor_cfg.cp = 0;
  sensor_cfg.nc = sizeof(sensors)/2;
  sb_cache_update(ps_spi_cache, PS_SPI_FIFO_LEN_OFFSET, sensor_cfg.nc);
  sb_cache_update(ps_spi_cache, PS_SPI_FIFO_OFFSET, sensor_cfg.data[sensor_cfg.cp++]);
}

static void sensor_cfg_action(uint16_t addr) {
  addr = addr;
  if (sb_cache_was_read(ps_spi_cache, PS_SPI_FIFO_OFFSET)) {
    if (sensor_cfg.cp >= sensor_cfg.nc) {
      sensor_cfg.cp = 0;
    }
    sb_cache_update(ps_spi_cache, PS_SPI_FIFO_LEN_OFFSET, sensor_cfg.nc-sensor_cfg.cp);
    sb_cache_update(ps_spi_cache, PS_SPI_FIFO_OFFSET, sensor_cfg.data[sensor_cfg.cp++]);
  }
}

/* ********************************************************************************
 * Norton Arch. Features
 *   Pressure Sensor Driver structure
 *                   Driver structure reset function
 */

// Reset Function - Register ISR and enable SPI Hardware resource for Pressure Sensors
void ps_spi_reset(void) {
  if (!ps_spi_enabled) {
    PS_SPI_init();
    spi_m_async_register_callback(&PS_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_ps_spi);
    spi_m_async_enable(&PS_SPI);
    proms[0].AD_CS = ADC1_CS;
    proms[1].AD_CS = ADC2_CS;
    proms[2].AD_CS = ADC3_CS;
    proms[0].EE_CS = EEP1_CS;
    proms[1].EE_CS = EEP2_CS;
    proms[2].EE_CS = EEP3_CS;
    // sensor_cfg_init();
    ps_spi_enabled = true;
  }
}

subbus_driver_t sb_ps_spi = {
  PS_SPI_BASE_ADDR, PS_SPI_HIGH_ADDR,
  ps_spi_cache,
  ps_spi_reset,
  ps_spi_poll,        // driver state machine
  sensor_cfg_action,  // action function for FIFO
  false
};

#endif // uDACS_B
