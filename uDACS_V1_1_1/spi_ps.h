/* ***********************************************************************
 * \file spi_PS.h
 *   Declare various constants, structures and methods for the Pressure
 *   Sensor on the uDACS PMOD connector SPI channel
 *
*/
#ifndef SPI_PS_H_
#define SPI_PS_H_

#include <hal_spi_m_async.h>
#include "serial_num.h"
#include "uDACS_pins.h"
#include "rtc_timer.h"
#include "subbus.h"

#ifdef uDACS_B

extern struct spi_m_async_descriptor PS_SPI;
void PS_SPI_init(void);

// #define SUBBUS_MATLAB_TEST_ADDR     0x30
#define PS_SPI_STATUS_OFFSET         0
#define PS_SPI_FIFO_LEN_OFFSET       1
#define PS_SPI_FIFO_OFFSET           2
#define PS_SPI_T1_OFFSET             3
#define PS_SPI_P1 OFFSET             5
#define PS_SPI_T2_OFFSET             7
#define PS_SPI_P2_OFFSET             9
#define PS_NUM_WORDS_PER_CHANNEL   (PS_SPI_T2_OFFSET-PS_SPI_T1_OFFSET)   // Number of 16 bit words per channel
#define PS_SPI_BASE_ADDR          0x50
#define PS_SPI_HIGH_ADDR          0x5A

#define PS_SPI_MAX_XFR_LENGTH      44
#define PS_SPI_ENABLE_DEFAULT      false

#define DEGREE_POLYNOMIAL            4   // 4 coefficients for a 3rd deg polynomial

// PS EEPROM address Map
#define PARTNUMBER_EEADDR              0
#define SERIALNUMBER_EEADDR            16
#define PRESSURERANGE_EEADDR           27
#define PRESSUREMIN_EEADDR             31
#define PRESSUREUNITS_EEADDR           35
#define PRESSUREREF_EEADDR             40
#define RESERVED_EEADDR                41
#define ADCCONFIG_EEADDR               61
#define OFF_COEFF_EEADDR               130
#define SPAN_COEFF_EEADDR              210
#define SHAPE_COEFF_EEADDR             290
#define CHECKSUM_EEADDR                450
#define EEPROM_END_EEADDR              451

// PS EEPROM Element Size Information
#define PART_NO_SIZE                 (SERIALNUMBER_EEADDR - PARTNUMBER_EEADDR)
#define SERIAL_NO_SIZE               (PRESSURERANGE_EEADDR - SERIALNUMBER_EEADDR)
#define PRESSURE_UNIT_SIZE           (PRESSUREREF_EEADDR - PRESSUREUNITS_EEADDR)
#define PRESSURE_REF_SIZE            (RESERVED_EEADDR - PRESSUREREF_EEADDR)
#define MAX_EEPROM_SIZE              (EEPROM_END_EEADDR + 1)
#define ADC_CONFIG_SIZE              8 // Number of ADC Config. Registers bytes
#define EEPROM_FLOAT_SIZE            4 // Size of any Floats stored in EEPROM

#define PN_SIZE 18
#define SN_SIZE 12
#define UNITS_SIZE 6
#define REF_SIZE 2

#define compile_time_assert(cond,msg) extern char msg[(cond)?1:-1];
compile_time_assert(PN_SIZE >= PART_NO_SIZE+1, PN_SIZE_NOT_LARGE_ENOUGH)
compile_time_assert(SN_SIZE >= SERIAL_NO_SIZE+1, SN_SIZE_NOT_LARGE_ENOUGH)
compile_time_assert(UNITS_SIZE >= PRESSURE_UNIT_SIZE+1, UNITS_SIZE_NOT_LARGE_ENOUGH)
compile_time_assert(REF_SIZE >= PRESSURE_REF_SIZE+1, REF_SIZE_NOT_LARGE_ENOUGH)

#define RESET_AD            6 // Reset ADC command
#define WR_AD_REG_ALL      67 // Write ADC Reg cmd = 0b 0100 0011, all four reg's
                              // To program a configuration register, the host sends a
                              // WREG command [0100 RRNN], where
                              //            RR is the Register Number
                              //            NN is the (#bytes write)-1 (auto increments)
                              //   67 = 0x43 = All 4 registers starting at Reg 0
#define WR_AD_REG_MODE     68 //   68 = 0x44 =     1 Register  starting at Reg 1
#define PS_AD_MODE_T_330  134 // 0b 100 (330 Hz) 00 (normal) 1 (fixed) 1 (temperature) 0 (fixed)
#define PS_AD_MODE_P_330  132 // 0b 100 (330 Hz) 00 (normal) 1 (fixed) 0 (pressure   ) 0 (fixed)
#define PS_AD_MODE_T_20     6 // 0b 000 ( 20 Hz) 00 (normal) 1 (fixed) 1 (temperature) 0 (fixed)
#define PS_AD_MODE_P_20     4 // 0b 000 ( 20 Hz) 00 (normal) 1 (fixed) 0 (pressure   ) 0 (fixed)
#define PS_START_CNV        8 // Start Conversion Command
// delays
#define AD_RESET_DELAY         (1*RTC_COUNTS_PER_MSEC)  // # 1ms clocks needed to insure Reset CMD took
#define WAIT_CONVERSION_330    (4*RTC_COUNTS_PER_MSEC)  // # 1ms clocks needed to insure Conversion is ready at 330 Samples/sec
#define WAIT_CONVERSION_20     (51*RTC_COUNTS_PER_MSEC) // # 1ms clocks needed to insure Conversion is ready at 20 Samples/sec

// Various Unit Conversions and Gain, offset corrections
#define T_GAIN                       0.03125   // Honeywell Deg C / AD count
#define TORR_PER_INH2O               1.8683204 // Unit Conversion
#define TORR_PER_PSI                68.9475728 // Unit Conversion
#define MBAR_PER_INH2O               2.4908891 // Unit Conversion
#define MBAR_PER_PSI                51.7149324 // Unit Conversion
#define MBAR_PER_TORR                1.3332237 // Unit Conversion
#define GAIN_MKS                     0.3150201 // MKS Baratron torr / count
#define OFF_MKS                     -1393      // MKS Baratron offset in counts

// Subbus Address Space

extern subbus_driver_t sb_ps_spi;
void ps_spi_poll(void);
void ps_spi_reset(void);

#endif // uDACS_B
#endif