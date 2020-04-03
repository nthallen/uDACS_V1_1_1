/* ***********************************************************************
 * \file spi_PS.h                                                   
 *   Declare various constants, structures and methods for the Pressure
 *   Sensor on the uDACS PMOD connector SPI channel
 *
*/ 
#ifndef SPI_PR_SN_H_
#define SPI_PR_SN_H_
#include "driver_init.h"
#include "subbus.h"

#define PS_SPI_MAX_XFR_LENGTH      44   
#define PS_SPI_ENABLE_DEFAULT      false

// Subbus Address Space
#define PS_SPI_BASE_ADDR           0x32
#define PS_SPI_HIGH_ADDR           0x3E

#define DEGREE_POLYNOMIAL            4   // 4 coefficients for a 3rd deg polynomial

// PS EEPROM address Map
#define PARTNUMBER_ADDR              0
#define SERIALNUMBER_ADDR            16
#define PRESSURERANGE_ADDR           27
#define PRESSUREMIN_ADDR             31
#define PRESSUREUNITS_ADDR           35
#define PRESSUREREF_ADDR             40
#define RESERVED_ADDR                41
#define ADCCONFIG_ADDR               61
#define OFF_COEFF_ADDR               130
#define SPAN_COEFF_ADDR              210
#define SHAPE_COEFF_ADDR             290
#define CHECKSUM_ADDR                450
#define EEPROM_END_ADDR              451

// PS EEPROM Element Size Information
#define PART_NO_SIZE                 (SERIALNUMBER_ADDR - PARTNUMBER_ADDR)
#define SERIAL_NO_SIZE               (PRESSURERANGE_ADDR - SERIALNUMBER_ADDR)
#define PRESSURE_UNIT_SIZE           (PRESSUREREF_ADDR - PRESSUREUNITS_ADDR)
#define PRESSURE_REF_SIZE            (RESERVED_ADDR - PRESSUREREF_ADDR)
#define MAX_EEPROM_SIZE              (EEPROM_END_ADDR + 1)
#define ADC_CONFIG_SIZE              8 // Number of ADC Config. Registers bytes
#define EEPROM_FLOAT_SIZE            4 // Size of any Floats stored in EEPROM

#define RESET_AD					 6 // Reset ADC command
#define WR_AD_REG_ALL               67 // Write ADC Reg cmd = 0b 0100 0011, all four reg's
									   // To program a configuration register, the host sends a
									   // WREG command [0100 RRNN], where
									   //            RR is the Register Number
									   //            NN is the (#bytes write)-1 (auto increments)
									   //   67 = 0x43 = All 4 registers starting at Reg 0
#define WR_AD_REG_MODE				68 //   68 = 0x44 =     1 Register  starting at Reg 1
#define PS_AD_MODE_T			   134 // 0b 100 (330 Hz) 00 (normal) 1 (fixed) 1 (temperature) 0 (fixed)
#define PS_AD_MODE_P			   132 // 0b 100 (330 Hz) 00 (normal) 1 (fixed) 0 (pressure   ) 0 (fixed)
#define PS_START_CNV				 8 // Start Conversion Command	
// delays
#define AD_RESET_DELAY				 1 // # 1ms clocks needed to insure Reset CMD took
#define WAIT_CONVERSION			     4 // # 1ms clocks needed to insure Conversion is ready at 330 Samples/sec
                       
									   
// Various Unit Conversions and Gain, offset corrections
#define T_GAIN                       0.03125   // Honeywell Deg C / AD count
#define TORR_PER_INH2O               1.8683204 // Unit Conversion
#define TORR_PER_PSI                68.9475728 // Unit Conversion
#define MBAR_PER_INH2O               2.4908891 // Unit Conversion
#define MBAR_PER_PSI                51.7149324 // Unit Conversion
#define MBAR_PER_TORR                1.3332237 // Unit Conversion
#define GAIN_MKS                     0.3150201 // MKS Baratron torr / count
#define OFF_MKS                     -1393      // MKS Baratron offset in counts

//extern subbus_driver_t sb_PS_spi;
void ps_spi_poll(void);
void ps_spi_reset(void);

#endif