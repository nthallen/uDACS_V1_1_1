/* ***********************************************************************
 * \file spi_PS.h                                                   
 *   Declare various constants, structures and methods for the Pressure
 *   Sensor on the uDACS PMOD connector SPI channel
 *
*/ 
#ifndef SPI_PS_H_INCLUDED
#define SPI_PS_H_INCLUDED
#include "driver_init.h"
#include "subbus.h"

#define MAX_SPI_XFR_LENGTH 32            // All transfers full duplex, max length
#define PS_SPI_ENABLE_DEFAULT true

// Subbus Address Space
#define PS_SPI_BASE_ADDR           0x32
#define PS_SPI_HIGH_ADDR           0x3E

#define DEGREE_POLYNOMIAL            4   // 4 coefficients for a 3rd deg polynomial
#define ADC_CONFIG_SIZE              4   // Number of ADC Config. Registers

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

// Various Unit Conversions and Gain, offset corrections
#define T_Gain                       0.03125   // Honeywell Deg C / AD count
#define TORR_PER_INH2O               1.8683204 // Unit Conversion
#define TORR_PER_PSI                68.9475728 // Unit Conversion
#define MBAR_PER_INH2O               2.4908891 // Unit Conversion
#define MBAR_PER_PSI                51.7149324 // Unit Conversion
#define MBAR_PER_TORR                1.3332237 // Unit Conversion
#define GAIN_MKS                     0.3150201 // MKS Baratron torr / count
#define OFF_MKS                     -1393      // MKS Baratron offset in counts

extern subbus_driver_t sb_PS_spi;
void PS_spi_enable(bool value);

#endif