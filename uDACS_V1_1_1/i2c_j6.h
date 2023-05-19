/************************************************************************/
/* 2:16 PM 4/24/2023	file i2c_j6.h 
	
	I2C interface to uDACS J6 and on-board MS8607 PTRH measurement

/************************************************************************/
#ifndef I2C_J6_H_INCLUDED
#define I2C_J6_H_INCLUDED
// #include <hal_spi_m_async.h> // keep?
#include "subbus.h"

#define I2C_J6_ENABLE_DEFAULT true
#define I2C_J6_ENABLED true
#define I2C_MS8607_ENABLE_DEFAULT true
#define I2C_MS8607_ENABLED true
#define I2C_P6_ENABLE_DEFAULT false
#define I2C_P6_ENABLED false

#define I2C_J6_MAX_READ_LENGTH 16
#define I2C_J6_BASE_ADDR 0x60
#define I2C_J6_HIGH_ADDR 0x6E

#define ADC_LSW_OFFSET(x) (5+2*(x))
#define ADC_MSB_OFFSET(x) (6+2*(x))

#define MSP_I2C_ADDR 0x76
#define MSRH_I2C_ADDR 0x40

// MS8607 Pressure Commands
#define MSP_RESET 0x1E // ADC reset command
#define MSP_ADC_READ 0x00 // ADC read command
#define MSP_CONV_D1 0x40 // ADC D1 conversion command
#define MSP_CONV_D2 0x50 // ADC D2 conversion command
#define MSP_OSR_OFFS 0x05 // ADC OSR offset (Default = 8192)
//	#define MSP_ADC_256 0x00 // ADC OSR=256	Offset 0
//	#define MSP_ADC_512 0x02 // ADC OSR=512	Offset 1
//	#define MSP_ADC_1024 0x04 // ADC OSR=1024	Offset 2
//	#define MSP_ADC_2048 0x06 // ADC OSR=2056	Offset 3
//	#define MSP_ADC_4096 0x08 // ADC OSR=4096	Offset 4
//	#define MSP_ADC_8192 0x08 // ADC OSR=8192	Offset 5
#define MSP_PROM_RD 0xA0 // Prom read command base address

// MS8607 Relative Humidity Commands
#define MSRH_RESET 0xFE // RH reset command
#define MSRH_WRITE_UREG 0xE6 // Write User Register
#define MSRH_READ_UREG 0xE7 // Read User Register
#define MSRH_MEAS_D3_HOLD 0xE5 // Measure RH (D3) Hold Master
#define MSRH_MEAS_D3 0xF5 // Measure RH (D3) No Hold Master
#define MSRH_PROM_RD 0xA0 // Prom read command base address

// *** Need RH User Register Definitions here ***


extern subbus_driver_t sb_i2c_j6;
void i2c_enable(bool value);

#endif
