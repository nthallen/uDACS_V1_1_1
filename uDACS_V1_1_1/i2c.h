/************************************************************************/
/* 2:16 PM 4/24/2023	file i2c.h 
	
	uDACS I2C interfaces 
        UC_I2C : J4
        PM_I2C : J6
        

 ************************************************************************/
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED
#include <hal_i2c_m_async.h>
#include "subbus.h"

#define I2C_J4_BASE_ADDR 0x80
#define I2C_J4_STATUS_OFFSET 0x00
#define I2C_J4_HIGH_ADDR 0x8F

#define I2C_J4_ENABLE_DEFAULT true	// Default true. SUBBUS can change
#define I2C_MS5607_ENABLED true	// Default true
#define I2C_P4_ENABLED false	// Default false

#define I2C_J4_MAX_READ_LENGTH 3

#define ADC_LSW_OFFSET(x) (5+2*(x))
#define ADC_MSB_OFFSET(x) (6+2*(x))

#define MSP_I2C_ADDR 0x77
#define MSRH_I2C_ADDR 0x40

// MS5607 Pressure Commands
#define MSP_RESET 0x1E // ADC reset command
#define MSP_ADC_READ 0x00 // ADC read command
#define MSP_CONV_D1 0x40 // ADC D1 conversion command
#define MSP_CONV_D2 0x50 // ADC D2 conversion command
#define MSP_OSR_OFFS 0x04 // ADC OSR offset (Default = 4096)
//	#define MSP_ADC_256 0x00 // ADC OSR=256	Offset 0
//	#define MSP_ADC_512 0x02 // ADC OSR=512	Offset 1
//	#define MSP_ADC_1024 0x04 // ADC OSR=1024	Offset 2
//	#define MSP_ADC_2048 0x06 // ADC OSR=2056	Offset 3
//	#define MSP_ADC_4096 0x08 // ADC OSR=4096	Offset 4
//	#define MSP_ADC_8192 0x0A // ADC OSR=8192	Offset 5
#define MSP_PROM_RD 0xA0 // Prom read command base address

// MS5607 Relative Humidity Commands
#define MSRH_RESET 0xFE // RH reset command
#define MSRH_WRITE_UREG 0xE6 // Write User Register
#define MSRH_READ_UREG 0xE7 // Read User Register
#define MSRH_MEAS_D3_HOLD 0xE5 // Measure RH (D3) Hold Master
#define MSRH_MEAS_D3 0xF5 // Measure RH (D3) No Hold Master
#define MSRH_PROM_RD 0xA0 // Prom read command base address

// *** Need RH User Register Definitions here ***


extern subbus_driver_t sb_i2c_j4;
void i2c_j4_enable(bool value);
extern struct i2c_m_async_desc UC_I2C;
void UC_I2C_PORT_init(void);
void UC_I2C_CLOCK_init(void);
void UC_I2C_init(void);

#endif
