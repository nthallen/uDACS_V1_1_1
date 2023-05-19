/*
 * Code originally generated from Atmel Start's atmel_start_pins.h.
 * Changes made there by Atmel Start should be migrated into this file.
 * 
 * 
 * 
 */
#ifndef UDACS_DRIVER_INIT_INCLUDED
#define UDACS_DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_ext_irq.h>

#include <hal_spi_m_async.h>

#include <hal_i2c_m_async.h>

#include <hal_i2c_m_async.h>

#include <hal_spi_m_async.h>
#include <hal_usart_async.h>
#include <hal_timer.h>

extern struct spi_m_async_descriptor AD_SPI;

extern struct i2c_m_async_desc PM_I2C;

extern struct i2c_m_async_desc UC_I2C;

extern struct spi_m_async_descriptor SD_SPI;
extern struct usart_async_descriptor USART_CTRL;
extern struct timer_descriptor       TIMER_0;

void AD_SPI_PORT_init(void);
void AD_SPI_CLOCK_init(void);
void AD_SPI_init(void);

void PM_I2C_PORT_init(void);
void PM_I2C_CLOCK_init(void);
void PM_I2C_init(void);

void UC_I2C_PORT_init(void);
void UC_I2C_CLOCK_init(void);
void UC_I2C_init(void);

void SD_SPI_PORT_init(void);
void SD_SPI_CLOCK_init(void);
void SD_SPI_init(void);

void USART_CTRL_PORT_init(void);
void USART_CTRL_CLOCK_init(void);
void USART_CTRL_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // UDACS_DRIVER_INIT_INCLUDED
