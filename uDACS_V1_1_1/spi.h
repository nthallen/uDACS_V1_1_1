/************************************************************************/
/* \file spi.h                                                          */
/************************************************************************/
#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED
#include "driver_init.h"
#include "subbus.h"

#define MAX_SPI_READ_LENGTH 4
#define ADC_CONVERT_TIMEOUT 500
#define SPI_ENABLE_DEFAULT true
#define SPI_ADC_ENABLED true
#define SPI_DAC_U13_ENABLED true
#define SPI_BASE_ADDR 0x10
#define SPI_HIGH_ADDR 0x13

extern subbus_driver_t sb_spi;
void spi_enable(bool value);

#endif
