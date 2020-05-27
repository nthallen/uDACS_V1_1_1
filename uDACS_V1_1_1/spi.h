/************************************************************************/
/* \file spi.h                                                          */
/************************************************************************/
#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED
#include <hal_spi_m_async.h>
#include "subbus.h"

#define MAX_SPI_READ_LENGTH 32
#define ADC_CONVERT_TIMEOUT 500
#define SPI_ENABLE_DEFAULT true
#define SPI_AD7770_ENABLED true
#define SPI_DAC_U13_ENABLED true
#define SPI_BASE_ADDR 0x10
#define POLL_COUNT_ADDR 0x25
#define GEN_ERRS_ADDR 0x26
#define REG_QUERY_ADDR 0x27
#define SPI_HIGH_ADDR 0x27
#define ADC_LSW_OFFSET(x) (5+2*(x))
#define ADC_MSB_OFFSET(x) (6+2*(x))

extern subbus_driver_t sb_spi;
void spi_enable(bool value);
struct spi_m_async_descriptor AD_SPI;
void AD_SPI_init(void);

#endif
