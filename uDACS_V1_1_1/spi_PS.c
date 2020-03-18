/************************************************************************/
/* \file spi.c                                                          */
/************************************************************************/
#include <hal_spi_m_async.h>
#include <hal_gpio.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include "atmel_start_pins.h"
#include "spi_PS.h"
#include "subbus.h"

// CCTIT Checksum and Checksum Table with 0x1021 seed
//
unsigned short int CRC16_Computed;   // computed CRC on selected EEPROM

static const unsigned short int gu16Crc16Table[256] = {
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
// static uint8_t PS_spi_read_data[MAX_SPI_XFR_LENGTH];
static uint8_t PS_xfr_Rbuf[MAX_SPI_XFR_LENGTH] = {
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static uint8_t PS_xfr_Wbuf[MAX_SPI_XFR_LENGTH] = {
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static volatile uint8_t PS_SPI_txfr_complete = true;   // async transfer complete?
static bool PS_spi_enabled = PS_SPI_ENABLE_DEFAULT;    // PS SPI comms enabled?

// Declare EEPROM structure - holds EEPROM Contents locally
struct {
	unsigned char       PartNumber[PART_NO_SIZE];
	unsigned char       SerialNumber[SERIAL_NO_SIZE];
	float               Pressure_Range;
	float               Pressure_Min;
	unsigned char       PressureUnit[PRESSURE_UNIT_SIZE];
	unsigned char       PressureRef[PRESSURE_REF_SIZE];
	unsigned char       ADC_Config[ADC_CONFIG_SIZE];
	float               Off_Coeff[DEGREE_POLYNOMIAL];
	float               Spn_Coeff[DEGREE_POLYNOMIAL];
	float               Shp_Coeff[DEGREE_POLYNOMIAL];
	unsigned short int  CheckSum;
} PROM_1, PROM_2, *pPROM; 

// Pressure sensors require two SPI modes:
//    MODE 0 when reading from EEPROM
//    MODE 3 when reading Pressure or Temperature values
//
static enum spi_transfer_mode PS_spi_current_transfer_mode = SPI_MODE_0;

signed short int t_raw;                 // raw temperature reading

/* **************************************************************************
 * ASF4 HAL functions for SPI transfers
 *
 */
static inline void PS_chip_select(uint8_t pin) {
	gpio_set_pin_level(pin, false);
}
static inline void PS_chip_deselect(uint8_t pin) {
	gpio_set_pin_level(pin, true);
}

// SPI Hardware Done Interrupt callback function
static void complete_cb_PS_SPI(const struct spi_m_async_descriptor *const io_descr) {
	PS_SPI_txfr_complete = true;
//	PS_chip_deselect(CS_EEP1);
}

// Kick off SPI Transfer, non-blocking
static void PS_start_spi_transfer(uint8_t pin, uint8_t const *txbuf, int length, enum spi_transfer_mode mode) {
	assert(length <= MAX_SPI_XFR_LENGTH,__FILE__,__LINE__);
	if (PS_spi_current_transfer_mode != mode) {
		spi_m_async_disable(&PS_SPI);
		spi_m_async_set_mode(&PS_SPI, mode);
		PS_spi_current_transfer_mode = mode;
		spi_m_async_enable(&PS_SPI);
	}
	PS_chip_select(pin);
	PS_SPI_txfr_complete = false;
	gpio_set_pin_level(J8P1, true);
	gpio_set_pin_level(J8P1, false);
	spi_m_async_transfer(&PS_SPI, txbuf, PS_xfr_Rbuf, length);
}

void PS_spi_enable(bool value) {
	PS_spi_enabled = value;
}

static void PS_spi_reset(void) {
  if (!sb_PS_spi.initialized) {
    spi_m_async_register_callback(&PS_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_PS_SPI);
    spi_m_async_enable(&PS_SPI);
    sb_PS_spi.initialized = true;
  }
}

/* ***************************************************************************
 * Pressure Sensor driver functions
 *
 */
// EEPROM spans a 9 Bit address space, upper bit is embedded in Read cmd
static inline void EEPROM_read(uint16_t addr) {
	PS_xfr_Wbuf[0] = (addr <= 0xFF ? 0x03 : 0x0B);
	PS_xfr_Wbuf[1] = (uint8_t)addr;
}

// read uint8_t block of length m from EEPROM
void read8Block(uint8_t pin, int addr, uint8_t numBytes) {
	EEPROM_read(addr);
	PS_start_spi_transfer(pin, PS_xfr_Wbuf, numBytes, SPI_MODE_0);
}

// read EEPROM 4 bytes convert to floats
float bytes2float32(int addr) {
	unsigned long dat32 = (PS_xfr_Rbuf[3] << 24) + (PS_xfr_Rbuf[2] << 16)
	                    + (PS_xfr_Rbuf[1] <<  8) + (PS_xfr_Rbuf[0]);
	float val = *((float*)&dat32);
	return(val);
}

/* *****************************************************************************
 * Honeywell RSC series Pressure Sensor Driver State machine - Poll Function.  
 * Main loop = state clock, i.e. evaluate inputs, act, decide next state
 *             each time main calls driver's poll function
 *
 */
static void PS_spi_poll(void){
	if(PS_SPI_txfr_complete == true){
		read8Block(CS_EEP1, 0x00, 4); // Get Pressure Sensor Manufacture Model #
	} 
	else {
		for(uint8_t ii=0; ii<8; ii++) {
			gpio_set_pin_level(J7P1, true);
		}
		gpio_set_pin_level(J7P1, false);
	}
}

static subbus_cache_word_t PS_spi_cache[PS_SPI_HIGH_ADDR-PS_SPI_BASE_ADDR+1] = {
	{ 0, 0, true,  false,  true,  false, false },
	{ 0, 0, true,  false,  true,  false, false },	 
};

subbus_driver_t sb_PS_spi = {
	PS_SPI_BASE_ADDR, PS_SPI_HIGH_ADDR,
	PS_spi_cache,
	PS_spi_reset,
	PS_spi_poll,
	0,
	false
};
