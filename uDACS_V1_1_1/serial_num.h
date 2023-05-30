/** @file serial_num.h
 * This file must define:
 *  CAN_BOARD_ID: The CAN Identifier for the board. This must be unique on a specific CAN Bus
 *  SUBBUS_BOARD_SN: The serial number of this board among boards of the same SUBBUS_BOARD_TYPE
 *  SUBBUS_BUILD_NUM:
 *  SUBBUS_SUBFUNCTION: The board type code as defined in "SYSCON Memory Maps and Subbus Board IDs"
 *  SUBBUS_BOARD_ID: Board Identification number (*not* the CAN_BOARD_ID). Defines the type of board
 *     SUBBUS_SUBFUNCTION may be the same as SUBBUS_BOARD_ID if there is no significant
 *     configuration difference between boards. If different, the SUBBUS_BOARD_ID values
 *     should be documented along with the SUBBUS_BOARD_SN etc. in the board's Specifications document
 *  SUBBUS_BOARD_INSTRUMENT_ID: Number that maps to Instrument name. (not yet used as of 4/18/19)
 *  SUBBUS_BOARD_REV: String encapsulating almost anything here
 */
#ifndef SERIAL_NUM_H_INCLUDED
#define SERIAL_NUM_H_INCLUDED
#include "uDACS_pins.h"

// These parameters are common to all boards built with this code
#define SUBBUS_BOARD_FIRMWARE_REV "V1.8"
#define SUBBUS_BOARD_BUILD_NUM 13
#define HAVE_RTC

/**
 * Build definitions
 * 1: Initial build
 */
#if ! defined(SUBBUS_BOARD_SN)
#error Must define SUBBUS_BOARD_SN in Build Properties
#endif

#if ! defined(SUBBUS_SUBFUNCTION)
// subfunction should be either 9 for Rev A or 14 for Rev B
#error Must define SUBBUS_SUBFUNCTION
#endif

#if SUBBUS_SUBFUNCTION == 9 // uDACS Rev A
  #define SUBBUS_BOARD_BOARD_REV "Rev A"

  #if SUBBUS_BOARD_SN == 1
    #define SUBBUS_BOARD_INSTRUMENT_ID 7
    #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
    #define SUBBUS_BOARD_ID 1
    #define SUBBUS_BOARD_BOARD_TYPE "uDACS A"
  #endif
#elif SUBBUS_SUBFUNCTION == 14 // uDACS Rev B
  #define SUBBUS_SUBFUNCTION_HEX E
  #define SUBBUS_BOARD_BOARD_REV "Rev B"

  #if SUBBUS_BOARD_SN == 1
    #define SUBBUS_BOARD_INSTRUMENT_ID 7
    #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
    #define SUBBUS_BOARD_ID 1
    #define SUBBUS_BOARD_BOARD_TYPE "uDACS A"
    #define SB_FAIL_PIN SPR7
    #define SB_FAIL_PIN2 SPR29
    // #define SB_FAIL_TIMEOUT_SECS 20
  #endif

  #if SUBBUS_BOARD_SN == 2
  #define SUBBUS_BOARD_INSTRUMENT_ID 7
  #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
  #define SUBBUS_BOARD_ID 2
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS B"
  #define uDACS_B
  #endif

  #if SUBBUS_BOARD_SN == 3
  #define SUBBUS_BOARD_INSTRUMENT_ID 7
  #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
  #define SUBBUS_BOARD_ID 2
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS B"
  #define uDACS_B
  #endif

  #if SUBBUS_BOARD_SN == 4
  #define SUBBUS_BOARD_INSTRUMENT_ID 7
  #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
  #define SUBBUS_BOARD_ID 2
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS B"
  #define uDACS_B
  #endif

  #if SUBBUS_BOARD_SN == 5
  #define SUBBUS_BOARD_INSTRUMENT_ID 10
  #define SUBBUS_BOARD_INSTRUMENT "FOCAL"
  #define SUBBUS_BOARD_ID 3
  #define SUBBUS_BOARD_BOARD_TYPE "OE uDACS"
  #define SUBBUS_BOARD_LOCATION "CO2"
//  #define uDACS_B	// Need this out to disable spi_ps.c/.h
  #endif

  #if SUBBUS_BOARD_SN == 6
  #define SUBBUS_BOARD_INSTRUMENT_ID 10
  #define SUBBUS_BOARD_INSTRUMENT "FOCAL"
  #define SUBBUS_BOARD_ID 3
  #define SUBBUS_BOARD_BOARD_TYPE "OE uDACS"
  #define SUBBUS_BOARD_LOCATION "Methane"
  //  #define uDACS_B	// Need this out to disable spi_ps.c/.h
  #endif

  #if SUBBUS_BOARD_SN == 7
  #define SUBBUS_BOARD_INSTRUMENT_ID 10
  #define SUBBUS_BOARD_INSTRUMENT "FOCAL"
  #define SUBBUS_BOARD_ID 3
  #define SUBBUS_BOARD_BOARD_TYPE "OE uDACS"
  #define SUBBUS_BOARD_LOCATION "CO2 Backup"
  //  #define uDACS_B	// Need this out to disable spi_ps.c/.h
  #endif

  #if SUBBUS_BOARD_SN == 8
  #define SUBBUS_BOARD_INSTRUMENT_ID 10
  #define SUBBUS_BOARD_INSTRUMENT "FOCAL"
  #define SUBBUS_BOARD_ID 3
  #define SUBBUS_BOARD_BOARD_TYPE "OE uDACS"
  #define SUBBUS_BOARD_LOCATION "Methane Backup"
  //  #define uDACS_B	// Need this out to disable spi_ps.c/.h
  #endif

  #if SUBBUS_BOARD_SN == 9
  #define SUBBUS_BOARD_INSTRUMENT_ID 10
  #define SUBBUS_BOARD_INSTRUMENT "Test"
  #define SUBBUS_BOARD_ID 3
  #define SUBBUS_BOARD_BOARD_TYPE "Spare uDACS"
  #define SUBBUS_BOARD_LOCATION "Stock"
  //  #define uDACS_B	// Need this out to disable spi_ps.c/.h
  #endif

#else
  #error Unsupported SUBFUNCTION number
#endif

#ifndef SUBBUS_BOARD_LOCATION
  #define SUBBUS_BOARD_LOCATION "DPOPS"
#endif

#ifdef uDACS_B
  #define PS_MOSI PMOD1
  #define PS_MISO PMOD7
  #define PS_SCLK PMOD5
  #define ADC1_CS PMOD8
  #define EEP1_CS PMOD4
  #define ADC2_CS PMOD6
  #define EEP2_CS PMOD2
  #define ADC3_CS SPR_AO
  #define EEP3_CS PMOD3
  #define PPMP_CNTL SPR7
  #define BPMP_CNTL SPR29
  #define PPMPS   UC_SCL
  #define BPMPS   UC_SDA
#endif


#ifndef SUBBUS_SUBFUNCTION_HEX
#define SUBBUS_SUBFUNCTION_HEX SUBBUS_SUBFUNCTION
#endif

#ifdef CAN_BOARD_ID

#define SUBBUS_BOARD_DESC_STR(SN,ID) SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
" S/N:" #SN " CAN ID:" #ID " " SUBBUS_BOARD_LOCATION
#define SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID) SUBBUS_BOARD_DESC_STR(SUBBUS_BOARD_SN,CAN_BOARD_ID)
#define SUBBUS_BOARD_DESC SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID)

#else

#define SUBBUS_BOARD_DESC_STR(SN) SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
" S/N:" #SN " " SUBBUS_BOARD_LOCATION
#define SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN) SUBBUS_BOARD_DESC_STR(SUBBUS_BOARD_SN)
#define SUBBUS_BOARD_DESC SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN)

#endif // CAN_BOARD_ID


#define SUBBUS_BOARD_REV_STR(SN,SF) "V" #SF ":0:" SUBBUS_BOARD_DESC
#define SUBBUS_BOARD_REV_XSTR(SN,SF) SUBBUS_BOARD_REV_STR(SN,SF)
#define SUBBUS_BOARD_REV SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,SUBBUS_SUBFUNCTION_HEX)

#endif
