/*
 * Code originally generated from Atmel Start's atmel_start_pins.h.
 * Changes made there by Atmel Start should be migrated into this file.
 */
#ifndef UDACS_PINS_H_INCLUDED
#define UDACS_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions
// ATSAMD21G18A-A
//          AT - Atmel
//              SMAD - General purpose MicroController
//                21 - 32-bit Cortex-M0+,
//                 G - 48 pin chip with
//                18 - 256K Flash 32K SRAM,
//                 A - Default Variant (vs. Floating point)
//                 A - TQFP Package

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

//      Signal                 Port Bit
#define AD_MOSI    GPIO(GPIO_PORTA, 8)
#define AD_MISO    GPIO(GPIO_PORTA, 10)
#define AD_SCLK    GPIO(GPIO_PORTA, 9)
#define ADC_CS     GPIO(GPIO_PORTA, 11)
#define DAC_CS     GPIO(GPIO_PORTB, 9)
#define ADC_ALERT  GPIO(GPIO_PORTA, 1)
#define DRDY       GPIO(GPIO_PORTB, 3)

#define SD_MOSI    GPIO(GPIO_PORTA, 12)
#define SD_MISO    GPIO(GPIO_PORTA, 15)
#define SD_SCLK    GPIO(GPIO_PORTA, 13)
#define SD_CS      GPIO(GPIO_PORTA, 14)
#define SD         GPIO(GPIO_PORTA, 27)  // Monitors if card in Slot ??

#define PMOD1      GPIO(GPIO_PORTA, 19)
#define PMOD7      GPIO(GPIO_PORTA, 16)
#define PMOD5      GPIO(GPIO_PORTA, 17)
#define PMOD8      GPIO(GPIO_PORTB, 10)
#define PMOD4      GPIO(GPIO_PORTA, 28)
#define PMOD6      GPIO(GPIO_PORTB, 11)
#define PMOD2      GPIO(GPIO_PORTA, 21)

#define UC_SDA     GPIO(GPIO_PORTA, 22)
#define UC_SCL     GPIO(GPIO_PORTA, 23)

#define UART_TX    GPIO(GPIO_PORTB, 22)
#define UART_RX    GPIO(GPIO_PORTB, 23)

#define PMOD3      GPIO(GPIO_PORTA, 18)  // Spare on PMOD Conn
#define START      GPIO(GPIO_PORTB, 2)   // Unused (?) input ADC
#define SPR7       GPIO(GPIO_PORTB, 8)   // Spare on J7 pin 1
#define SPR29      GPIO(GPIO_PORTA, 20)  // Spare on J8 pin 1

// 27 of 38 definable PINS defined

#ifdef uDACS_B
#define PS_MOSI PMOD1
#define PS_MISO PMOD7
#define PS_SCLK PMOD5
#define CS_ADC1 PMOD8
#define CS_EEP1 PMOD4
#define CS_ADC2 PMOD6
#define CS_EEP2 PMOD2
#endif

#endif // UDACS_PINS_H_INCLUDED
