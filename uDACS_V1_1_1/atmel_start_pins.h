/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define AD_MOSI   GPIO(GPIO_PORTA, 8)
#define AD_SCLK   GPIO(GPIO_PORTA, 9)
#define AD_MISO   GPIO(GPIO_PORTA, 10)
#define ADC_CS    GPIO(GPIO_PORTA, 11)
#define DAC_CS    GPIO(GPIO_PORTB, 9)
#define START     GPIO(GPIO_PORTB, 2)
#define DRDY      GPIO(GPIO_PORTB, 3)
#define ADC_Alert GPIO(GPIO_PORTA, 1)

#define SD_MOSI   GPIO(GPIO_PORTA, 12)
#define SD_MSCLK  GPIO(GPIO_PORTA, 13)
#define SD_MISO   GPIO(GPIO_PORTA, 15)
#define SD_CS     GPIO(GPIO_PORTA, 14)
#define SD_Sns    GPIO(GPIO_PORTA, 27)

#define PS_MOSI   GPIO(GPIO_PORTA, 19)
#define PS_SCLK   GPIO(GPIO_PORTA, 17)
#define PS_MISO   GPIO(GPIO_PORTA, 16)
#define EEP1_CS   GPIO(GPIO_PORTA, 28)
#define EEP2_CS   GPIO(GPIO_PORTA, 21)
#define ADC1_CS   GPIO(GPIO_PORTB, 10)
#define ADC2_CS   GPIO(GPIO_PORTB, 11)

#define Pmp_CNTL_1 GPIO(GPIO_PORTB, 8)
#define Pmp_CNTL_2 GPIO(GPIO_PORTA, 20)
#define Pmp_STAT_1 GPIO(GPIO_PORTA, 22)
#define Pmp_STAT_2 GPIO(GPIO_PORTA, 23)

#define PB22 GPIO(GPIO_PORTB, 22)
#define PB23 GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
