/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef _VARIANT_DSOCENTRAL_STM32WB5MMG_
#define _VARIANT_DSOCENTRAL_STM32WB5MMG_

// The definitions here needs a STM32WB core >=1.6.6
#define ARDUINO_STM32WB_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32WB_CONFIG_LSECLK             32774
#define STM32WB_CONFIG_HSECLK             32000000
#define STM32WB_CONFIG_SYSOPT             (STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH | STM32WB_SYSTEM_OPTION_SMPS_CURRENT_220mA | STM32WB_SYSTEM_OPTION_USART1_SYSCLK | STM32WB_SYSTEM_OPTION_LSE_MODE_2)
#define STM32WB_CONFIG_PIN_VBUS           STM32WB_GPIO_PIN_PVM1
#define STM32WB_CONFIG_PIN_BUTTON         STM32WB_GPIO_PIN_PH3
#define STM32WB_CONFIG_PIN_VBAT_SWITCH    STM32WB_GPIO_PIN_PD0
#define STM32WB_CONFIG_VBAT_SENSE_CHANNEL STM32WB_ADC_CHANNEL_VBAT
#define STM32WB_CONFIG_VBAT_SENSE_DELAY   10000
#define STM32WB_CONFIG_VBAT_SENSE_PERIOD  STM32WB_ADC_VBAT_PERIOD
#define STM32WB_CONFIG_VBAT_SENSE_SCALE   (1.27 * 3.0)

#define STM32WB_CONFIG_BLE_LSE_SOURCE     BLE_LSE_SOURCE_MOD5MM

#define USBCON

/** Master clock frequency */
#define VARIANT_MCK			  F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "USBAPI.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (37u)
#define NUM_DIGITAL_PINS     (26u)
#define NUM_ANALOG_INPUTS    (11u)
#define NUM_ANALOG_OUTPUTS   (0u)

/*
 * Analog pins
 */
#define PIN_A0               (2ul)
#define PIN_A1               (3ul)
#define PIN_A2               (4ul)
#define PIN_A3               (5ul)

#define PIN_A4               (8ul)
#define PIN_A5               (9ul)
#define PIN_A6               (10ul)
#define PIN_A7               (11ul)

#define PIN_A8               (14ul)
#define PIN_A9               (15ul)
#define PIN_A10              (16ul)
#define PIN_A11              (17ul)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A4;
#define ADC_RESOLUTION		12

// Bus A 
#define BUSA_PWM0               (0u)
#define BUSA_PWM1               (1u)
#define BUSA_AIN0               PIN_A0  // Current A.0
#define BUSA_AIN1               PIN_A1  // Current A.1
#define BUSA_AIN2               PIN_A2  // Current A.0
#define BUSA_AIN3               PIN_A3  // Current A.1

// Bus B
#define BUSB_PWM0               (6u)
#define BUSB_PWM1               (7u)
#define BUSB_AIN0               PIN_A4  // Current B.0
#define BUSB_AIN1               PIN_A5  // Current B.1
#define BUSB_AIN2               PIN_A6  // Temp B.0
#define BUSB_AIN3               PIN_A7  // Temp B.1

// Bus C
#define BUSC_PWM0               (12u)
#define BUSC_PWM1               (13u)
#define BUSC_AIN0               PIN_A8  // Current B.0
#define BUSC_AIN1               PIN_A9  // Current B.1
#define BUSC_AIN2               PIN_A10  // Temp B.0
#define BUSC_AIN3               PIN_A11  // Temp B.1

// Bus D
#define BUSD_SDA         		(18u)
#define BUSD_SCL         		(19u)
#define BUSD_GPIO0          	(20u)
#define BUSD_GPIO1          	(21u)
#define BUSD_GPIO2          	(22u)
#define BUSD_WAKE_INT      		(30u)	 

// Bus E
#define BUSE_SPI_SCK          	(23u)
#define BUSE_SPI_MOSI         	(24u)
#define BUSE_SPI_MISO         	(25u)
#define BUSE_SPI_SS          	(26u)
#define BUSE_SAI_FS          	(27u)
#define BUSE_SAI_SCK          	(28u)
#define BUSE_SAI_SD          	(29u)
#define BUSE_WAKE_INT      		(30u)

// LEDs
#define PIN_LED              	(34u)
#define PIN_LED2             	(35u)
#define PIN_LED3             	(31u)
#define PIN_LED4             	(32u)
#define PIN_LED5             	(33u)

#define LED_BUILTIN          	PIN_LED

#define LED_STATUS         		PIN_LED
#define LED_ERROR          		PIN_LED2
#define LED_RGB_GRN       		PIN_LED5
#define LED_RGB_BLUE       		PIN_LED3
#define LED_RGB_RED       		PIN_LED4


#define PIN_BUTTON           	(36u)
static const uint8_t BUTTON = PIN_BUTTON;

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 1

#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI        BUSE_SPI_MOSI
#define PIN_SPI_MISO        BUSE_SPI_MISO
#define PIN_SPI_SCK         BUSE_SPI_SCK
#define PIN_SPI_SS			BUSE_SPI_SS

static const uint8_t SS	  = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

// Bus D
#define PIN_WIRE_SDA         BUSD_SDA
#define PIN_WIRE_SCL         BUSD_SCL

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * PDM Interfaces
 */
#define PDM_INTERFACES_COUNT 0

#define PWM_INSTANCE_COUNT  4

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  Serial;
extern Uart Serial1;
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

#endif /* _VARIANT_DSOCENTRAL_STM32WB5MMG_ */

