/*
 * Copyright (c) 2021 Thomas Roell.  All rights reserved.
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

#include "Arduino.h"
#include "wiring_private.h"

#define PWM_INSTANCE_TIM1      0
#define PWM_INSTANCE_TIM2      1
#define PWM_INSTANCE_TIM16     2
#define PWM_INSTANCE_TIM17     3

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..7 - Digital pins
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA0),  STM32WB_GPIO_PIN_PA0_TIM2_CH1,   0,                                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA1),  STM32WB_GPIO_PIN_PA1_TIM2_CH2,   0,                                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB10), STM32WB_GPIO_PIN_PB10_TIM2_CH3,  0,                                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB11), STM32WB_GPIO_PIN_PB11_TIM2_CH4,  0,                                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_4,    ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD14), STM32WB_GPIO_PIN_PD14_TIM1_CH1,  0,                                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD15), STM32WB_GPIO_PIN_PD15_TIM1_CH2,  0,                                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    { GPIOE, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PE0),  STM32WB_GPIO_PIN_PE0_TIM16_CH1,  0,                                  PWM_INSTANCE_TIM16,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOE, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PE1),  STM32WB_GPIO_PIN_PE1_TIM17_CH1,  0,                                  PWM_INSTANCE_TIM17,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },

    // 8..9 - I2C pins (SDA,SCL)
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB7),  STM32WB_GPIO_PIN_PB7,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB8),  STM32WB_GPIO_PIN_PB8,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 10..15 - Analog pins
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC0),  STM32WB_GPIO_PIN_PC0,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_1,    ADC_CHANNEL_1    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC1),  STM32WB_GPIO_PIN_PC1,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_2,    ADC_CHANNEL_2    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC2),  STM32WB_GPIO_PIN_PC2,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_2,    ADC_CHANNEL_3   },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC3),  STM32WB_GPIO_PIN_PC3,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC4),  STM32WB_GPIO_PIN_PC4,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_13   },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC5),  STM32WB_GPIO_PIN_PC5,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_14   },

    // 16..19 - LED/BUTTON
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD0),  STM32WB_GPIO_PIN_PD0,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC6),  STM32WB_GPIO_PIN_PC6,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD1),  STM32WB_GPIO_PIN_PD1,            0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB14),  STM32WB_GPIO_PIN_PB14,          0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    STM32WB_TIM_INSTANCE_TIM1,
    STM32WB_TIM_INSTANCE_TIM2,
    STM32WB_TIM_INSTANCE_TIM16,
    STM32WB_TIM_INSTANCE_TIM17,
};

extern const stm32wb_uart_params_t g_Serial1Params = {
    STM32WB_UART_INSTANCE_LPUART1,
    STM32WB_UART_IRQ_PRIORITY,
    STM32WB_DMA_CHANNEL_NONE,
    STM32WB_DMA_CHANNEL_NONE,
    NULL,
    0,
    {
        STM32WB_GPIO_PIN_PB10_LPUART1_RX,
        STM32WB_GPIO_PIN_PB11_LPUART1_TX,
        STM32WB_GPIO_PIN_PB12_LPUART1_RTS_DE,
        STM32WB_GPIO_PIN_PB13_LPUART1_CTS,
        STM32WB_GPIO_PIN_NONE,
    },
};

extern const stm32wb_spi_params_t g_SPIParams = {
    STM32WB_SPI_INSTANCE_SPI2,
    STM32WB_SPI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH3_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI2_RX),
    (STM32WB_DMA_CHANNEL_DMA1_CH4_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI2_TX),
    {
        STM32WB_GPIO_PIN_PB15_SPI2_MOSI,
        STM32WB_GPIO_PIN_PC2_SPI2_MISO,
        STM32WB_GPIO_PIN_PD1_SPI2_SCK,
    },
};

extern const stm32wb_uart_params_t g_SPI1Params = {
    STM32WB_UART_INSTANCE_USART1,
    STM32WB_UART_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH1_INDEX | STM32WB_DMA_CHANNEL_SELECT_USART1_RX),
    (STM32WB_DMA_CHANNEL_DMA1_CH2_INDEX | STM32WB_DMA_CHANNEL_SELECT_USART1_TX),
    NULL,
    0,
    {
        STM32WB_GPIO_PIN_PB7_USART1_RX,
        STM32WB_GPIO_PIN_PB6_USART1_TX,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_PB5_USART1_CK,
    },
};

extern const stm32wb_i2c_params_t g_WireParams = {
    STM32WB_I2C_INSTANCE_I2C3,
    STM32WB_I2C_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH5_INDEX | STM32WB_DMA_CHANNEL_SELECT_I2C3_RX),
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PC0_I2C3_SCL,
        STM32WB_GPIO_PIN_PC1_I2C3_SDA,
    },
};

extern const stm32wb_i2c_params_t g_Wire1Params = {
    STM32WB_I2C_INSTANCE_I2C1,
    STM32WB_I2C_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH6_INDEX | STM32WB_DMA_CHANNEL_SELECT_I2C1_RX),
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PB8_I2C1_SCL,
        STM32WB_GPIO_PIN_PA10_I2C1_SDA,
    },
};

static const stm32wb_spi_params_t g_SPI2Params = {
    STM32WB_SPI_INSTANCE_SPI1,
    STM32WB_SPI_IRQ_PRIORITY,
    STM32WB_DMA_CHANNEL_NONE,
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PA7_SPI1_MOSI,
        STM32WB_GPIO_PIN_PA6_SPI1_MISO,
        STM32WB_GPIO_PIN_PA5_SPI1_SCK,
    },
};

extern const stm32wb_sai_params_t g_PDMParams = {
    STM32WB_SAI_INSTANCE_SAI1,
    STM32WB_SAI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH1_INDEX | STM32WB_DMA_CHANNEL_SELECT_SAI1_A),
    (STM32WB_SAI_CONFIG_BLOCK_A | STM32WB_SAI_CONFIG_PDM_DI1),
    {
        STM32WB_GPIO_PIN_PA8_SAI1_PDM_CK2,
        STM32WB_GPIO_PIN_PC3_SAI1_PDM_DI1,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
    },
};

static const stm32wb_sdspi_params_t g_SDSPI2Params =
{
    {
        STM32WB_GPIO_PIN_PB3,
    },
};

static stm32wb_spi_t g_SPI2;

static const stm32wb_sfsqi_params_t g_SFSQIParams =
{
    STM32WB_QUADSPI_IRQ_PRIORITY,
    {
        STM32WB_GPIO_PIN_PA3_QUADSPI_CLK,
        STM32WB_GPIO_PIN_PD3_QUADSPI_NCS,
        STM32WB_GPIO_PIN_PD4_QUADSPI_IO0,
        STM32WB_GPIO_PIN_PD5_QUADSPI_IO1,
        STM32WB_GPIO_PIN_PD6_QUADSPI_IO2,
        STM32WB_GPIO_PIN_PD7_QUADSPI_IO3,
    },
};

void initVariant()
{
    //stm32wb_spi_create(&g_SPI2, &g_SPI2Params);

    //stm32wb_sdspi_initialize(&g_SPI2, &g_SDSPI2Params);

    stm32wb_sfsqi_initialize(&g_SFSQIParams);
}
