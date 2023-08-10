/*
 * Copyright (c) 2016-2023 Thomas Roell.  All rights reserved.
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

#include "armv7m.h"
#include "stm32wbxx.h"

#include "stm32wb_adc.h"
#include "stm32wb_system.h"

#define ADC_SAMPLE_TIME_2_5         0
#define ADC_SAMPLE_TIME_6_5         1
#define ADC_SAMPLE_TIME_12_5        2
#define ADC_SAMPLE_TIME_24_5        3
#define ADC_SAMPLE_TIME_47_5        4
#define ADC_SAMPLE_TIME_92_5        5
#define ADC_SAMPLE_TIME_247_5       6
#define ADC_SAMPLE_TIME_640_5       7

#define ADC_CCR_CKMODE_SYSCLK       0
#define ADC_CCR_CKMODE_HCLK_DIV_1   (ADC_CCR_CKMODE_0)
#define ADC_CCR_CKMODE_HCLK_DIV_2   (ADC_CCR_CKMODE_1)
#define ADC_CCR_CKMODE_HCLK_DIV_4   (ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1)

#define ADC_CCR_PRESC_DIV_1         0
#define ADC_CCR_PRESC_DIV_2         (ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_4         (ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_6         (ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_8         (ADC_CCR_PRESC_2)
#define ADC_CCR_PRESC_DIV_10        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_12        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_16        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_32        (ADC_CCR_PRESC_3)
#define ADC_CCR_PRESC_DIV_64        (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_128       (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_256       (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)

#define STM32WB_ADC_CALFACT_UNDEFINED 0xffffffff

#define STM32WB_ADC_SAMPLE_TIME(_ticks, _adcclk) ((uint32_t)((((double)(_ticks) * (double)1e9) / (double)(_adcclk)) + 0.5))

static const uint32_t stm32wb_adc_threshold_2[8] = {
    STM32WB_ADC_SAMPLE_TIME(2.5,   2000000),
    STM32WB_ADC_SAMPLE_TIME(6.5,   2000000),
    STM32WB_ADC_SAMPLE_TIME(12.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(24.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(47.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(92.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(247.5, 2000000),
    STM32WB_ADC_SAMPLE_TIME(640.5, 2000000),
};

static const uint32_t stm32wb_adc_threshold_16[8] = {
    STM32WB_ADC_SAMPLE_TIME(2.5,   16000000),
    STM32WB_ADC_SAMPLE_TIME(6.5,   16000000),
    STM32WB_ADC_SAMPLE_TIME(12.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(24.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(47.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(92.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(247.5, 16000000),
    STM32WB_ADC_SAMPLE_TIME(640.5, 16000000),
};

static const uint32_t stm32wb_adc_threshold_32[8] = {
    STM32WB_ADC_SAMPLE_TIME(2.5,   32000000),
    STM32WB_ADC_SAMPLE_TIME(6.5,   32000000),
    STM32WB_ADC_SAMPLE_TIME(12.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(24.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(47.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(92.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(247.5, 32000000),
    STM32WB_ADC_SAMPLE_TIME(640.5, 32000000),
};

#define STM32WB_ADC_CALFACT_UNDEFINED 0xffffffff

typedef struct _stm32wb_adc_device_t {
    uint32_t                  hclk;
    uint32_t                  calfact;
    uint32_t                  smp;
    uint32_t                  period;
    const uint32_t            *threshold;
} stm32wb_adc_device_t;

static stm32wb_adc_device_t stm32wb_adc_device;

static int32_t __svc_stm32wb_adc_convert(uint32_t channel, uint32_t period)
{
    uint32_t hclk, ckmode, smp, data;
    
    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_ADC);

    hclk = stm32wb_system_hclk();

    if (stm32wb_adc_device.hclk != hclk)
    {
        if (hclk == 2000000)
        {
            ckmode = ADC_CCR_CKMODE_HCLK_DIV_1;
            
            stm32wb_adc_device.threshold = stm32wb_adc_threshold_2;
        }
        else if (hclk == 1600000)
        {
            ckmode = ADC_CCR_CKMODE_HCLK_DIV_1;
            
            stm32wb_adc_device.threshold = stm32wb_adc_threshold_16;
        }
        else
        {
            /* Use a 32MHz ADCCLK, so that a 40uS sampling time can be achieved.
             */
            
            if (hclk == 32000000) { ckmode = ADC_CCR_CKMODE_HCLK_DIV_1; }
            else                  { ckmode = ADC_CCR_CKMODE_HCLK_DIV_2; }
            
            stm32wb_adc_device.threshold = stm32wb_adc_threshold_32;
        }

        ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~(ADC_CCR_CKMODE | ADC_CCR_PRESC)) | ckmode;
    
        stm32wb_adc_device.hclk = hclk;
        stm32wb_adc_device.calfact = STM32WB_ADC_CALFACT_UNDEFINED;
        stm32wb_adc_device.smp = 0;
        stm32wb_adc_device.period = 0;
    }

    ADC1->CR = 0; // reset ADC_CR_DEEPPWD

    ADC1->CR = ADC_CR_ADVREGEN;
    
    armv7m_core_udelay(20);

    if (stm32wb_adc_device.calfact == STM32WB_ADC_CALFACT_UNDEFINED)
    {
	ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
	
	while (ADC1->CR & ADC_CR_ADCAL)
	{
	}
	
	stm32wb_adc_device.calfact = ADC1->CALFACT;
    }
    
    ADC1->ISR = ADC_ISR_ADRDY;

    do
    {
	ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADEN;
    }
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    if (channel == STM32WB_ADC_CHANNEL_VREFINT)
    {
        ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
    }
    else
    {
        if (channel >= STM32WB_ADC_CHANNEL_TSENSE)
        {
            if (channel == STM32WB_ADC_CHANNEL_TSENSE)
            {
                ADC1_COMMON->CCR |= ADC_CCR_TSEN;
                
                armv7m_core_udelay(120);
            }
            else
            {
                ADC1_COMMON->CCR |= ADC_CCR_VBATEN;
            }
        }
    }

    if (stm32wb_adc_device.period != period)
    {
        stm32wb_adc_device.period = period;

        for (smp = 0; smp < 7; smp++)
        {
            if (period <= stm32wb_adc_device.threshold[smp])
            {
                break;
            }
        }

        stm32wb_adc_device.smp = smp;
    }
    
    /* Silicon ERRATA 2.4.4. Wrong ADC conversion results when delay between
     * calibration and first conversion or between 2 consecutive conversions is too long. 
     */

    ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_JQDIS;
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = (channel << 6);
    ADC1->CALFACT = stm32wb_adc_device.calfact;
	
    ADC1->ISR = ADC_ISR_EOC;

    ADC1->CR |= ADC_CR_ADSTART;
    
    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
    }
    
    data = ADC1->DR & ADC_DR_RDATA;

    if (channel < 10)
    {
        ADC1->SMPR1 = stm32wb_adc_device.smp << (channel * 3);
    }
    else
    {
        ADC1->SMPR2 = stm32wb_adc_device.smp << ((channel * 3) - 30);
    }
    
    ADC1->ISR = ADC_ISR_EOC;

    ADC1->CR |= ADC_CR_ADSTART;
    
    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
    }
    
    data = ADC1->DR & ADC_DR_RDATA;
    
    ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADDIS;

    while (ADC1->CR & ADC_CR_ADEN)
    {
    }
    
    ADC1->CR = 0; // reset ADC_CR_ADVREGEN

    ADC1->CR = ADC_CR_DEEPPWD;
    
    ADC1_COMMON->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN | ADC_CCR_TSEN);
    ADC1_COMMON->CCR;

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_ADC);

    return data;
}

uint32_t stm32wb_adc_convert(uint32_t channel, uint32_t period)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_2((uint32_t)&__svc_stm32wb_adc_convert, channel, period);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_adc_convert(channel, period);
    }

    return 0;
}


