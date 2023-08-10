/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define ARDUINO_MAIN
#include "Arduino.h"
#include "wiring_private.h"

#if defined(ARDUINO_MAKEFILE)

#include "STM32WB.h"

#include "stm32wb_lptim.h"
#include "stm32wb_rtc.h"
#include "Wire.h"

#define BME280_I2C_ADDRESS 0x76

uint8_t bme280_data[8];

/* BME280 Calibration Data */
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

void bme280_read_calibration_data()
{
    uint8_t data[26];

    Wire.beginTransmission(BME280_I2C_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission(false);
    Wire.requestFrom(BME280_I2C_ADDRESS, 26);
    Wire.read(&data[0], 26);

    dig_T1 = (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[0]);
    dig_T2 = (int16_t)(((uint16_t)data[3] << 8) | (uint16_t)data[2]);
    dig_T3 = (int16_t)(((uint16_t)data[5] << 8) | (uint16_t)data[4]);
    dig_P1 = (uint16_t)(((uint16_t)data[7] << 8) | (uint16_t)data[6]);
    dig_P2 = (int16_t)(((uint16_t)data[9] << 8) | (uint16_t)data[8]);
    dig_P3 = (int16_t)(((uint16_t)data[11] << 8) | (uint16_t)data[10]);
    dig_P4 = (int16_t)(((uint16_t)data[13] << 8) | (uint16_t)data[12]);
    dig_P5 = (int16_t)(((uint16_t)data[15] << 8) | (uint16_t)data[14]);
    dig_P6 = (int16_t)(((uint16_t)data[17] << 8) | (uint16_t)data[16]);
    dig_P7 = (int16_t)(((uint16_t)data[19] << 8) | (uint16_t)data[18]);
    dig_P8 = (int16_t)(((uint16_t)data[21] << 8) | (uint16_t)data[20]);
    dig_P9 = (int16_t)(((uint16_t)data[23] << 8) | (uint16_t)data[22]);
    dig_H1 = data[25];

    Wire.beginTransmission(BME280_I2C_ADDRESS);
    Wire.write(0xe1);
    Wire.endTransmission(false);
    Wire.requestFrom(BME280_I2C_ADDRESS, 7);
    Wire.read(&data[0], 7);
    
    dig_H2 = (int16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[0]);
    dig_H3 = data[2];
    dig_H4 = (int16_t)((int8_t)data[3] * 16) | (int16_t)(data[4] & 0x0f);
    dig_H5 = (int16_t)((int8_t)data[5] * 16) | (int16_t)(data[4] >> 4);
    dig_H6 = (int8_t)data[6];
}

void bme280_write_config()
{
    Wire.beginTransmission(BME280_I2C_ADDRESS);
    Wire.write(0xf2);
    Wire.write(0x01);
    Wire.endTransmission();

    Wire.beginTransmission(BME280_I2C_ADDRESS);
    Wire.write(0xf4);
    Wire.write(0x24);
    Wire.endTransmission();
}

float bme280_compensate_temperature(uint32_t temperature_uncomp, int32_t &t_fine)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((temperature_uncomp / 8) - ((int32_t)dig_T1 * 2));
    var1 = (var1 * ((int32_t)dig_T2)) / 2048;
    var2 = (int32_t)((temperature_uncomp / 16) - ((int32_t)dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)dig_T3)) / 16384;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) / 256;

    if (temperature < temperature_min) {
        temperature = temperature_min;
    } else if (temperature > temperature_max) {
        temperature = temperature_max;
    }

    return (float)temperature / 100.0f;
}

float bme280_compensate_pressure(uint32_t pressure_uncomp, int32_t t_fine)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) * 131072);
    var2 = var2 + (((int64_t)dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)dig_P3) / 256) + ((var1 * ((int64_t)dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)dig_P1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0) {
        var4 = 1048576 - pressure_uncomp;
        var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
        var1 = (((int64_t)dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)dig_P7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min) {
            pressure = pressure_min;
        } else if (pressure > pressure_max) {
            pressure = pressure_max;
        }
    } else {
        pressure = pressure_min;
    }

    return (float)pressure / 100.0f;
}

float bme280_compensate_humidity(uint32_t humidity_uncomp, int32_t t_fine)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(humidity_uncomp * 16384);
    var3 = (int32_t)(((int32_t)dig_H4) * 1048576);
    var4 = ((int32_t)dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max) {
        humidity = humidity_max;
    }

    return (float)humidity / 1024.0f;
}

volatile uint32_t clock_e = 0;
volatile bool event = false;

uint32_t clock_s = 0;

uint32_t count = 0;

uint32_t clock_table[128];
float    temp_table[128];
int32_t  vrefint_data_table[128];
int32_t  tsense_data_table[128];
int32_t  tsense_table[128];

void myCallback()
{
    clock_e = stm32wb_lptim_timeout_clock();
    
    event = true;
}

void setup(void) {
    Serial.begin(9600);

    while (!Serial) { }

    Wire.begin();

    bme280_read_calibration_data();
    bme280_write_config();

    Serial.println();
  
    stm32wb_lptim_timeout_enable();

    pinMode(10, INPUT_PULLUP);

    attachInterrupt(10, myCallback, RISING, 0);

    while (!event) {}

    event = false;

    clock_s = clock_e = 0;

    count = 0;
}

void loop(void) {
    uint32_t i, clock;
    int32_t t_fine, vrefint_data, tsense_data, vref,tsense;
    float temp;
    char buf[1024];
    
    __WFE();
  
    if (event)
    {
        event = false;

        clock = clock_e - clock_s;

        clock_s = clock_e;
        
        vrefint_data = stm32wb_adc_convert(STM32WB_ADC_CHANNEL_VREFINT, STM32WB_ADC_VREFINT_PERIOD);
        tsense_data = stm32wb_adc_convert(STM32WB_ADC_CHANNEL_TSENSE, STM32WB_ADC_TSENSE_PERIOD);

        Wire.beginTransmission(BME280_I2C_ADDRESS);
        Wire.write(0xf4);
        Wire.write(0x25);
        Wire.endTransmission();
        
        delay(10);
        
        Wire.beginTransmission(BME280_I2C_ADDRESS);
        Wire.write(0xf7);
        Wire.endTransmission(false);
        Wire.requestFrom(BME280_I2C_ADDRESS, 8);
        Wire.read(&bme280_data[0], 8);
        
        temp = bme280_compensate_temperature((((uint32_t)bme280_data[3] << 12) | ((uint32_t)bme280_data[4] << 4) | ((uint32_t)bme280_data[5] >> 4)), t_fine);

        vref = ((int32_t)(STM32WB_ADC_VREFINT_VREF * 4096) * STM32WB_ADC_VREFINT_CAL) / vrefint_data;
        
        tsense = (tsense_data * vref) / (int32_t)(STM32WB_ADC_TSENSE_CAL_VREF * 4096);
        
        i = (count & 127);

        clock_table[i] = clock;
        temp_table[i] = temp;
        tsense_table[i] = tsense;
        vrefint_data_table[i] = vrefint_data;
        tsense_data_table[i] = tsense_data;

        if (count >= 128)
        {
            clock = 0;
            temp = 0;
            tsense = 0;
            vrefint_data = 0;
            tsense_data = 0;

            for (i = 0; i < 128; i++)
            {
                clock += clock_table[i];
                temp += temp_table[i];
                tsense += tsense_table[i];
                vrefint_data += vrefint_data_table[i];
                tsense_data += tsense_data_table[i];
            }

            sprintf(buf, "%d.%03d, %d.%03d,, %d, %d, %d,, %d\r\n",
                    clock / 128, (((clock & 127) * 1000) / 128),
                    ((int)((temp * 100) / 128) / 100), ((((int)((temp * 100) / 128) % 100) * 1000) / 100),
                    ((clock + 2) / 4) * 8,
                    (int)((temp / 128.0) * 100),
                    (int)(tsense / 128),
                    ((((int32_t)(STM32WB_ADC_TSENSE_CAL2_TEMP * 100 - STM32WB_ADC_TSENSE_CAL1_TEMP * 100) * ((tsense / 128) - STM32WB_ADC_TSENSE_CAL1))
                      / (STM32WB_ADC_TSENSE_CAL2 - STM32WB_ADC_TSENSE_CAL1))
                     + (int32_t)(STM32WB_ADC_TSENSE_CAL1_TEMP * 100)));

            Serial.write(buf, strlen(buf));
        }
        
        count++;
    }
}

#endif

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() {
}

#if defined(USBCON)

void initUSB() __attribute__((weak));
void initUSB() {
  USBDevice.begin();
  USBDevice.start();
}

#endif

void (*g_serialEventRun)(void) = NULL;

/*
 * \brief Main entry point of Arduino application
 */
int main( void ) {
  init();
  initVariant();

#if defined(USBCON)
  initUSB();
#endif
  
  setup();

  for (;;)
  {
    loop();
    if (g_serialEventRun) (*g_serialEventRun)();
  }

  return 0;
}
