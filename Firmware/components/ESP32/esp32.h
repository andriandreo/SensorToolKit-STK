/******************************************************************************  
 * @file       esp32.h
 * @brief      (Generic) ESP32 board implementation code needed for port file.
 *             Header file.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V1.0.0
 * @date       October 2024
 * 
 * @par        Revision History:
 *              * Added support for other ESP32 boards.
 *              * Checked for redundant includes.
 * 
 * @todo       * Re-implement SPI transactions to use tx/rx data instead of buffers.
 *             * Analyse the need for SPI lazy init.
 *             * Implement Wi-Fi, BLE and MQTT functionality.
 * 
 * 
 * It is intended to include all the declarations, definitions, macros and
 * functions needed for the port file of the ESP32 board family as suggested
 * by Analog Devices, Inc. in the AD594x porting guide.
 *              
 * ****************************************************************************
 * This software is built in collaboration with JLM Innovation GmbH.
 * 
 * It is intended for use in conjunction of the ESP-IDF framework.
 * ----------------------------------------------------------------------------
 * @license: http://www.apache.org/licenses/LICENSE-2.0 
 *           (accessed on January 31, 2025)
 * 
 * Copyright (c) 2025 Andrés Alberto
 * 
 * Based on ESP-IDF examples (Apache 2.0 Licensed) from:
 * - `wifi/getting-started/station`
 * - `protocols/mqtt5`
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
******************************************************************************/

// Include guard
#ifndef ESP32_H
#define ESP32_H

// Include dependencies
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Needed for delay functions with `vTaskDelay`
#include "esp_task_wdt.h"  // Needed for watchdog timer functions
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

/* Definition of ESP32 SPI PINS */
#ifdef CONFIG_IDF_TARGET_ESP32
#  ifdef CONFIG_USE_SPI1_PINS
#    define AD594x_HOST    SPI1_HOST
// Use default pins, same as the flash chip.
#    define PIN_NUM_MISO 7
#    define PIN_NUM_MOSI 8
#    define PIN_NUM_CLK  6
#  else
#    define AD594x_HOST    HSPI_HOST
#    define PIN_NUM_MISO 18
#    define PIN_NUM_MOSI 23
#    define PIN_NUM_CLK  19
#  endif

#  define PIN_NUM_CS   13
#elif defined CONFIG_IDF_TARGET_ESP32S2
#  define AD594x_HOST    SPI2_HOST

#  define PIN_NUM_MISO 37
#  define PIN_NUM_MOSI 35
#  define PIN_NUM_CLK  36
#  define PIN_NUM_CS   34
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C2)
#  define AD594x_HOST    SPI2_HOST

#  if defined(CONFIG_USE_XIAO_BOARD) && defined(CONFIG_IDF_TARGET_ESP32C3)
#    define PIN_NUM_MISO 9
#    define PIN_NUM_MOSI 10
#    define PIN_NUM_CLK  8
#    define PIN_NUM_CS   20
#  elif defined(CONFIG_USE_XIAO_BOARD) && defined(CONFIG_IDF_TARGET_ESP32C6)
#    define PIN_NUM_MISO 20
#    define PIN_NUM_MOSI 18
#    define PIN_NUM_CLK  19
#    define PIN_NUM_CS   17
#  else
#    define PIN_NUM_MISO 2
#    define PIN_NUM_MOSI 7
#    define PIN_NUM_CLK  6
#    define PIN_NUM_CS   10
#  endif
#elif CONFIG_IDF_TARGET_ESP32S3
#  define AD594x_HOST    SPI2_HOST

#  ifdef CONFIG_USE_XIAO_BOARD
#    define PIN_NUM_MISO 8
#    define PIN_NUM_MOSI 9
#    define PIN_NUM_CLK  7
#    define PIN_NUM_CS   44
#  else
#    define PIN_NUM_MISO 13
#    define PIN_NUM_MOSI 11
#    define PIN_NUM_CLK  12
#    define PIN_NUM_CS   10
#  endif

#elif CONFIG_IDF_TARGET_ESP32H2
#  define AD594x_HOST    SPI2_HOST

#  define PIN_NUM_MISO 0
#  define PIN_NUM_MOSI 5
#  define PIN_NUM_CLK  4
#  define PIN_NUM_CS   1
#endif

/* Definition for AD5940 Pins */
#define AD5940_SCK_PIN    PIN_NUM_CLK
#define AD5940_MISO_PIN   PIN_NUM_MISO
#define AD5940_MOSI_PIN   PIN_NUM_MOSI
#define AD5940_CS_PIN     PIN_NUM_CS

#ifdef CONFIG_USE_XIAO_BOARD
#  ifdef CONFIG_IDF_TARGET_ESP32C3
#    define AD5940_RST_PIN    21   
#    define AD5940_GP0INT_PIN 3
#    define AD5940_GP1_PIN    4
#    define AD5940_GP2_PIN    5

#  elif CONFIG_IDF_TARGET_ESP32C6
#    define AD5940_RST_PIN    16   
#    define AD5940_GP0INT_PIN 0
#    define AD5940_GP1_PIN    1
#    define AD5940_GP2_PIN    2

#  elif CONFIG_IDF_TARGET_ESP32S3
#    define AD5940_RST_PIN    43   
#    define AD5940_GP0INT_PIN 2
#    define AD5940_GP1_PIN    3
#    define AD5940_GP2_PIN    4
#  endif

#else
#error "Please, set the right AD594x-to-MCU interfacig pins according to your design"
#endif

/******************************************************************************
 * @brief Accessor function to get the ESP32 SPI handle (getter function)
 * 
******************************************************************************/ 
spi_device_handle_t esp_getSPI(void);

/******************************************************************************
 * @brief Setup function for initialising ESP32 SPI protocol
 * 
******************************************************************************/ 
void esp_initSPI(void);

/******************************************************************************
 * @brief Setup function for initialising ESP32 GPIOs in accordance to AD594x
 * 
******************************************************************************/ 
void esp_initGPIO(void);

/******************************************************************************
 * @brief Function for R/W 'N' single bytes via ESP32 SPI protocol
 * 
******************************************************************************/ 
void esp_spi_ReadWriteNBytes(uint8_t *pSendBuffer, uint8_t *pRecvBuff, unsigned long length);

#endif // ESP32_H