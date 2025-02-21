/******************************************************************************  
 * @file       esp32.h
 * @brief      (Generic) ESP32 board implementation code needed for port file.
 *             Header file.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V2.0.0
 * @date       January 2025
 * 
 * @par        Revision History:
 *              * Added support for other ESP32 boards.
 *              * Implemented Wi-Fi and MQTT functionality.
 *              * Checked for redundant includes.
 * 
 * @todo       * Implement Bluetooth functionality.
 *             * Re-implement SPI transactions to use tx/rx data instead of buffers.
 *             * Analyse the need for SPI lazy init.
 *             * Implement UART-INTR functionality if needed.
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
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Needed for delay functions with `vTaskDelay`
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "mqtt5_client.h"

#include "esp_task_wdt.h"  // Needed for watchdog timer functions
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

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

/* 
    This code uses Wi-Fi and MQTT configuration that you can set 
    via project configuration menu `idf.py menuconfig`.

    If you'd rather not, just change the below entries to strings with
    the config you want - i.e. `#define ESP_WIFI_SSID "mywifissid"`
*/
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define ESP_MQTT_USER      CONFIG_ESP_MQTT_USER
#define ESP_MQTT_PASS      CONFIG_ESP_MQTT_PASSWORD

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
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
 * @brief Setup function for initialising ESP32 Wi-Fi protocol and connection
 * 
******************************************************************************/ 
void esp_initWIFI(void);

/******************************************************************************
 * @brief Setup function for initialising ESP32 MQTT protocol and connection
 * 
******************************************************************************/ 
void esp_initMQTT(void);

/******************************************************************************
 * @brief Function for R/W 'N' single bytes via ESP32 SPI protocol
 * 
******************************************************************************/ 
void esp_spi_ReadWriteNBytes(uint8_t *pSendBuffer, uint8_t *pRecvBuff, unsigned long length);

/******************************************************************************
 * @brief Function for sending a string to MQTT broker via ESP32 MQTT protocol
 * 
******************************************************************************/ 
void esp_sendMQTT(char *topic, char *payload);


/******************************************************************************
 * @brief Setup function for initialising ESP32 LOG info display and capabilities
 * 
******************************************************************************/ 
void esp_initLOG(void);

#endif // ESP32_H