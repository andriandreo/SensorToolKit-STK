/******************************************************************************
 * @file       esp32.c
 * @brief      (Generic) ESP32 board implementation code needed for port file.
 *             Source file.
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
#ifndef ESP32_C
#define ESP32_C

// Include dependencies
#include "esp32.h"

static const char TAG[] = "esp32-AD594x";

/* Define the SPI handle, static to this source file */
static spi_device_handle_t spi = NULL;

spi_device_handle_t esp_getSPI(void) {
    if (spi == NULL) {
        esp_initSPI();  // Initialise the SPI if it not done yet
    }
    return spi;
}

void esp_initSPI(void){
    esp_err_t ret;
    ESP_LOGI(TAG, "[SPI] Initializing bus SPI%d...", AD594x_HOST+1); // '+1' as `SPI_HOST` is 0-based (SPI1=0, SPI2=1,...)
    if (spi != NULL) {
        // If SPI is already initialised, return early
        return;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };

    spi_device_interface_config_t devcfg = {
        .clock_source = SPI_CLK_SRC_DEFAULT,   // Cannot use XTAL as it is not connected to the ESP32, just to AFE
        .queue_size = 1,                       // No 'queing' is intended as per Datasheet
        .mode = 0,                             // SPI mode 0 (SCLK idles low, data clocked on SCLK falling edge)
        .clock_speed_hz = SPI_MASTER_FREQ_16M, // Clock out at 16 MHz
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(AD594x_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Attach the AFE to the SPI bus
    ret = spi_bus_add_device(AD594x_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void esp_spi_ReadWriteNBytes(uint8_t *pSendBuffer, uint8_t *pRecvBuff, unsigned long length){
    if (length == 0) return;           // No need to send anything

    spi_transaction_t t = {
        .length = length*8,            // `length` parameter in bytes, transaction `t.length` is in bits.
        .tx_buffer = pSendBuffer,      // TX Data to be sent (Write)
        .rx_buffer = pRecvBuff,        // RX Data to be received (Read)
        // TODO: could be more efficient to use tx and rx data instead of buffer | no need to alloc memory.
    };

    esp_err_t ret = spi_device_polling_transmit(esp_getSPI(), &t); // Transmit!
    assert(ret == ESP_OK); // Should have had no issues.
}

void esp_initGPIO(void){
    gpio_reset_pin(AD5940_CS_PIN);
    gpio_set_intr_type(AD5940_CS_PIN, GPIO_INTR_DISABLE);     // Disable interrupt (only enabled for GPIO0)
    gpio_set_direction(AD5940_CS_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(AD5940_CS_PIN, GPIO_PULLUP_ONLY);

    gpio_reset_pin(AD5940_RST_PIN);
    gpio_set_intr_type(AD5940_RST_PIN, GPIO_INTR_DISABLE);    // Disable interrupt (only enabled for GPIO0)
    gpio_set_direction(AD5940_RST_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(AD5940_RST_PIN, GPIO_PULLUP_ONLY);

    gpio_reset_pin(AD5940_GP0INT_PIN);
    gpio_set_direction(AD5940_GP0INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(AD5940_GP0INT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(AD5940_GP0INT_PIN, GPIO_INTR_NEGEDGE); // Interrupt on falling edge

    gpio_reset_pin(AD5940_GP1_PIN);
    gpio_set_direction(AD5940_GP1_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(AD5940_GP1_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(AD5940_GP1_PIN, GPIO_INTR_DISABLE);    // Disable interrupt (only enabled for GPIO0)
}

#endif // ESP32_C