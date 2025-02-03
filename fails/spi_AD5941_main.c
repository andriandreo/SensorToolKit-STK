/* SPI Master Half Duplex EEPROM example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include "ad5940.h"


/*
 This code demonstrates how to use the SPI master half duplex mode to read/write an AD594x AFE.
*/

#ifdef CONFIG_IDF_TARGET_ESP32
#  ifdef CONFIG_EXAMPLE_USE_SPI1_PINS
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
#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C6
#  define AD594x_HOST    SPI2_HOST

#  define PIN_NUM_MISO 9
#  define PIN_NUM_MOSI 10
#  define PIN_NUM_CLK  8
#  define PIN_NUM_CS   20

#elif CONFIG_IDF_TARGET_ESP32S3
#  define AD594x_HOST    SPI2_HOST

#  define PIN_NUM_MISO 8
#  define PIN_NUM_MOSI 9
#  define PIN_NUM_CLK  7
#  define PIN_NUM_CS   44

#elif CONFIG_IDF_TARGET_ESP32H2
#  define AD594x_HOST    SPI2_HOST

#  define PIN_NUM_MISO 0
#  define PIN_NUM_MOSI 5
#  define PIN_NUM_CLK  4
#  define PIN_NUM_CS   1
#endif

/*
 The AFE needs a bunch of command/argument values to be initialized. They are stored in this struct. - [!!!] – Needed?
*/
typedef struct {
    uint16_t addr;
    uint16_t data;
} afe_init_cmd_t;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const afe_init_cmd_t init_cmds[]={
    {0x0908, 0x02C9},
    {0x0C08, 0x206C},
    {0x21F0, 0x0010},
    {0x0410, 0x02C9},
    {0x0A28, 0x0009},
    {0x238C, 0x0104},
    {0x0A04, 0x4859},
    {0x0A04, 0xF27B},
    {0x0A00, 0x8009},
    {0x22F0, 0x0000},
};

/* Send a command to the AFE. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void afe_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       // Zero out the transaction
    t.length=8;                     // Command is 8 bits
    t.tx_buffer=&cmd;               // The data is the cmd itself
    if (keep_cs_active) {
      t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   // Keep CS active (low) after data transfer
    }
    spi_device_polling_transmit(spi, &t);  // Transmit!
}

/* Send data to the AFE. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void afe_TXdata(spi_device_handle_t spi, uint8_t *addr, uint8_t *data, int len)
{
    spi_transaction_t t;
    if (len==0) return;                    // No need to send anything

    memset(&t, 0, sizeof(t));              // Zero out the transaction
    t.length=3*8;                          // `len` is in bytes (3 bytes), transaction length is in bits
    t.flags=SPI_TRANS_USE_TXDATA;          // Use `tx_data` directly instead of buffer (32-bit limit)
    t.tx_data[0]=SPICMD_SETADDR;           // Send AFE SPI Command to Set Address as Data
    t.tx_data[1]=addr[0];                  // Send Address MSB
    t.tx_data[2]=addr[1];                  // Send Address LSB
    spi_device_polling_transmit(spi, &t);  // Transmit!

    uint8_t TXdata[5] = {SPICMD_WRITEREG, data[0], data[1], data[2], data[3]}; // Send the command for writing in a register, then data
    memset(&t, 0, sizeof(t));              // Zero out the transaction
    t.length=len*8;                        // `len` is in bytes, transaction length is in bits
    t.tx_buffer=TXdata;                    // Send Data
    spi_device_polling_transmit(spi, &t);  // Transmit!
}

/* Receive data from the AFE. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void afe_RXdata(spi_device_handle_t spi, uint8_t *addr, uint8_t *data, int len)
{
    uint8_t dummyB = 0x00;

    spi_transaction_t t;
    if (len==0) return;                    // No need to send anything

    memset(&t, 0, sizeof(t));              // Zero out the transaction
    t.length=3*8;                          // `len` is in bytes (3 bytes), transaction length is in bits
    t.flags=SPI_TRANS_USE_TXDATA;          // Use `tx_data` directly instead of buffer (32-bit limit)
    t.tx_data[0]=SPICMD_SETADDR;           // Send AFE SPI Command to Set Address as Data
    t.tx_data[1]=addr[0];                  // Send Address MSB
    t.tx_data[2]=addr[1];                  // Send Address LSB
    spi_device_polling_transmit(spi, &t);  // Transmit!

    afe_cmd(spi, SPICMD_READREG, true);    // Send the command for writing in a register, keep CS active (low)
    memset(&t, 0, sizeof(t));              // Zero out the transaction
    t.length=len*8;                        // `len` is in bytes, transaction length is in bits
    t.flags=SPI_TRANS_USE_TXDATA;          // Use `tx_data` directly instead of buffer (32-bit limit)
    t.flags=SPI_TRANS_USE_RXDATA;          // Use `rx_data` directly instead of buffer (32-bit limit)
    t.tx_data[0]=SPICMD_READREG;           // Send AFE SPI Command to Read Data from a register
    t.tx_data[1]=dummyB;                   // Send Dummy byte to initiate Read
    spi_device_polling_transmit(spi, &t);  // Transmit!

    for (int i = 0; i < len; i++) {        // NOTE: equal to `i == len-1`
        data[i]=t.tx_data[i];              // Receive Data – [!!!] – REWORK
    }
}

/* Write Data to a register in the AFE. 
*/
void afe_Write(spi_device_handle_t spi, uint32_t addr, uint32_t data, int len)
{
    uint16_t addr16 = addr & 0xFFFF; // Convert to 16-bit address
    uint8_t addr8[2] = {addr16 >> 8, addr16 & 0xFF}; // Split into two bytes
    
    uint8_t data8[4] = {0, 0, 0, 0};
    for (int i = len; i > 0; i--) {  // NOTE: equal to `i >= 1`
        //data &= 0xFF << (8*(i-1)); // [!!!] – REWORK to avoid rewrite of `data` variable
        data8[len-i] = (data & (0xFF << 8*(i-1))) >> 8*(i-1); // [!!!] – WORKS?
    }
    afe_TXdata(spi, addr8, data8, len);
}

/* Read Data from a register in the AFE. 
*/
uint32_t afe_Read(spi_device_handle_t spi, uint32_t addr, int len)
{
    uint16_t addr16 = addr & 0xFFFF; // Convert to 16-bit address
    uint8_t addr8[2] = {addr16 >> 8, addr16 & 0xFF}; // Split into two bytes
    
    uint8_t data8[4] = {0, 0, 0, 0};
    afe_RXdata(spi, addr8, data8, len);

    uint32_t data = 0;
    switch (len) {
        case 1:
            data = data8[0];
            break;
        case 2:
            data = data8[0] << 8 | data8[1];
            break;
        case 3:
            data = data8[0] << 16 | data8[1] << 8 | data8[2];
            break;
        case 4:
            data = data8[0] << 24 | data8[1] << 16 | data8[2] << 8 | data8[3];
            break;
    }

    return data;
}


//Initialize the AFE
void afe_init(spi_device_handle_t spi)
{
    int cmd=0;
    // const afe_init_cmd_t* init_cmds; // Only needed if function parameters are pointers (CHECK FOR PROPER POINTER HANDLING)

    //Send all the commands
    for (int i = 0; i < 10; i++) {
        afe_Write(spi, init_cmds[cmd].addr, init_cmds[cmd].data, 2);
        cmd++;
    }
}    

static const char TAG[] = "AD594x_test";

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
#ifndef CONFIG_EXAMPLE_USE_SPI1_PINS
    ESP_LOGI(TAG, "Initializing bus SPI%d...", AD594x_HOST+1); // '+1' as `SPI_HOST` is 0-based (SPI1=0, SPI2=1,...)
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    spi_device_interface_config_t devcfg={
        //.command_bits=16,                       // Default to 16-bit commands [!!!] – Need a rework depending on registers
        //.address_bits=32,                       // Default to 32-bit addresses [!!!] – Need a rework if we want to use 16-bit addresses?
        .dummy_bits=8,                          // AFE needs 1 byte
        .clock_source=SPI_CLK_SRC_XTAL,         // Use the external 16 MHz crystal oscillator
        .clock_speed_hz=16*1000*1000,           // Clock out at 16 MHz
        .mode=0,                                // SPI mode 0
        .spics_io_num=PIN_NUM_CS,               // CS pin
        .queue_size=7,                          // We want to be able to queue 7 transactions at a time [!!!]
        //.pre_cb=afe_spi_pre_transfer_callback,  // Specify pre-transfer callback to handle D/C line – Needed for displays and other peripherals, usually not AFEs
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(AD594x_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
#else
    ESP_LOGI(TAG, "Attach to main bus...");
#endif
    // Attach the AFE to the SPI bus
    ret=spi_bus_add_device(AD594x_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    // Initialise the AFE
    ESP_LOGI(TAG, "Initialising device...");
    afe_init(spi);

    // Reading ID Registers
    ESP_LOGI(TAG, "Reading ID Registers");

    uint16_t adiid=afe_Read(spi,REG_AFECON_ADIID,2);
//    ESP_LOGI(TAG, "AD Identifier: %d", adiid);
//
//    uint16_t chipid=afe_Read(spi,REG_AFECON_CHIPID,2);
//    ESP_LOGI(TAG, "Device Identifier + Silicon Rev. Num.: %d", chipid);
//
//    ESP_LOGI(TAG, "Example finished.");
//
//    while (1) {
//        // Add your main loop handling code here.
//        vTaskDelay(1);
//
//        // Reading ID Registers
//        ESP_LOGI(TAG, "Reading ID Registers");
//
//        adiid=afe_Read(spi,REG_AFECON_ADIID,2);
//        ESP_LOGI(TAG, "AD Identifier: %d", adiid);
//
//        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay of 1 second
//
//        chipid=afe_Read(spi,REG_AFECON_CHIPID,2);
//        ESP_LOGI(TAG, "Device Identifier + Silicon Rev. Num.: %d", chipid);
//
//        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay of 1 second
//    }
}
