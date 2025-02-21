/******************************************************************************  
 * @file       ESP32Port.c
 * @brief      (Generic) ESP32 board port file.
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
 * This software is a port of the AD5940 driver for the ESP32 board
 * built from the software provided by Analog Devices, Inc. (ADI).
 * 
 * ****************************************************************************
 * This software is built in collaboration with JLM Innovation GmbH.
 * 
 * It is intended for use in conjunction of the ESP-IDF framework.
 * ----------------------------------------------------------------------------
 * @license: https://github.com/analogdevicesinc/ad5940lib/blob/master/LICENSE 
 *           (accessed on January 30, 2025)
 * 
 * Copyright (c) 2019 Analog Devices, Inc.  All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.  
 * - Modified versions of the software must be conspicuously marked as such.
 * - This software is licensed solely and exclusively for use with processors/products manufactured by or for Analog Devices, Inc.
 * - This software may not be combined or merged with other code in any manner that would cause the software to become subject to terms and conditions which differ from those listed here.
 * - Neither the name of Analog Devices, Inc. nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * - The use of this software may or may not infringe the patent rights of one or more patent holders.  This license does not release you from the requirement that you obtain separate licenses from these patent holders to use this software.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * 2019-01-10-7CBSD SLA
******************************************************************************/

// Include guard
#ifndef ESP32PORT_C
#define ESP32PORT_C

// Include dependencies
#include "ad5940.h"
#include <stdio.h>
#include "esp32.h"

/* Further Declarations/Definitions */
void Ext_Int0_Handler(void *arg); // `INT0` as stated by ADI MCU porting guide, `*arg` added for ESP32 function signature compliance
volatile static uint8_t ucInterrupted = 0; // Flag to indicate interrupt occurred

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to 
                     provide a more efficent way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
  esp_spi_ReadWriteNBytes((uint8_t*) pSendBuffer, (uint8_t*) pRecvBuff, length);
} 

void AD5940_CsClr(void)
{
  gpio_set_level(AD5940_CS_PIN, 0); // Bring CS line low
}

void AD5940_CsSet(void)
{
  gpio_set_level(AD5940_CS_PIN, 1);
}

void AD5940_RstSet(void)
{
  gpio_set_level(AD5940_RST_PIN, 1);
}

void AD5940_RstClr(void)
{
  gpio_set_level(AD5940_RST_PIN, 0);
}

void AD5940_Delay10us(uint32_t time)
{
  time/=100;
  vTaskDelay(time/portTICK_PERIOD_MS);
}

uint32_t AD5940_GetMCUIntFlag(void)
{
	return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
  // Disable and re-enable interrupt to clear and re-arm the interrupt
  gpio_intr_disable(AD5940_GP0INT_PIN); // Disable interrupt
  gpio_intr_enable(AD5940_GP0INT_PIN);  // Re-enable interrupt

	ucInterrupted = 0;
	return 1;
}

uint32_t AD5940_MCUResourceInit(void *pCfg)
{  
  /* Step 1. Initialize SPI peripheral and its GPIOs for CS/RST */
  // DEinitialise Watchog
  esp_task_wdt_deinit(); // Makes testing easier

  //esp_initSPI(); // Lazy init SPI protocol via getter function `esp_getSPI` in SPI R/W - SPI protocol is MSB-first by default for ESP32
  esp_initGPIO();
  /* Enable SPI clock */
  // Enabled by default in `spi_bus_initialize` in ESP32, not like STM32
  
  // Chip Select high to de-select AD594x initially
  AD5940_CsSet();
  AD5940_RstSet();
  
  /* Step 2. Configure external interrupt line */
  /* Initialize GPIO interrupt that connects to AD594x interrupt output pin (GP0, GP3, GP4, GP6 or GP7; GP3-7 are AD5940 only) */
  
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  // Install GPIO Interrupt Service Routine (ISR)
  gpio_install_isr_service(0); // '0' is the default ESP32 interrupt priority

  // Hook ISR handler for specific GPIO(0) pin
  gpio_isr_handler_add(AD5940_GP0INT_PIN, Ext_Int0_Handler, (void*) AD5940_GP0INT_PIN);
  return 0;
}

/* MCU related external line interrupt service routine */
void Ext_Int0_Handler(void *arg)
{
  ucInterrupted = 1;

  // Disable and re-enable interrupt to clear and re-arm the interrupt
  gpio_intr_disable(AD5940_GP0INT_PIN); // Disable interrupt
  gpio_intr_enable(AD5940_GP0INT_PIN);  // Re-enable interrupt

  /* This example just set the flag and deal with interrupt in `AD5940_Main` function. It's your choice to choose how to process interrupt. */
}

#endif // ESP32PORT_C
