/******************************************************************************  
 * @file       ESP32Port.c
 * @brief      (Generic) ESP32 board port file.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V1.0.0
 * @date       November 2023
 * 
 * @par        Revision History:
 * @todo       * Add support for other ESP32 boards.
 * 
 * This software is a port of the AD5940 driver for the ESP32 board
 * built from the software provided by Analog Devices, Inc. (ADI).
 * It is intended for use in conjunction of the ESP-IDF framework.
 * 
 * Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
 * 
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
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

void AD5940_RstClr(void)
{
  gpio_set_level(AD5940_RST_PIN, 0);
}

void AD5940_RstSet(void)
{
  gpio_set_level(AD5940_RST_PIN, 1);
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
	ucInterrupted = 0;
	return 1;
}

uint32_t AD5940_MCUResourceInit(void *pCfg)
{  
  /* Step 1. Initialize SPI peripheral and its GPIOs for CS/RST */
  // DEinitialise Watchog
  esp_task_wdt_deinit(); // Makes testing easier

  esp_initSPI(); // SPI protocol is MSB-first by default for ESP32
  esp_initGPIO();
  /* Enable SPI clock */
  // Enabled by default in `spi_bus_initialize` in ESP32, not like STM32
  
  // Chip Select high to de-select AD594x initially
  AD5940_CsSet();
  AD5940_RstSet();
  
  /* Step 2. Configure external interrupot line */
  /* Initialize GPIO interrupt that connects to AD594x interrupt output pin (GP0, GP3, GP4, GP6 or GP7; GP3-7 are AD5940 only) */
  
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  // Install GPIO Interrupt Service Routine (ISR)
  gpio_install_isr_service(0); // '0' is the default ESP32 interrupt priority

  // Hook ISR handler for specific GPIO(0) pin
  gpio_isr_handler_add(AD5940_GP0INT_PIN, Ext_Int0_Handler, (void*) AD5940_GP0INT_PIN);
  return 0;
}

/* MCU related external line interrupt service routine */
// The interrupt handler handles the interrupt to the MCU
// when the AD594x INTC pin generates an interrupt to alert the MCU that data is ready
void Ext_Int0_Handler(void *arg)
{
  ucInterrupted = 1;
  /* This example just set the flag and deal with interrupt in `AD5940_Main` function. It's your choice to choose how to process interrupt. */
  //__HAL_GPIO_EXTI_CLEAR_IT(AD5940_GP0INT_PIN); // [!!!] CLEAR PIN as they do with STM32?
}

#endif // ESP32PORT_C
