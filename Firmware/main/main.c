/******************************************************************************  
 * @file       main.c
 * @brief      Main source file for the setup and implementation of AD594x apps.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V1.0.0
 * @date       October 2024
 * 
 * @par        Revision History:
 *              * Added support for other ESP32 boards.
 *              * Checked for redundant includes.
 * 
 * @todo       * Implement Wi-Fi, Bluetooth and MQTT functionality.
 *             * Re-implement SPI transactions to use tx and rx data instead of buffers.
 *             * Analyse the need for SPI lazy init.
 *             * Implement UART-INTR functionality if needed.
 * 
 * 
 * It is intended to include all the calls to initialise and configure the
 * AD594x, along with the implementation of the application code of choice 
 * to be run on the MCU. Thus, it uses the port file of the ESP32 board family 
 * as suggested by Analog Devices, Inc. in the AD594x porting guide.
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
#ifndef MAIN_C
#define MAIN_C

// Include dependencies
#include <stdio.h>
#include "ad5940.h"

/* Functions that used to initialize MCU platform */
uint32_t MCUPlatformInit(void *pCfg);
void AD5940_Main(void);

void app_main(void)
{
  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);    /* Initialize resources used by AD5940, like SPI/GPIO/Interrupt. */

  printf("Hello AD5940-Build Time:%s\n",__TIME__);
  AD5940_Main();
}

#define DEBUG_UART                         UART_NUM_0 // UART0 as per XIAO pins
//#define DEBUG_UART_IRQN                    

/* Definition for AD5940 Pins */
#define DEBUGUART_TX_PIN                   PIN_NUM_TX
#define DEBUGUART_RX_PIN                   PIN_NUM_RX

void Error_Handler(void){
  while(1);
}

uint32_t MCUPlatformInit(void *pCfg)
{
  /* Init UART */  
  // Configured and started by default in ESP32 via USB, otherwise setup `UrtCfg(int iBaud)` function
	return 1;
}

//void UART0_IRQHandler(void)
//{
//  // Enable UART interrupt source in NVIC
//  /* Rx Interrupt */
//}

/******************************************************************************
#include "stdio.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __ICCARM__
int putchar(int c)
#else
int __attribute__((used)) write(int handle, char *data, int size)
#endif
{
    for (int i = 0; i < size; i++) {
        uart_write_bytes(UART_NUM_1, &data[i], 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Optional: Small delay to avoid overrunning the UART FIFO
    }
    return size;
}

#ifdef __cplusplus
}
#endif
******************************************************************************/

/******************************************************************************
 * @brief  This function create a FreeRTOS task to handle UART communication.
 * @param  None
 * @return None
******************************************************************************/
//void xTask_Handler(void)
//{
//  
//}

#endif // MAIN_C