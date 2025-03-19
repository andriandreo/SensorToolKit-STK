/******************************************************************************  
 * @file       ARDUINOPort.cpp
 * @brief      (Generic) ARDUINO board port file.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V1.0.0
 * @date       February 2025
 * 
 * @par        Revision History:
 * 
 * @todo       * Implement Wi-Fi, MQTT, BLE and NFC connectivity protocols.
 *
 * 
 * This software is a port of the AD5940 driver for ARDUINO-compatible boards
 * built from the software provided by Analog Devices, Inc. (ADI).
 * 
 * ****************************************************************************
 * This software is built in collaboration with JLM Innovation GmbH.
 * 
 * It is intended for use in conjunction of ARDUINO IDE.
 * ----------------------------------------------------------------------------
 * @license: https://github.com/analogdevicesinc/ad5940lib/blob/master/LICENSE 
 *           (accessed on February 12, 2025)
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
#ifndef ARDUINOPORT_CPP
#define ARDUINOPORT_CPP

// Include dependencies
extern "C" {
  #include "ad5940.h"
  #include "math.h"
  #include <stdio.h>
}
#include <SPI.h>
#include <Arduino.h>

/******************************************************************************
 * NOTE: CHOOSE THE MCU TO BE USED 
 *
******************************************************************************/
// #define XIAOnRF
// #define XIAOS3
#define XIAOC3
/******************************************************************************
******************************************************************************/

#if defined(XIAOS3) && defined(XIAOnRF) && defined(XIAOC3)
#error Please select the correct chip by define XIAOS3, XIAOC3 or XIAOnRF.
#endif

#if defined(XIAOS3) && (defined(XIAOnRF) || defined(XIAOC3))
#error Please select the correct chip by define XIAOS3, XIAOC3 or XIAOnRF.
#endif

#if defined(XIAOC3) && defined(XIAOnRF)
#error Please select the correct chip by define XIAOS3, XIAOC3 or XIAOnRF.
#endif

#if !defined(XIAOS3) && !defined(XIAOnRF) && !defined(XIAOC3)
#error Please select the correct chip by define XIAOS3, XIAOC3 or XIAOnRF.
#endif

#if defined(XIAOnRF)
#include <LibPrintf.h> // Allows to use C-style `printf()` statement instead of Arduino style `Serial.print()` - Sth. similar already included for ESP32 by libraries thereof
#endif

#if defined(XIAOnRF) && !defined(ARDUINO_ARCH_MBED)
#include "Adafruit_TinyUSB.h"
#endif

/******************************************************************************
 * NOTE: User could configure following parameters 
 * ALSO UNCOMMENT `#define CHIPSEL_594X` LINE IN `ad5940.h` LIBRARY FILE (!)
 *
******************************************************************************/
// Pin macros depending on target board, can be extended with other boards 
// (check if enough flash memory is available)
#ifdef XIAOS3
#define AD5940_CS_PIN 44    // XIAO ESP32S3 GPIO 44
#define AD5940_RST_PIN 43   // XIAO ESP32S3 GPIO 0
#define AD5940_GP0INT_PIN 2 // XIAO ESP32S3 GPIO 43 (AD594x `INT0`)
#endif

#ifdef XIAOC3
#define AD5940_CS_PIN 20    // XIAO ESP32C3 GPIO 20
#define AD5940_RST_PIN 21   // XIAO ESP32C3 GPIO 22
#define AD5940_GP0INT_PIN 3 // XIAO ESP32C3 GPIO 3 (AD594x `INT0`)
#endif

#ifdef XIAOnRF
#define AD5940_CS_PIN 7     // XIAO ESP32S3 GPIO 44
#define AD5940_RST_PIN 6    // XIAO ESP32S3 GPIO 0
#define AD5940_GP0INT_PIN 1 // XIAO ESP32S3 GPIO 43 (AD594x `INT0`)
#endif

#if defined(ARDUINO_ARCH_MBED)
// Define SPI pins explicitly for XIAO nRF52840
#define SPI_MOSI   D10
#define SPI_MISO   D9
#define SPI_SCK    D8
MbedSPI mbedSPI(SPI_MOSI, SPI_MISO, SPI_SCK);
#endif
/******************************************************************************
******************************************************************************/

/* Further Declarations/Definitions */
void Ext_Int0_Handler(void); // `INT0` as stated by ADI MCU porting guide
volatile static uint8_t ucInterrupted = 0; // Flag to indicate interrupt occurred

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to
         provide a more efficient way to transmit/receive data.
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
  /* SPI parameters for R/W operation:
  *   - `speedMaximum`: The maximum speed of communication. For a AD594x rated up to 16 MHz, use 16000000.
  *   - `dataOrder`: `MSBFIRST` or `LSBFIRST` - NOTE: ESP32 BSP uses `SPI_MSBFIRST`/`SPI_LSBFIRST` instead (!). 
  *   - `dataMode`: `SPI_MODE0`, `SPI_MODE1`, `SPI_MODE2`, or `SPI_MODE3` - `SPI_MODE0` | Data on SCLK falling edge.
  */
  #if defined(XIAOS3) || defined(XIAOC3) || defined(ESP32)
  SPI.beginTransaction(SPISettings(16000000, SPI_MSBFIRST, SPI_MODE0));
  #elif defined(ARDUINO_ARCH_MBED)
  mbedSPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  #else
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  #endif

  for (int i = 0; i < length; i++)
  {
    #if defined(ARDUINO_ARCH_MBED)
    *pRecvBuff++ = mbedSPI.transfer(*pSendBuffer++);
    #else
    *pRecvBuff++ = SPI.transfer(*pSendBuffer++); 
    #endif
  }

  #if defined(ARDUINO_ARCH_MBED)
  mbedSPI.endTransaction();
  #else
  SPI.endTransaction();
  #endif
}

void AD5940_CsClr(void)
{
  digitalWrite(AD5940_CS_PIN, LOW); // Bring CS line low
}

void AD5940_CsSet(void)
{
  digitalWrite(AD5940_CS_PIN, HIGH);
}

void AD5940_RstSet(void)
{
  digitalWrite(AD5940_RST_PIN, HIGH);
}

void AD5940_RstClr(void)
{
  digitalWrite(AD5940_RST_PIN, LOW);
}

/******************************************************************************
 * @brief Timers implementation and `AD5940_Delay10us` functions
 * 
******************************************************************************/ 
#if defined(XIAOS3) || defined(XIAOC3) || defined(ESP32)
#include "driver/timer.h"

// ESP32 implementation using hardware timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool delayFlag = false;

void IRAM_ATTR onTimer() {
    delayFlag = true; // Set the flag in the ISR
}

void initTimer() {
    timer = timerBegin(100000); // 100 kHz → 10 µs per tick
    timerAttachInterrupt(timer, &onTimer); // Attach the ISR function
    timerAlarm(timer, 10, true, 0); // Configure the timer alarm to trigger every 10 µs
}

void AD5940_Delay10us(uint32_t time) {
    while (time > 0) {
        delayFlag = false;
        while (!delayFlag);
        time--;
    }
}

#elif defined(ARDUINO_NRF52_ADAFRUIT) // Non-mbed (Adafruit-based BSP)
#include <nrf_timer.h>
#include <nrf.h>

// nRF52 implementation using nrf_timer
volatile bool delayFlag = false;

#define TIMER_INSTANCE NRF_TIMER3 // Using TIMER3 to avoid conflicts (TIMER0-2 reserved by SOFT-DEVICE)

extern "C" void TIMER3_IRQHandler(void) {
    if (nrf_timer_event_check(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0)) {
        nrf_timer_event_clear(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
        delayFlag = true;
    }
}

void initTimer() {
    // Enable TIMER3 interrupt in NVIC
    NVIC_EnableIRQ(TIMER3_IRQn);
    NVIC_SetPriority(TIMER3_IRQn, 1);  // Set a reasonable priority

    // Configure TIMER3 as a 16 MHz timer (16 ticks = 1 µs)
    nrf_timer_mode_set(TIMER_INSTANCE, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(TIMER_INSTANCE, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_frequency_set(TIMER_INSTANCE, NRF_TIMER_FREQ_16MHz);

    // Set up compare event for 10 µs
    nrf_timer_cc_set(TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, 160);

    // Enable interrupt on compare event
    nrf_timer_int_enable(TIMER_INSTANCE, NRF_TIMER_INT_COMPARE0_MASK);
}

void AD5940_Delay10us(uint32_t time) {
    while (time > 0) {
        delayFlag = false;
        // Clear and start the timer
        nrf_timer_task_trigger(TIMER_INSTANCE, NRF_TIMER_TASK_CLEAR);
        nrf_timer_task_trigger(TIMER_INSTANCE, NRF_TIMER_TASK_START);
        while (!delayFlag);
        time--;
    }
}

#elif defined(ARDUINO_ARCH_MBED) // mbed-enabled BSP
#include <mbed.h>

// nRF52 (mbed OS) implementation using mbed::Timer
mbed::Timer timer;

void initTimer() {
    // No special setup required for mbed Timer
}

void AD5940_Delay10us(uint32_t time) {
    timer.reset();
    timer.start();
    while (timer.elapsed_time().count() < time * 10); // Convert to µs
    timer.stop();
}

#else // Fallback for other MCUs
void initTimer() {
    // No special setup required for micros() method
}

void AD5940_Delay10us(uint32_t time) {
    unsigned long start = micros();
    while (micros() - start < time * 10); // Fallback using micros()
}
#endif
/******************************************************************************
******************************************************************************/

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
  #if defined(ARDUINO_ARCH_MBED)
  mbedSPI.begin();
  #else
  SPI.begin();
  #endif
  
  // SPI CS GPIO pin configuration
  pinMode(AD5940_CS_PIN, OUTPUT);
  // SPI RST GPIO pin configuration
  pinMode(AD5940_RST_PIN, OUTPUT);

  // Chip Select high to de-select AD594x initially
  AD5940_CsSet();
  AD5940_RstSet();

  // Set the SPI parameters
  // Already set for every SPI transaction

  /* Step 2. Configure external interrupt line */
  /* Initialize GPIO interrupt that connects to AD594x interrupt output pin (GP0, GP3, GP4, GP6 or GP7; GP3-7 are AD5940 only) */
  
  // GPIO(0) INT pin configuration
  pinMode(AD5940_GP0INT_PIN, INPUT_PULLUP);
  // Hook ISR handler for specific GPIO(0) pin
  attachInterrupt(digitalPinToInterrupt(AD5940_GP0INT_PIN), Ext_Int0_Handler, FALLING);

  return 0;
}

/* MCU related external line interrupt service routine */
void Ext_Int0_Handler()
{
  ucInterrupted = 1;
  /* This example just set the flag and deal with interrupt in `AD5940_Main` function. It's your choice to choose how to process interrupt. */
}

#endif // ARDUINOPORT_CPP
