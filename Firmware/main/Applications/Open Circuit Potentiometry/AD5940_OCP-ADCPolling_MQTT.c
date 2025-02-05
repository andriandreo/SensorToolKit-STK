/******************************************************************************  
 * @file       AD5940_OCP_ADCPolling_MQTT.c
 * @brief      Source file for performing Open Circuit Pontential (OCP)
 *             measurements using ADC Polling mode.
 *             Data is sent to a MQTT broker via Wi-Fi. 
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V1.0.1
 * @date       January 2024
 * 
 * @par        Revision History:
 *             * Implemented NOTCH filter for best results 
 *             * Firt implementation of Wi-Fi and MQTT connectivity
 *
 * @todo       * Check for differences between several ADC MUX inputs.
 *             * Check for the utility of the Digital + Notch Filters (w/ vs. w/o)
 *             * Check whether the calibration parameters need to match operational ones (?)
 *             * Implement Bluetooth functionality. 
 * 
 * ****************************************************************************
 * This software is an implementation for OCP measurementes using the AD594x
 * AFE built from the software provided by Analog Devices, Inc. (ADI).
 * 
 * Adapted from examples code by Analog Devices, Inc.
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
/** @addtogroup AD5940_OCP
  * @{
    @defgroup ADC_Polling
    @{
  */

 // Include guard
#ifndef AD5940_OCP_ADCPolling_C
#define AD5940_OCP_ADCPolling_C

// Include dependencies
#include "ad5940.h"
#include "esp32.h"
#include <stdio.h>

#define ADCPGA_GAIN_SEL   ADCPGA_1P5
#define AD5940_MAX_DAC_OUTPUT   2400.0      // Max. voltage the 6-Bit and 12-Bit DAC can provide
#define AD5940_MIN_DAC_OUTPUT   200.0       // Min. voltage the 6-Bit and 12-Bit DAC can provide
#define AD5940_6BIT_DAC_1LSB    ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 64)      // Size of one LSB of the 6-Bit DAC
#define AD5940_12BIT_DAC_1LSB   ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 4095)    // Size of one LSB of the 12-Bit DAC

static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_GAIN_SEL;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

static void AD5940_AFERefBuffer_Init(void){
  AFERefCfg_Type AFERefBuffer_cfg;
  // Disable all control signals of the AFE
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);
  // Disable high power band-gap
  AFERefBuffer_cfg.HpBandgapEn = bTRUE;
  // Enable high power 1.1 V reference buffer
  AFERefBuffer_cfg.Hp1V1BuffEn = bTRUE;
  // Enable high power 1.8 V reference buffer
  AFERefBuffer_cfg.Hp1V8BuffEn = bTRUE;
  // Disable discharge of 1.1 V capacitor
  AFERefBuffer_cfg.Disc1V1Cap = bFALSE;
  // Disable discharge of 1.8 V capacitor
  AFERefBuffer_cfg.Disc1V8Cap = bFALSE;
  // Disable thermal buffer
  AFERefBuffer_cfg.Hp1V8ThemBuff = bFALSE;
  // Disable current limit for 1.8 V buffer
  AFERefBuffer_cfg.Hp1V8Ilimit = bFALSE;
  // Disable 1.1 V reference buffer
  AFERefBuffer_cfg.Lp1V1BuffEn = bFALSE;
  // Disable 1.8 V reference buffer                                                       
  AFERefBuffer_cfg.Lp1V8BuffEn = bFALSE;
  // Enable low power band gap
  AFERefBuffer_cfg.LpBandgapEn = bTRUE;
  // Enable 2.5 V reference buffer
  AFERefBuffer_cfg.LpRefBufEn = bTRUE;
  // Disable boost buffer current
  AFERefBuffer_cfg.LpRefBoostEn = bFALSE;
  // Configure reference buffer
  AD5940_REFCfgS(&AFERefBuffer_cfg);
}

static void AD5940_LPLoop_Init(void){
  LPLoopCfg_Type LPLoop_cfg;
  // Chose LPAMP0 because LPAMP1 is only available on ADuCM355
  LPLoop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  //LPLoop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM; // Normal power mode
  LPLoop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_BOOST3; // This setting increases amplifier bandwidth and output current capability to the max
  // Enable potential amplifier
  LPLoop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  // Enable low power TIA amplifier
  LPLoop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  LPLoop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K; // Low-pass RC Filter resistor to 20 kOhm
  LPLoop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT; // 0 Ohm Rload
  LPLoop_cfg.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN; // LPTIA Internal RTIA to 20 kOhm
  // Leave SW2 and SW4 open to prevent controlling the CE/ RE pins
  // Close swtiches to support internal resistor (SW6 is connecting -Ve of PA to +Ve of LPTIA)
  //LPLoop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(4) | LPTIASW(7);
  LPLoop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(12) | LPTIASW(13);

  // Low power DAC config
  // Chose LPDAC0 because LPDAC1 is only available on ADuCM355
  LPLoop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  // Disable connection from DACS to amplifiers
  //LPLoop_cfg.LpDacCfg.LpDacSW = 0;
  LPLoop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2PIN | LPDACSW_VZERO2PIN;
  // Set reference for low power DAC to internal 2.5 V
  LPLoop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  // Connect 12-Bit DAC to Vbias
  LPLoop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  // Connect 6-Bit DAC to Vzero
  LPLoop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  // Power off register where 6-Bit and 12-Bit DAC write their data to
  LPLoop_cfg.LpDacCfg.PowerEn = bFALSE;

  // Apply the settings and write to corresponding registers
  AD5940_LPLoopCfgS(&LPLoop_cfg);
}

void AD5940_Main(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  
  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  AD5940_PGA_Calibration();

  /* Initialize AFE basic blocks */
  AD5940_AFERefBuffer_Init();
  AD5940_LPLoop_Init();

  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_VRE0; // RE0 (RE)
  adc_base.ADCMuxN = ADCMUXN_LPTIA0_N; // SE0 (WE)
  adc_base.ADCPga = ADCPGA_GAIN_SEL;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  //adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;     /* Setting not valid when Notch is working simoultaneously */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_2;     /* See Table 41 of AD594x Datasheet for available settings */
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  //adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. HERE BYPASSED (!) */
  adc_filter.BpNotch = bFALSE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. HERE USED (!) */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  //AD5940_ADCMuxCfgS(ADCMUXP_AIN1, ADCMUXN_AIN0);   /* Optionally, you can change ADC MUX with this function */ // [!!!] - INPUT CHANGE HERE!

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 

  //AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  //AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_ADCPowerCtrlS(bTRUE);
  AD5940_ADCConvtCtrlS(bTRUE);
  
  while(1)
  {
    uint32_t rd;
    char MQTTstring[64];
    if(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_SINC2RDY))  
    {
      static uint32_t count;
      AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
      rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      count ++;
      /* ADC Sample rate is 800kSPS. SINC3 OSR is 2, SINC2 OSR is 1333. So the final output data rate is 800kSPS/2/1333 = 300.0750Hz */
      if(count == 300) /* Print data @1Hz */
      {
        count = 0;
        float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
        printf("ADC Code: %d, diff-volt: %.4f\n",(int)rd, diff_volt);
        snprintf(MQTTstring, sizeof(MQTTstring), "{\"ADCCode\":%d, \"OCP\":%.4f}", (int)rd, diff_volt);
        //printf("%s\n", MQTTstring);
        esp_sendMQTT("SENSOR/OCP", MQTTstring);
      }
    }
  }
}

#endif // AD5940_OCP_ADCPolling_C

/**
 * @}
 * @}
 * */
