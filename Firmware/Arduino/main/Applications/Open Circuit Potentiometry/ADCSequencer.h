/******************************************************************************  
 * @file       ADCSequencer.h
 * @brief      General ADC-based Sequencer parameters header file.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V1.0.0
 * @date       December 2024
 * 
 * @par        Revision History:
 * 
 * @todo       
 * 
 * It is intended to include all the declarations, definitions, macros and
 * functions needed to setup the sequencer for general non-complex scenarios.
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
    @defgroup ADC_Sequencer
    @{
  */

// Include guard
#ifndef ADCSEQUENCER_H
#define ADCSEQUENCER_H

#ifdef __cplusplus
extern "C" {
#endif

// Include dependencies
#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by `AppInit` */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD594x */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence */

/* Application related parameters */ 
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  float ODR;                    /* Output Data Rate in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and sample frequency decides the max ODR. */

  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  uint32_t ADCPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint8_t ADCRate;

  uint32_t FifoSrc;             /* FIFO source. FIFOSRC_SINC3, FIFOSRC_DFT, FIFOSRC_SINC2NOTCH, FIFOSRC_VAR or FIFOSRC_MEAN */
  uint32_t DataFifoSrc;         /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3, DATATYPE_SINC2, DATATYPE_DFT or DATATYPE_NOTCH*/   
  uint32_t FifoThresh;          /* FIFO threshold. Should be N*4 */
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  
  uint32_t HstiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */

/* Private variables for internal usage */
  float FreqofData;             /* The frequency of latest data sampled */
  SEQInfo_Type seqinfo0, seqinfo1, seqinfo2, seqinfo3;
  SEQInfo_Type InitSeqInfo, MeasureSeqInfo;
  BoolFlag ADCSeqInited;        /* If the program run firstly, generated sequence commands */
  BoolFlag StopRequired;        /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;       /* Count how many measurements have been made */
  uint32_t MeasSeqCycleCount;   /* How long the measurement sequence will take */
  float MaxODR;                 /* Max ODR for sampling in this config */  

/* End */
}AppADCSeqCfg_Type;

#ifdef __cplusplus
}
#endif

#endif // ADCSEQUENCER_H

/**
 * @}
 * @}
 * */