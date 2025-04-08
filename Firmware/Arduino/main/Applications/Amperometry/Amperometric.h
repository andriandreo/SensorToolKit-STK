/******************************************************************************  
 * @file       Amperometric.h
 * @brief      Header file for performing amperometric measurements
 *             via the LPTIA loop using ADC with sequencer support.
 * 
 * @author     Andrés Alberto Andreo Acosta (based on mlambe's work)
 * @version    V1.0.0
 * @date       January 2025
 * 
 * @par        Revision History:
 * 
 * @todo       
 * 
 * ****************************************************************************
 * This software is an implementation for amperometric measurementes using the 
 * AD594x AFE built from the software provided by Analog Devices, Inc. (ADI).
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
/** @addtogroup AD5940_AMP
  * @{
    @defgroup LPTIA_Amperometry
    @{
  */

// Include guard
#ifndef AMPEROMETRIC_H
#define AMPEROMETRIC_H

#ifdef __cplusplus
extern "C" {
#endif

// Include dependencies
#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define RCAL 200.0          /* Set to the RCAL value for your design - 200 Ohm on STK-board (!) */
#define EXT_RTIA 10000000.0 /* Set to the External RTIA (RC0_0-RC0_1) value for your design - 10 MOhm on STK-board (!) */

#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV
/* 
  Note: this example will use SEQID_0 as measurement sequence, and use SEQID_1 as init sequence. 
  SEQID_3 is used for calibration.
*/

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppAMPInit */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
  
/* Application related parameters */ 
  BoolFlag ReDoRtiaCal;         /* Set this flag to bTRUE when there is need to do calibration. */
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t FifoThresh;           /* FIFO threshold. Should be N*4 */   
  float AmpODR;                 /* in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically.*/
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  float RcalVal;                /* Rcal value in Ohm */
  float ADCRefVolt;               /* Measured 1.82 V reference*/
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  uint32_t ADCPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t DataFifoSrc;         /* DataFIFO source. FIFOSRC_SINC3, FIFOSRC_DFT, FIFOSRC_SINC2NOTCH, FIFOSRC_VAR, FIFOSRC_MEAN*/
  uint32_t LptiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t LpTiaRf;             /* Rfilter select */
  uint32_t LpTiaRl;             /* SE0 Rload select */
  fImpPol_Type RtiaCalValue;           /* Calibrated Rtia value */
  float Vzero;                  /* Voltage on SE0 pin and Vzero, optimumly 1100mV*/
  float SensorBias;             /* Sensor bias voltage = VRE0 - VSE0 */
  BoolFlag ExtRtia;             /* Use internal or external Rtia */
  float ExtRtiaVal;							/* External Rtia value if using one */
  BoolFlag AMPInited;           /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;          /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;         /* Count how many times impedance have been measured */
/* End */
}AppAMPCfg_Type;

/**
 * int32_t type Impedance result in Cartesian coordinate 
*/
typedef struct
{
  float Current;
  float Voltage;
}fAmpRes_Type;



#define AMPCTRL_START          0
#define AMPCTRL_STOPNOW        1
#define AMPCTRL_STOPSYNC       2
#define AMPCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

AD5940Err AppAMPGetCfg(void *pCfg);
AD5940Err AppAMPInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppAMPISR(void *pBuff, uint32_t *pCount);
AD5940Err AppAMPCtrl(int32_t AmpCtrl, void *pPara);
float AppAMPCalcVoltage(uint32_t ADCcode);
float AppAMPCalcCurrent(uint32_t ADCcode);

#ifdef __cplusplus
}
#endif

#endif // AMPEROMETRIC_H

/**
 * @}
 * @}
 * */

