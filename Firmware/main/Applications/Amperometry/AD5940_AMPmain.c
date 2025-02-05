/******************************************************************************  
 * @file       AD5940_AMPmain.c
 * @brief      Source file used to control amperometric application and further 
 *             process data.
 * 
 * @author     Andrés Alberto Andreo Acosta (based on nxu2's work)
 * @version    V1.0.0
 * @date       January 2025
 * 
 * @par        Revision History:
 * 
 * @todo       * Test HSTIA as the AFE for MUX with Switch Matrix
 *             * Minimize RF-coupled signal (+/-6 nA of noise)
 * 
 * This code is configured to take sub-µA current measurements with a 2-wire 
 * zero-biased sensor.
 * 
 * The sensor is supposed to be connected to SE0/RE0 by default. RE0/CE0 are
 * shorted internally.
 * 
 * (!) Current input range is mainly controlled via appropriate selection of 
 * RTIA in such a way the resulting input voltage is inside the AFE input 
 * range, i.e., +/- 900 mV.
 * Change RTIA value or use an external one accordingly to your needs.
 * 
 * The LPTIA Switch Configuration is chosen to optimize the performance for
 * zero-biased sensors (3rd and 4th options in Table 21 of datasheet).
 * 
 * Notch filter enabled is implemented in this example for best results. 
 * See comments for available alternatives (bypassed notch).
 * 
 * `printf` statements are thought to generate CSV-like logs reachable by UART.
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
#ifndef AD5940_AMPmain_C
#define AD5940_AMPmain_C

// Include dependencies
#include "Amperometric.h"

#define APPBUFF_SIZE 1000
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;
int count = 0;
/* It's your choice here how to do with the data. Here is just an example to print them to UART */
int32_t AMPShowResult(float *pData, uint32_t DataCount)
{
  /* Print data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("%d, %f uA\n", count, pData[i]);
  }

  count += 1;
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
	SEQCfg_Type seq_cfg;
  AGPIOCfg_Type gpio_cfg;
	LFOSCMeasure_Type LfoscMeasure;
  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                      /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 1;      
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
	fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
	
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_SYNC|GP1_SLEEP|GP0_INT;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
	
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  /* Measure LFOSC frequency */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  // printf("Freq:%f\n", LFOSCFreq); /* DEBUG */
  return 0;
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940AMPStructInit(void)
{
  AppAMPCfg_Type *pAMPCfg;
  
  AppAMPGetCfg(&pAMPCfg);
	pAMPCfg->WuptClkFreq = LFOSCFreq;
  /* Configure general parameters */
  pAMPCfg->SeqStartAddr = 0;
  pAMPCfg->MaxSeqLen = 512;     /* @todo add checker in function */  
  pAMPCfg->RcalVal = RCAL;      /* RCAL on EndoPALi board = 200 Ohm */
  pAMPCfg->NumOfData = -1;      /* Never stop until you stop it manually by AppAMPCtrl() function */	
	
	
	/* Configure measurement parameters */
  pAMPCfg->AmpODR = 1;          	        /* Time between samples in seconds */
  pAMPCfg->FifoThresh = 1;      		      /* Number of measurements before alerting host microcontroller */

  pAMPCfg->SensorBias = 0;   			        /* Sensor bias voltage between RE0 and SE0 - Zero-biased sensor here, change LPTIASWx otherwise as well (!) */
	pAMPCfg->LptiaRtiaSel = LPTIARTIA_160K; /* Must be < 160 KOhm | Rcal = 200 Ohm (!) */ 
	pAMPCfg->LpTiaRl = LPTIARLOAD_3K6;
	pAMPCfg->Vzero = 1100;        		      /* Vzero voltage. Voltage on Sense electrode. Unit is mV*/
	
	pAMPCfg->ADCRefVolt = 1.82;		          /* Measure voltage on Vref_1V8 pin */
}

void AD5940_Main(void)
{
  uint32_t temp;
  
  AD5940PlatformCfg();
  AD5940AMPStructInit();                  /* Configure your parameters in this function */ 
  AppAMPInit(AppBuff, APPBUFF_SIZE);      /* Initialize AMP application. Provide a buffer, which is used to store sequencer commands */
  AppAMPCtrl(AMPCTRL_START, 0);           /* Control AMP measurement to start. Second parameter has no meaning with this command. */

  while(1)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppAMPISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      AMPShowResult((float*)AppBuff, temp); /* Show the results to UART */
    }
  }
}

#endif // AD5940_AMPmain_C

/**
 * @}
 * @}
 * */