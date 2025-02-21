/******************************************************************************  
 * @file       Amperometric.c
 * @brief      Source file for performing amperometric measurements
 *             via the LPTIA loop using ADC with sequencer support.
 * 
 * @author     Andrés Alberto Andreo Acosta (based on mlambe's work)
 * @version    V1.0.0
 * @date       January 2025
 * 
 * @par        Revision History:
 * 
 * @todo       * Test HSTIA as the AFE for MUX with Switch Matrix
 *             * Minimize RF-coupled signal (+/-6 nA of noise)
 * 
 * This code configures ADC, DAC, LPTIA loop, sequencer and FIFO to take
 * sub-µA current measurements with a 2-wire zero-biased sensor.
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
#ifndef AMPEROMETRIC_C
#define AMPEROMETRIC_C

// Include dependencies
#include "Amperometric.h"

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters.
*/
AppAMPCfg_Type AppAMPCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,
  .FifoThresh = 5,                /* Number of points for FIFO */
  
  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .AmpODR = 1.0,                  /* Sample time in seconds. I.e. every 5 (FIFO_THRES = 5) seconds make a measurement */
  .NumOfData = -1,
  .RcalVal = RCAL,             
  .PwrMod = AFEPWR_LP,
  .AMPInited = bFALSE,
  .StopRequired = bFALSE,
  
  /* LPTIA Configure */
  .ExtRtia = bFALSE,              /* Set to true if using external RTIA */
  .LptiaRtiaSel = LPTIARTIA_160K, /* Configure RTIA - Rcal = 200 Ohm cannot CAL RTIA > 160 KOhm (!) */ 
  .LpTiaRf = LPTIARF_1M,          /* Configure LPF resistor - 1 MOhm optimal for DC meas. as per datasheet */
  .LpTiaRl = LPTIARLOAD_SHORT,
  .ReDoRtiaCal = bTRUE,
  .RtiaCalValue = {0.0, 0.0},
	.ExtRtiaVal = EXT_RTIA,       
  
/*LPDAC Configure */
  .Vzero = 1100,                  /* Sets voltage on SE0 and LPTIA */
  .SensorBias = 0,                /* Sets voltage between RE0 and SE0 - Zero-biased sensor here, change LPTIASWx otherwise as well (!) */
  
/* ADC Configure*/
  .ADCPgaGain = ADCPGA_1P5,
  .ADCSinc3Osr = ADCSINC3OSR_4,
  .ADCSinc2Osr = ADCSINC2OSR_22,
  .DataFifoSrc = FIFOSRC_SINC2NOTCH,
  .ADCRefVolt = 1.8162,			      /* Measure voltage on ADCRefVolt pin and enter here*/
};

/**
   This function is provided for upper controllers that want to change 
   application parameters specially for user defined parameters.
*/
AD5940Err AppAMPGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppAMPCfg_Type**)pCfg = &AppAMPCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppAMPCtrl(int32_t AmpCtrl, void *pPara)
{
  switch (AmpCtrl)
  {
    case AMPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;

      AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
      if(AppAMPCfg.AMPInited == bFALSE)
        return AD5940ERR_APPERROR;
      /* Start it */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.SeqxSleepTime[SEQID_0] = 4-1;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppAMPCfg.WuptClkFreq*AppAMPCfg.AmpODR)-4-1; 
      AD5940_WUPTCfg(&wupt_cfg);
      
      AppAMPCfg.FifoDataCount = 0;  /* restart */
      break;
    }
    case AMPCTRL_STOPNOW:
    {
      AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
      /* Start Wupt right now */
      AD5940_WUPTCtrl(bFALSE);
      /* There is chance this operation will fail because sequencer could put AFE back 
        to hibernate mode just after waking up. Use STOPSYNC is better. */
      AD5940_WUPTCtrl(bFALSE);
      break;
    }
    case AMPCTRL_STOPSYNC:
    {
      AppAMPCfg.StopRequired = bTRUE;
      break;
    }
    case AMPCTRL_SHUTDOWN:
    {
      AppAMPCtrl(AMPCTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
      /* Turn off LPloop related blocks which are not controlled automatically by sleep operation */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* Enter Hibernate */
    }
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

/* Generate init sequence */
static AD5940Err AppAMPSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lp_loop;
  DSPCfg_Type dsp_cfg;
  SWMatrixCfg_Type sw_cfg;
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  //AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bTRUE;
  aferef_cfg.Lp1V8BuffEn = bTRUE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	

	lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;
  lp_loop.LpDacCfg.DacData6Bit = (uint32_t)((AppAMPCfg.Vzero-200)/DAC6BITVOLT_1LSB);
	lp_loop.LpDacCfg.DacData12Bit =(int32_t)((AppAMPCfg.SensorBias)/DAC12BITVOLT_1LSB) + lp_loop.LpDacCfg.DacData6Bit*64;
	if(lp_loop.LpDacCfg.DacData12Bit>lp_loop.LpDacCfg.DacData6Bit*64)
		lp_loop.LpDacCfg.DacData12Bit--;
	lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = AppAMPCfg.LpTiaRf;
  lp_loop.LpAmpCfg.LpTiaRload = AppAMPCfg.LpTiaRl;
  if(AppAMPCfg.ExtRtia == bTRUE)
  {
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(9)|LPTIASW(2)|LPTIASW(4)|LPTIASW(8)|LPTIASW(5)|LPTIASW(6)|LPTIASW(12)|LPTIASW(13); /* SW8 to short CE0 & RE0 internally; SW6 to short RE0 & SE0 to Vbias - Best performance for zero-biased sensors */
  }else
  {
    lp_loop.LpAmpCfg.LpTiaRtia = AppAMPCfg.LptiaRtiaSel;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5)|LPTIASW(6)|LPTIASW(8)|LPTIASW(2)|LPTIASW(4)|LPTIASW(12)|LPTIASW(13); /* SW8 to short CE0 & RE0 internally; SW6 to short RE0 & SE0 to Vbias - Best performance for zero-biased sensors */
  }
  AD5940_LPLoopCfgS(&lp_loop);

  
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_VZERO0;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_AIN4;
  dsp_cfg.ADCBaseCfg.ADCPga = AppAMPCfg.ADCPgaGain;
  
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  memset(&dsp_cfg.DftCfg, 0, sizeof(dsp_cfg.DftCfg));
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppAMPCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppAMPCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);
  
  sw_cfg.Dswitch = 0;
  sw_cfg.Pswitch = 0;
  sw_cfg.Nswitch = 0;
  sw_cfg.Tswitch = 0; 
  AD5940_SWMatrixCfgS(&sw_cfg);
    
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_AFECtrlS(AFECTRL_SINC2NOTCH, bFALSE);
  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    AppAMPCfg.InitSeqInfo.SeqId = SEQID_1;
    AppAMPCfg.InitSeqInfo.SeqRamAddr = AppAMPCfg.SeqStartAddr;
    AppAMPCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppAMPCfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppAMPCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppAMPSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataType = DATATYPE_NOTCH;
  clks_cal.ADCRate = ADCRATE_800KHZ; /* ONLY used when data type is DATATYPE_NOTCH */
  clks_cal.DataCount = 1;
  clks_cal.ADCSinc2Osr = AppAMPCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppAMPCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppAMPCfg.SysClkFreq/AppAMPCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
	WaitClks += 15;
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));    /* wait 250us */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);   /* Start ADC convert*/
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE);  /* Stop ADC */
  AD5940_SEQGpioCtrlS(0);
  AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  if(error == AD5940ERR_OK)
  {
    AppAMPCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppAMPCfg.MeasureSeqInfo.SeqRamAddr = AppAMPCfg.InitSeqInfo.SeqRamAddr + AppAMPCfg.InitSeqInfo.SeqLen ;
    AppAMPCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppAMPCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppAMPCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}
static AD5940Err AppAMPRtiaCal(void)
{
fImpPol_Type RtiaCalValue;  /* Calibration result */
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

  lprtia_cal.bPolarResult = bTRUE;                /* Magnitude + Phase */
  lprtia_cal.AdcClkFreq = AppAMPCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppAMPCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;        /* Use SINC2 data as DFT data source */
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;         /* Maximum DFT number */
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;   /* For frequency under 12Hz, need to optimize DFT source. Use SINC3 data as DFT source */
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppAMPCfg.AdcClkFreq/4/22/2048*3;  /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
  lprtia_cal.fRcal = AppAMPCfg.RcalVal;
  lprtia_cal.LpTiaRtia = AppAMPCfg.LptiaRtiaSel;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppAMPCfg.RtiaCalValue = RtiaCalValue;

  printf("Rcal.Magnitude = %.1f ohm\n", AppAMPCfg.RtiaCalValue.Magnitude); /* DEBUG */
 
  return AD5940ERR_OK;
}
/* This function provide application initialize.   */
AD5940Err AppAMPInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Do RTIA calibration */
  if(((AppAMPCfg.ReDoRtiaCal == bTRUE) || \
      AppAMPCfg.AMPInited == bFALSE) && AppAMPCfg.ExtRtia == bFALSE)  /* Do calibration on the first initializaion */
  {
    AppAMPRtiaCal();
    AppAMPCfg.ReDoRtiaCal = bFALSE;
  }else
		AppAMPCfg.RtiaCalValue.Magnitude = AppAMPCfg.ExtRtiaVal;
  
	/* Reconfigure FIFO */
  AD5940_FIFOCtrlS(DFTSRC_SINC3, bFALSE);									/* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = AppAMPCfg.DataFifoSrc;
  fifo_cfg.FIFOThresh = AppAMPCfg.FifoThresh;              
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppAMPCfg.AMPInited == bFALSE)||\
       (AppAMPCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* Generate initialize sequence */
    error = AppAMPSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;

    /* Generate measurement sequence */
    error = AppAMPSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppAMPCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }
  /* Initialization sequencer  */
  AppAMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppAMPCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppAMPCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  
  /* Measurement sequence  */
  AppAMPCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppAMPCfg.MeasureSeqInfo);

//  seq_cfg.SeqEnable = bTRUE;
//  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
	AD5940_SEQCtrlS(bTRUE);  /* Enable sequencer, and wait for trigger. It's disabled in initialization sequence */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */

  AD5940_AFEPwrBW(AppAMPCfg.PwrMod, AFEBW_250KHZ);
  AppAMPCfg.AMPInited = bTRUE;  /* AMP application has been initialized. */
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppAMPRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppAMPCfg.NumOfData > 0)
  {
    AppAMPCfg.FifoDataCount += *pDataCount/4;
    if(AppAMPCfg.FifoDataCount >= AppAMPCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppAMPCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppAMPDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i, datacount;
  datacount = *pDataCount;
  float *pOut = (float *)pData;
  for(i=0;i<datacount;i++)
  {
    pData[i] &= 0xffff;
    pOut[i] = AppAMPCalcCurrent(pData[i]);
  }
  return AD5940ERR_OK;
}

/**

*/
AD5940Err AppAMPISR(void *pBuff, uint32_t *pCount)
{
  uint32_t FifoCnt;
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
	
  *pCount = 0;  
  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    //AppAMPRegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state - NOTE: FIFO re-config may break FIFO INTs, see GitHub issue (!) */
		AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); 
    //AD5940_EnterSleepS();  /* Manually put AFE back to hibernate mode. This operation only takes effect when register value is ACTIVE previously */

    /* Process data */ 
    AppAMPDataProcess((int32_t*)pBuff,&FifoCnt); 
    *pCount = FifoCnt;
    return 0;
  }
  
  return 0;
} 

/* Calculate voltage */
float AppAMPCalcVoltage(uint32_t ADCcode)
{
  float kFactor = 1.835/1.82;
  float fVolt = 0.0;
  int32_t tmp = 0;
  tmp = ADCcode - 32768;
  switch(AppAMPCfg.ADCPgaGain)
  {
    case ADCPGA_1:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/1)*kFactor;
      break;
    case ADCPGA_1P5:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/1.5f)*kFactor;
      break;
    case ADCPGA_2:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/2)*kFactor;
      break;
    case ADCPGA_4:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/4)*kFactor;
      break;
    case ADCPGA_9:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/9)*kFactor;
      break;
  } 
  return fVolt;
}
/* Calculate current in uA */
float AppAMPCalcCurrent(uint32_t ADCcode)
{
  float fCurrent, fVoltage = 0.0;
  fVoltage = AppAMPCalcVoltage(ADCcode);
  fCurrent = fVoltage/AppAMPCfg.RtiaCalValue.Magnitude;
  
  // printf("%.4f V\n", fVoltage); /* DEBUG */
  
  return -fCurrent*1000000; // "-" beacuse the Vout of TIA must generate a I to counter input I, thus reverse sign with respect to it
}

#endif // AMPEROMETRIC_C

/**
 * @}
 * @}
 * */
