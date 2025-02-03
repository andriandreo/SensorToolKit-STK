/*!
 *****************************************************************************
 @file:    AD5940_ADCMultiplexer.c
 @author:  Andrés Alberto Andreo Acosta
 @brief:   ADC Sequencer Multiplexing
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
// Include guard
#ifndef AD5940_ADCMultiplexer_C
#define AD5940_ADCMultiplexer_C

#include "ADCSequencer.h"

int32_t SeqISR(void);
BoolFlag bSeqEnd = bFALSE;

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters
*/
AppADCSeqCfg_Type AppADCSeqCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,

  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .ODR = 1,
  .NumOfData = -1,

  .PwrMod = AFEPWR_LP,
  .HstiaRtiaSel = HSTIARTIA_1K,

  .ADCPgaGain = ADCPGA_1P5,
  .ADCSinc3Osr = ADCSINC3OSR_2,
  .ADCSinc2Osr = ADCSINC2OSR_1333,

  .FifoThresh = 4,
  .ADCSeqInited = bFALSE,
  .StopRequired = bFALSE,
  .MeasSeqCycleCount = 0,
};

#define ADCPGA_GAIN_SEL   ADCPGA_1P5
static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
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

static int32_t AD5940_PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize(); /* Call this right after AFE reset */

  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO (and Sequencer)*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;
  fifo_cfg.FIFOSrc = FIFOSRC_MEAN;
  fifo_cfg.FIFOThresh = 2;
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0|AFEINTSRC_CUSTOMINT1|AFEINTSRC_CUSTOMINT2|AFEINTSRC_CUSTOMINT3, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_ClrMCUIntFlag(); /* Clear the MCU interrupt flag which will be set in ISR. */
  /* Step4: Reconfigure GPIO */
  /*  GP0: the interrupt output.
      GP1: normal GPIO
      GP2: not used
      GP3: controlled by sequencer.
      Others (AD5940 only!): not used. The default function is mode0.
   */
  gpio_cfg.FuncSet = GP0_INT|GP1_GPIO|GP3_SYNC;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  
  return 0;
}

/* Generate MUX sequence */
static AD5940Err AppADCSeqGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataType = DATATYPE_NOTCH;
  clks_cal.ADCRate = ADCRATE_800KHZ; /* ONLY used when data type is DATATYPE_NOTCH */
  clks_cal.DataCount = 128; /* STATSAMPLE_128 */
  clks_cal.ADCSinc3Osr = AppADCSeqCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = AppADCSeqCfg.ADCSinc2Osr;
  clks_cal.RatioSys2AdcClk = AppADCSeqCfg.SysClkFreq/AppADCSeqCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // SEQ0 GENERATOR: 1st pair of AINx | MUX
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_ADCMuxCfgS(ADCMUXP_VRE0, ADCMUXN_AIN0);

  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250)); // Wait 250us
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); // Start ADC convert
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); // Wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE); // Stop ADC

  AD5940_SEQGpioCtrlS(0);
  //AD5940_EnterSleepS(); //[!!!]

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_INT0());   /* Generate custom-interrupt 0. We can generate any custom interrupt(SEQ_INT0/1/2/3()) by sequencer. */
  //AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time - For init SEQ. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    /* Write command to SRAM */
    AppADCSeqCfg.seqinfo0.pSeqCmd = pSeqCmd;
    AppADCSeqCfg.seqinfo0.SeqId = SEQID_0;
    AppADCSeqCfg.seqinfo0.SeqLen = SeqLen;
    AppADCSeqCfg.seqinfo0.SeqRamAddr = AppADCSeqCfg.SeqStartAddr;
    AppADCSeqCfg.seqinfo0.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppADCSeqCfg.seqinfo0); /* Configure sequence0 info and write commands to SRAM */
  }
  else
    return error; /* Error */
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // SEQ1 GENERATOR: 2nd pair of AINx | MUX
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_ADCMuxCfgS(ADCMUXP_VRE0, ADCMUXN_AIN1);

  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250)); // Wait 250us
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); // Start ADC convert
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); // Wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE); // Stop ADC

  AD5940_SEQGpioCtrlS(0);
  //AD5940_EnterSleepS(); //[!!!]

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_INT1());   /* Generate custom-interrupt 0. We can generate any custom interrupt(SEQ_INT0/1/2/3()) by sequencer. */
  //AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time - For init SEQ. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    /* Write command to SRAM */
    AppADCSeqCfg.seqinfo1.pSeqCmd = pSeqCmd;
    AppADCSeqCfg.seqinfo1.SeqId = SEQID_1;
    AppADCSeqCfg.seqinfo1.SeqLen = SeqLen;
    AppADCSeqCfg.seqinfo1.SeqRamAddr = AppADCSeqCfg.seqinfo0.SeqRamAddr + AppADCSeqCfg.seqinfo0.SeqLen;
    AppADCSeqCfg.seqinfo1.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppADCSeqCfg.seqinfo1); /* Configure sequence0 info and write commands to SRAM */
  }
  else
    return error; /* Error */
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // SEQ2 GENERATOR: 3rd pair of AINx | MUX
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_ADCMuxCfgS(ADCMUXP_VRE0, ADCMUXN_AIN2);

  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250)); // Wait 250us
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); // Start ADC convert
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); // Wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE); // Stop ADC

  AD5940_SEQGpioCtrlS(0);
  //AD5940_EnterSleepS(); //[!!!]

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_INT2());   /* Generate custom-interrupt 0. We can generate any custom interrupt(SEQ_INT0/1/2/3()) by sequencer. */
  //AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time - For init SEQ. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    /* Write command to SRAM */
    AppADCSeqCfg.seqinfo2.pSeqCmd = pSeqCmd;
    AppADCSeqCfg.seqinfo2.SeqId = SEQID_2;
    AppADCSeqCfg.seqinfo2.SeqLen = SeqLen;
    AppADCSeqCfg.seqinfo2.SeqRamAddr = AppADCSeqCfg.seqinfo1.SeqRamAddr + AppADCSeqCfg.seqinfo1.SeqLen;
    AppADCSeqCfg.seqinfo2.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppADCSeqCfg.seqinfo2); /* Configure sequence0 info and write commands to SRAM */
  }
  else
    return error; /* Error */
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // SEQ3 GENERATOR: 4th pair of AINx | MUX
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_ADCMuxCfgS(ADCMUXP_VRE0, ADCMUXN_AIN3);

  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250)); // Wait 250us
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); // Start ADC convert
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); // Wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE); // Stop ADC

  AD5940_SEQGpioCtrlS(0);
  //AD5940_EnterSleepS(); //[!!!]

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_INT2()); /* Generate custom-interrupt 0. We can generate any custom interrupt(SEQ_INT0/1/2/3()) by sequencer. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Disable sequencer */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    /* Write command to SRAM */
    AppADCSeqCfg.seqinfo3.pSeqCmd = pSeqCmd;
    AppADCSeqCfg.seqinfo3.SeqId = SEQID_3;
    AppADCSeqCfg.seqinfo3.SeqLen = SeqLen;
    AppADCSeqCfg.seqinfo3.SeqRamAddr = AppADCSeqCfg.seqinfo2.SeqRamAddr + AppADCSeqCfg.seqinfo2.SeqLen;
    AppADCSeqCfg.seqinfo3.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppADCSeqCfg.seqinfo3); /* Configure sequence0 info and write commands to SRAM */
  }
  else
    return error; /* Error */
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------

  return AD5940ERR_OK;
}

void AD5940_Main(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  StatCfg_Type stat_cfg;
  SEQCfg_Type seq_cfg;
  WUPTCfg_Type wupt_cfg;
  
  AD5940_PlatformCfg();

  AD5940_PGA_Calibration();
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_VRE0; // RE0 (RE) [!!!]
  adc_base.ADCMuxN = ADCMUXN_AIN0; // TODO W/ LP-TIA INIT: SE0 (WE) [!!!]
  adc_base.ADCPga = ADCPGA_GAIN_SEL;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  //adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;     /* Setting not valid when Notch is working simoultaneously */
  adc_filter.ADCSinc3Osr = AppADCSeqCfg.ADCSinc3Osr;     /* See Table 41 of AD594x Datasheet for available settings */
  adc_filter.ADCSinc2Osr = AppADCSeqCfg.ADCSinc2Osr;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  //adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. HERE BYPASSED (!) */
  adc_filter.BpNotch = bFALSE;                /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. HERE USED (!) */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS. SEE BELOW (!) */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  //AD5940_ADCMuxCfgS(ADCMUXP_AIN2, ADCMUXN_VSET1P1);   /* Optionally, you can change ADC MUX with this function */

  /**
   * Statistic block receive data from SINC2+Notch block. Note the diagram in datasheet page 51 PrM. 
   * The SINC3 can be bypassed optionally. SINC2 cannot be bypassed.
   * */
  stat_cfg.StatDev = STATDEV_1;               /* Not used. */
  stat_cfg.StatEnable = bTRUE;
  stat_cfg.StatSample = STATSAMPLE_128;       /* Sample 128 points and calculate mean. */
  AD5940_StatisticCfgS(&stat_cfg);

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bTRUE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  //AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  //AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  //AD5940_ADCPowerCtrlS(bTRUE);
  //AD5940_ADCConvtCtrlS(bTRUE);

  /* Initialize SEQ generator helper routines */
  AppADCSeqGen();

  /* Configure wakeup timer */
  wupt_cfg.WuptEn = bFALSE;     /* Don't start it right now. */
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_D; /* A->B->C->D->A->B->C->D */
  wupt_cfg.WuptOrder[0] = SEQID_0;    /* Put SEQ0 to slot A */
  wupt_cfg.WuptOrder[1] = SEQID_1;    /* Put SEQ1 to slot B */
  wupt_cfg.WuptOrder[2] = SEQID_2;    /* Put SEQ2 to slot C */
  wupt_cfg.WuptOrder[3] = SEQID_3;    /* Put SEQ3 to slot D */
  /* There is no need to init up to slot DEFGH, that's WuptOrder[4] to WuptOrder[7], becaue we don't use it. EndofSeq is D.*/
  wupt_cfg.SeqxWakeupTime[SEQID_0] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. After, wakeup and trigger seq1 */
  wupt_cfg.SeqxWakeupTime[SEQID_1] = 1;
  wupt_cfg.SeqxWakeupTime[SEQID_2] = 1;
  //wupt_cfg.SeqxSleepTime[SEQID_3] = (uint32_t)(AppADCSeqCfg.WuptClkFreq/AppADCSeqCfg.ODR)-2-1; // ADD STH.? NEEDED? - "-2-1" | `SeqxWakeupTime` comment? [!!!]
  wupt_cfg.SeqxWakeupTime[SEQID_3] = 1;
  AD5940_WUPTCfg(&wupt_cfg);
  
  AD5940_WUPTCtrl(bTRUE); /* Enable wakeup timer. */
  while(1)
  {
    while(1)
    {
      if(AD5940_GetMCUIntFlag())
      {
        AD5940_ClrMCUIntFlag();
        SeqISR();
        if(bSeqEnd)
          break;
      }
    }
    AD5940_WUPTCtrl(bFALSE);  /* Wakeup timer is still running and triggering. Trigger is not accepted because sequencer 
                              is disabled in last sequence(SEQ1) command. */
    AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer again, because we disabled it in seq3 last command. */
  }
}

int32_t SeqISR(void)
{
  uint32_t IntFlag, FifoCnt, ADCBuff[512], tempMean = 0.0;
  float diff_volt  = 0.0, dV0 = 0.0, dV1 = 0.0, dV2 = 0.0, dV3 = 0.0;

  IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
  
  if(IntFlag & AFEINTSRC_CUSTOMINT0 || IntFlag & AFEINTSRC_CUSTOMINT1 || IntFlag & AFEINTSRC_CUSTOMINT2 || IntFlag & AFEINTSRC_CUSTOMINT3)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    AD5940_FIFORd((uint32_t *)ADCBuff, FifoCnt);
    //AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    for (int i = 0; i < FifoCnt; i++)
    {
      tempMean += ADCBuff[i] & 0xffff;
    }
    tempMean /= FifoCnt;
    diff_volt = AD5940_ADCCode2Volt((uint32_t)tempMean, ADCPGA_GAIN_SEL, 1.82);
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT0)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
    //printf("Custom INT0!\n"); // DEBUG
    dV0 = diff_volt;
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT1)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT1);
    //printf("Custom INT1!\n"); // DEBUG
    dV1 = diff_volt;
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT2)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT2);
    //printf("Custom INT2!\n"); // DEBUG
    dV2 = diff_volt;
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT3)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT3);
    //printf("Custom INT3!\n"); // DEBUG
    dV3 = diff_volt;
  }
  if(IntFlag & AFEINTSRC_ENDSEQ)  /* This interrupt is generated when Sequencer is disabled. */
  {
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    //printf("End of Sequence\n"); // DEBUG
    printf("dV0: %.4f, dV1: %.4f, dV2: %.4f, dV3: %.4f\n", dV0, dV1, dV2, dV3);
    bSeqEnd = bTRUE;
  }
	return AD5940ERR_OK;
}

#endif // AD5940_ADCMultiplexer_C
