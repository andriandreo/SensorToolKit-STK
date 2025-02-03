/*!
*****************************************************************************
@file:    AD5940_ADCSeqMUX.c
@author:  andriandreo based on Neo Xu's `AD5940_Temperature.c`
@brief:   AD5940 ADC readings example with sequencer support.
-----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "ADCSequencer.h"

/**
 * This example shows how to configure ADC and using sequencer and FIFO
 * to take voltage measurements.
 * Notch filter enabled is implemented in this example for best results. 
 * See comments for available alternatives (bypassed notch). 
*/

#define MEASURE_FREQ  	8.0f          // 8Hz (8SPS)
#define CH_NUMBER       8             // Number of IN differential channels

#define BUFF_SIZE 128
#define ADC_BUFF_SIZE 256

// This buffer will be used by sequence generator
uint32_t buff[BUFF_SIZE];
// This buffer will be used to store result from AD594x
uint32_t ADCbuff[ADC_BUFF_SIZE];
uint32_t data_count = 0;  // The ADC data count in buffer.

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters
*/
AppADCSeqCfg_Type AppADCSeqCfg = 
{
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,

  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .ODR = 1, // Output Data Rate (Hz)

  .PwrMod = AFEPWR_LP,
  .ADCPgaGain = ADCPGA_1P5,
  .ADCSinc3Osr = ADCSINC3OSR_2, /* See Table 41 of AD594x Datasheet for available settings, `ADCSINC3OSR_4` not valid when Notch enabled */
  .ADCSinc2Osr = ADCSINC2OSR_1333,
  .ADCRate = ADCRATE_800KHZ,

  .FifoSrc = FIFOSRC_SINC2NOTCH,
  .DataFifoSrc = DATATYPE_NOTCH,
  .FifoThresh = CH_NUMBER, // Generate FIFO threshold interrupt depending on the IN channel number
  .NumOfData = -1,
};

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
  pgacal.ADCPga = ADCPGA_1P5;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void){
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();

  AD5940_PGA_Calibration();

  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = AppADCSeqCfg.ADCRate==ADCRATE_800KHZ?SYSCLKDIV_1:SYSCLKDIV_2;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = AppADCSeqCfg.ADCRate==ADCRATE_800KHZ?bFALSE:bTRUE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                      /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = AppADCSeqCfg.FifoSrc;
  fifo_cfg.FIFOThresh = AppADCSeqCfg.FifoThresh;
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
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = AGPIO_Pin2;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Enable AFE to enter sleep mode. */
  return 0;
}

void _ad5940_analog_init(void){
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;

  /* Configure AFE power mode and bandwidth */
  //AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ); // ONLY needed in this example for DAC (`VREF1P8DAC`)

  // Init ad5940 for ADC measurement.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_VRE0;
  adc_base.ADCMuxN = ADCMUXN_AIN3;
  adc_base.ADCPga = ADCPGA_1P5;
  AD5940_ADCBaseCfgS(&adc_base);
  //AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE);
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = AppADCSeqCfg.ADCSinc3Osr;
  adc_filter.ADCSinc2Osr = AppADCSeqCfg.ADCSinc2Osr;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = AppADCSeqCfg.ADCRate;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bFALSE;                /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. - HERE USED */
  //adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. - HERE BYPASSED */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);   /* Turn on ADC power */
}

/**
 * @brief Init everything we need to measure voltage with ADC.
 */
void AD5940_adcInit(void){
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t seq_len;
  WUPTCfg_Type wupt_cfg;
  ClksCalInfo_Type clks_cal;
  uint32_t WaitClks;
  //clks_cal.DataType = DATATYPE_SINC2;
  clks_cal.DataType = DATATYPE_NOTCH;
  clks_cal.ADCRate = AppADCSeqCfg.ADCRate; /* ONLY used when data type is DATATYPE_NOTCH */
  clks_cal.DataCount = 1; /* Sample one data when wakeup - Set >1 for Continuous Stream, BE AWARE OF DATA|>FREQ. (?) & LIMIT TO BUFFER SIZE (`BUFF_SIZE`) [!!!] */
  clks_cal.ADCSinc3Osr = AppADCSeqCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = AppADCSeqCfg.ADCSinc2Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppADCSeqCfg.SysClkFreq/AppADCSeqCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  _ad5940_analog_init();
  // Generate sequence to measure ADC output
  AD5940_SEQGenInit(buff, BUFF_SIZE); // Init sequence generator
  AD5940_SEQGenCtrl(bTRUE); // From now on, record all register operations rather than write them to AD5940 through SPI.

  AD5940_SEQGpioCtrlS(AGPIO_Pin1);        /* Pull high AGPIO1 so we know the sequencer is running by observing pin status with oscilloscope etc. */
  
  /* CHANNEL 1 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_AIN2, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */

  /* CHANNEL 2 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_AIN1, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  
  /* CHANNEL 3 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_AIN0, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  
  /* CHANNEL 4 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_AIN4, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  
  /* CHANNEL 5 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_VSE0, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  
  /* CHANNEL 6 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_VDE0, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  
  /* CHANNEL 7 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_VCE0, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  
  /* CHANNEL 8 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling (if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_ADCMuxCfgS(ADCMUXP_VRE0, ADCMUXN_AIN3);
  //AD5940_ADCMuxCfgS(ADCMUXP_VREF1P8DAC, ADCMUXN_VSET1P1); // DAC Vref
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait another 50µs for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  // Wait for first data ready
  AD5940_AFECtrlS(/*AFECTRL_ADCCNV|*/AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC - Uncomment `AFECTRL_ADCCNV` for continuous data stream (!) */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */

  AD5940_SEQGpioCtrlS(0);                 /* Pull low AGPIO so we know end of sequence. */
  AD5940_EnterSleepS();                   /* Goto hibernate */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &seq_len);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error != AD5940ERR_OK){
    puts("Sequence generator error!");
  }
  
  AppADCSeqCfg.seqinfo0.pSeqCmd = pSeqCmd;
  AppADCSeqCfg.seqinfo0.SeqId = SEQID_0;   // Use SEQ0 to run this sequence
  AppADCSeqCfg.seqinfo0.SeqLen = seq_len;
  AppADCSeqCfg.seqinfo0.SeqRamAddr = AppADCSeqCfg.SeqStartAddr;    // Place this sequence from start of SRAM.
  AppADCSeqCfg.seqinfo0.WriteSRAM = bTRUE; // We need to write this sequence to AD5940 SRAM.
  AD5940_SEQInfoCfg(&AppADCSeqCfg.seqinfo0);
  
  // Now configure wakeup timer to trigger above sequence periodically to get ADC data.
  wupt_cfg.WuptEn = bFALSE; // Do not start it right now.
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  //wupt_cfg.SeqxSleepTime[SEQID_0] = 4-1;
  //wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(32e3f/MEASURE_FREQ)-4-1;
  wupt_cfg.SeqxSleepTime[SEQID_0] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppADCSeqCfg.WuptClkFreq/AppADCSeqCfg.ODR)-2-1;
  AD5940_WUPTCfg(&wupt_cfg);
  // Enable sequencer
  AD5940_SEQCtrlS(bTRUE); // Now sequencer is ready to be triggered.
}

void AD5940_adcISR(void){
  // Process data from AD5940 FIFO.
  uint32_t FifoCnt, IntcFlag;
  if(AD5940_WakeUp(10) > 10){  /* Wakeup AFE by read register, read 10 times at most */
    printf("Failed to wakeup AD5940!\n");
    return;
  }
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* We need time to read data from FIFO, so, do not let AD5940 goes to hibernate automatically */
  IntcFlag = AD5940_INTCGetFlag(AFEINTC_0);
  //printf("IntcFlag = 0x%08lx\n", IntcFlag); /* DEBUG */
  if(IntcFlag&AFEINTSRC_DATAFIFOTHRESH){
    FifoCnt = AD5940_FIFOGetCnt();
    FifoCnt = FifoCnt>ADC_BUFF_SIZE?ADC_BUFF_SIZE:FifoCnt;
    data_count = FifoCnt;
    AD5940_FIFORd(ADCbuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);    /* Allow AFE to enter sleep mode. AFE will stay at active mode until sequencer trigger sleep */
    AD5940_EnterSleepS();	// If MCU is too slow, comment this line, otherwise there is chance the sequencer is running at this point.
  }
}

void AD5940_PrintResult(void){
  for(int i=0; i<data_count; i++){
    uint32_t data = ADCbuff[i]&0xffff; // `int` (original) vs. `uint` (ADCPolling)? [!!!] 
    //printf("CH[%d] = %ld, %.4f V\n", i+1, data, AD5940_ADCCode2Volt(data, ADCPGA_1P5, 1.82));
    printf("%.4f, ", AD5940_ADCCode2Volt(data, ADCPGA_1P5, 1.82)); // CSV - Row (Values)
  }
}

void AD5940_Main(void){
  printf("CH[1], CH[2], CH[3], CH[4], CH[5], CH[6], CH[7], CH[8]\n"); // CSV - Heading
  AD5940PlatformCfg();
  AD5940_adcInit();
  AD5940_WUPTCtrl(bTRUE); // Start WUPT, so the sequence will be run periodically.
  while(1){
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(AD5940_GetMCUIntFlag()){
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      AD5940_adcISR();
      AD5940_PrintResult();
      printf("\n"); // CSV - New Line
    }
  }
}

