#include "sensors/EChem_BioZ.h"
#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "power/heater_task.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h" // <-- ADD THIS LINE
#include "util/debug_log.h"
#include <cmath>

using namespace sensor;

// Structure for how parameters are passed down from the host
struct BIOZ_INIT_PARAMETERS {
  float coilFrequency;      // [Hz]
  float speFrequency;       // [Hz]
} __attribute__((packed));

// Struct 2: Sent continuously during the 8-second loop (6 bytes)
struct BIOZ_MEAS_PARAMETERS {
  uint8_t target_mux;       // 0x00=SPE1, 0x01=SPE2, 0xFF=Both Coils
  uint8_t num_averages;     // 10 for Coils, 1 for SPE
  float Eac;                // [mV]
} __attribute__((packed));

EChem_BioZ::EChem_BioZ() {
  // Initialize structures to known values
  memset(&config, 0, sizeof(BioZConfig_Type));
  config.bParaChanged = bFALSE; // Flag used to indicate parameters have been set
  config.SeqStartAddr = 0;

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;
  config.SamplingInterval = 30.0f; // Default value 30s between samples

  config.RcalVal = 10000.0;     // 10kOhm on Biocoin
  // Switch/pin config settings
  config.DswitchSel = SWD_CE0;  // positive force pin for Impedance measurement
  config.PswitchSel = SWP_CE0;  // positive force pin for Impedance measurement
  config.NswitchSel = SWN_AIN1; // negative force pin for Impedance measurement
  config.TswitchSel = SWT_AIN1; // negative force pin for Impedance measurement
  config.SenseP = ADCMUXP_AIN3; // positive sense pin for Impedance measurement, don't care if using 2-wire
  config.SenseN = ADCMUXN_AIN2; // negative sense pin for Impedance measurement, don't care if using 2-wire

  config.PwrMod = AFEPWR_HP;
  config.AFEBW = AFEBW_250KHZ;
  config.ADCPgaGain = ADCPGA_1P5; /*Gain = 1.5V/V is the factory calibrated most accurate gain setting*/
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_22; // adjust these as needed if really fast or really slow sampling is required.
                                       // Power vs. SNR tradeoff.
  config.HstiaRtiaSel = HSTIARTIA_200;

  config.CtiaSel = 32;
  config.ExcitBufGain = EXCITBUFGAIN_2;
  config.HsDacGain = HSDACGAIN_1;
  config.HsDacUpdateRate = 7;
  config.DacVoltPP = 800.0;
  config.Eac = 9.0;
  config.SinFreq = 47100;
  config.FifoThresh = 4;
  config.IMP4WIRE = bTRUE;
  config.ACcoupled = bFALSE;

  config.DftNum = DFTNUM_8192;
  config.DftSrc = DFTSRC_SINC3;
  config.HanWinEn = bTRUE;

  config.SweepCfg.SweepEn = bFALSE;
  config.SweepCfg.SweepStart = 1000;
  config.SweepCfg.SweepStop = 100000.0;
  config.SweepCfg.SweepPoints = 101;
  config.SweepCfg.SweepLog = bTRUE;
  config.SweepCfg.SweepIndex = 0;

  config.StopRequired = bFALSE;

  config.MeasSeqCycleCount = 0;
}

bool EChem_BioZ::loadParameters(uint8_t* data, uint16_t len) {
  
  // --- Parse Initialization Parameters (8 bytes) ---
  if (len == sizeof(BIOZ_INIT_PARAMETERS)) {
    BIOZ_INIT_PARAMETERS initParams;
    memcpy(&initParams, data, len);
    //hardcoded
    config.coilFrequency = initParams.coilFrequency;
    config.speFrequency = initParams.speFrequency;

    dbgInfo("Init Params Loaded: Coil=" + String(config.coilFrequency) + "Hz, SPE=" + String(config.speFrequency) + "Hz");
    return true;
  } 
  
  // --- Parse Measurement Parameters (6 bytes) ---
  else if (len == sizeof(BIOZ_MEAS_PARAMETERS)) {
    BIOZ_MEAS_PARAMETERS measParams;
    memcpy(&measParams, data, len);

    config.target_mux = measParams.target_mux;
    config.num_averages = measParams.num_averages;
    config.Eac = measParams.Eac;
    dbgInfo("Measurement Params Received: MUX=" + String(config.target_mux) + ", Averages=" + String(config.num_averages) + ", Eac=" + String(config.Eac) + "mV");
    config.DacVoltPP = config.Eac;  
    
    config.bParaChanged = bTRUE; // Flag that we are ready to measure
    dbgInfo("Meas Params Loaded: MUX=" + String(config.target_mux) + ", Averages=" + String(config.num_averages) + ", Eac=" + String(config.Eac) + "mV");
    return true;
  } 
  
  // --- Error Handling ---
  else {
    dbgError("Incorrect parameter payload size! Received " + String(len) + " bytes.");
    return false;
  }
}




bool EChem_BioZ::globalStart() {
  dbgInfo("Global Start: Initializing and Calibrating...");
  if (config.bParaChanged != bTRUE) return false;

  clear();
  power::powerOnAFE(0);
  Start_AD5940_SPI();
  initAD5940();
  configureWaveformParameters();
  
  setupMeasurement(); // Note: Removed WUPT configuration from here!
  
  if (AD5940_WakeUp(10) > 10) return false;

  //AD5940_EnterSleepS();
  //Stop_AD5940_SPI();
  setRunning();
  return true;
}

bool EChem_BioZ::start() {
  if (!isRunning()) return false;
  
  //Start_AD5940_SPI();
  if (AD5940_WakeUp(10) > 10) return false;

  uint32_t ampWord = (uint32_t)(config.Eac / 800.0f * 2047 + 0.5f);
  AD5940_WriteReg(REG_AFE_WGAMPLITUDE, ampWord);
  if (config.target_mux == 0xFF) {
    // ==========================================
    // COMBINED COIL MEASUREMENT (0xFF)
    // ==========================================
    
    // 1. Safety Interlock: Suspend Heating
    power::suspendHeating();
    while (!power::isHeaterOff()) {
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    // 2. Setup AFE for Coils
    AD5940_WGFreqCtrlS(config.coilFrequency, config.SysClkFreq);
    config.SinFreq = config.coilFrequency;
    // config.RtiaCurrValue[0] = config.DualRtiaCal[0][0];
    // config.RtiaCurrValue[1] = config.DualRtiaCal[0][1];

    // 3. Measure Coil 1
    power::setBioZMux(0b10); // Coil 1
    vTaskDelay(pdMS_TO_TICKS(20)); // MUX settling time
    fImpPol_Type coil1_result = takeAveragedMeasurement(config.num_averages);
    push(coil1_result);
    Serial.printf("Channel: 2, Mag: %.5f\n", coil1_result.Magnitude);
    Serial.flush();

    // 4. Measure Coil 2
    power::setBioZMux(0b11); // Coil 2
    vTaskDelay(pdMS_TO_TICKS(20)); 
    fImpPol_Type coil2_result = takeAveragedMeasurement(config.num_averages);
    push(coil2_result);
    Serial.printf("Channel: 3, Mag: %.5f\n", coil2_result.Magnitude);
    Serial.flush();

    // 5. Restore Safety and Heaters
    power::setBioZMux(0b00); // Route MUX away from coils safely
    vTaskDelay(pdMS_TO_TICKS(5)); 
    power::resumeHeating();  // Re-engages PWM instantly

  } else {
    // ==========================================
    // STANDARD SPE MEASUREMENT
    // ==========================================
    power::setBioZMux(config.target_mux);
    AD5940_WGFreqCtrlS(config.speFrequency, config.SysClkFreq);
    config.SinFreq = config.speFrequency;
    // config.RtiaCurrValue[0] = config.DualRtiaCal[1][0];
    // config.RtiaCurrValue[1] = config.DualRtiaCal[1][1];
    
    vTaskDelay(pdMS_TO_TICKS(30)); 

    fImpPol_Type spe_result = takeAveragedMeasurement(config.num_averages);
    push(spe_result);
    if (config.target_mux == 0x00) {
      Serial.printf("Channel: SPE1, Mag: %.5f\n", spe_result.Magnitude);
    } else if (config.target_mux == 0x01) {
      Serial.printf("Channel: SPE2, Mag: %.5f\n", spe_result.Magnitude);
    }
  }

  // // Go back to sleep and transmit data
  // AD5940_EnterSleepS();
  // Stop_AD5940_SPI();
  //queueDataForTX(0);

  return true;
}

bool EChem_BioZ::globalStop() {
  dbgInfo("Global stop");
  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return false;             /* Wakeup Failed */
  /* Start Wupt right now */
  AD5940_WUPTCtrl(bFALSE);
  /* There is chance this operation will fail because sequencer could put AFE back
    to hibernate mode just after waking up. Use STOPSYNC is better. */
  AD5940_WUPTCtrl(bFALSE);
  AD5940_ShutDownS();
  Stop_AD5940_SPI();             // Once the test has started, turn off SPI to reduce power
  power::powerOffPeripherials(); // Shut down the test
  setStopped();
  //power::resumeHeating();
  return true;
}

/* Initialize AD5940 basic blocks like clock */
int32_t EChem_BioZ::initAD5940(void) {
  AD5940_HWReset();                                // Hardware reset
  AD5940_Initialize();                             // Platform configuration
  AD5940_ConfigureClock();                         // Step 1 - Configure clock
  AD5940_ConfigureFIFO(FIFOSIZE_4KB, FIFOSRC_DFT); // Step 2 - Configure FIFO and Sequencer
  // AD5940_ConfigureSequencer(SEQMEMSIZE_2KB);
  AD5940_ConfigureInterrupts(AFEINTSRC_DATAFIFOTHRESH); // Step 3 - Configure interrupt controller
  AD5940_ConfigureGPIO();                               // Step 4 - Reconfigure GPIO
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);                  // Enable AFE to enter sleep mode
  AD5940_MeasureLFOSC(&LFOSCFreq);                      // Measure the LFOSC frequency

  return 0;
}

/**
 * @brief Initialize the amperometric test. Call this function every time before starting amperometric test.
 */
AD5940Err EChem_BioZ::setupMeasurement(void) {
  dbgInfo("Running");
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB; /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Do RTIA calibration */
  AD5940_CalibrateHSRTIA();

  /* Now Reconfigure FIFO after Rtia cal for CA measurements*/
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE); /* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = config.FifoThresh;
  dbgInfo("FIFO Threshold set to " + String(config.FifoThresh) + " bytes");
  AD5940_FIFOCfg(&fifo_cfg);
  /* Clear interrupts*/
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Generate sequences */
  if (seq_buffer == 0) return AD5940ERR_PARA;
  if (SEQ_BUFF_SIZE == 0) return AD5940ERR_PARA;
  AD5940_SEQGenInit(seq_buffer, SEQ_BUFF_SIZE);

  /* Generate initialize sequence */
  error = generateInitSequence(); /* Application initialization sequence using either MCU or sequencer */
  if (error != AD5940ERR_OK) return error;

  /* Generate measurement sequence */
  error = generateMeasSequence();
  if (error != AD5940ERR_OK) return error;

  /* Initialize sequences */
  config.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg); /* Enable sequencer */
  AD5940_SEQMmrTrig(config.InitSeqInfo.SeqId);
  while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  
  /* Measurement sequence  */
  config.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.MeasureSeqInfo);

  //configureFrequencySpecifics(config.FreqofData); // Configure frequency-specific settings like gain and bandwidth based on the starting frequency  
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg); /* Enable sequencer, and wait for trigger */
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
  
  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); // set to low power mode with desired BW
  AD5940_WriteReg(REG_AFE_SWMUX, 1 << 3);

  return AD5940ERR_OK;
}

AD5940Err EChem_BioZ::configureFrequencySpecifics(float freq) {
  ADCFilterCfg_Type filter_cfg;
  DFTCfg_Type dft_cfg;
  HSDACCfg_Type hsdac_cfg;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  FreqParams_Type freq_params;
  uint32_t SeqCmdBuff[2];
  uint32_t SRAMAddr = 0;

  freq_params = AD5940_GetFreqParameters(freq);

  if (freq_params.HighPwrMode == bTRUE) {
    hsdac_cfg.ExcitBufGain = config.ExcitBufGain;
    hsdac_cfg.HsDacGain = config.HsDacGain;
    hsdac_cfg.HsDacUpdateRate = 0x7; // Faster update rate for DAC
    AD5940_HSDacCfgS(&hsdac_cfg);

    filter_cfg.ADCRate = ADCRATE_1P6MHZ; // Faster ADC
    config.AdcClkFreq = 32000000.0;

    AD5940_HPModeEn(bTRUE); // Enable High Power Mode
  } else {
    hsdac_cfg.ExcitBufGain = config.ExcitBufGain;
    hsdac_cfg.HsDacGain = config.HsDacGain;
    hsdac_cfg.HsDacUpdateRate = 0x1B; // Slower update rate is fine
    AD5940_HSDacCfgS(&hsdac_cfg);

    filter_cfg.ADCRate = ADCRATE_800KHZ; // Standard ADC rate
    config.AdcClkFreq = 16000000.0;

    AD5940_HPModeEn(bFALSE); // Disable High Power Mode
  }

  filter_cfg.ADCAvgNum = ADCAVGNUM_16; 
  filter_cfg.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  filter_cfg.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  filter_cfg.BpSinc3 = bFALSE;
  filter_cfg.BpNotch = bTRUE;
  filter_cfg.Sinc2NotchEnable = bTRUE;
  
  dft_cfg.DftNum = freq_params.DftNum;
  dft_cfg.DftSrc = freq_params.DftSrc;
  dft_cfg.HanWinEn = config.HanWinEn;

  AD5940_ADCFilterCfgS(&filter_cfg);
  AD5940_DFTCfgS(&dft_cfg);


  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = freq_params.DftSrc;
  clks_cal.DataCount = 1L << (freq_params.DftNum + 2);
  clks_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  // dbgInfo("--- Freq Update ---");
  // dbgInfo("Target Freq: " + String(freq));
  // dbgInfo("SysClk: " + String(config.SysClkFreq) + " AdcClk: " + String(config.AdcClkFreq));
  // dbgInfo("Calculated WaitClks: " + String(WaitClks));

  if (WaitClks > 0x3FFFFFFF) {
      dbgInfo("WARNING: WaitClks overflow! Value > 0x3FFFFFFF");
  }

  SRAMAddr = config.MeasureSeqInfo.SeqRamAddr;

  // dbgInfo("Base SRAM Addr: " + String(SRAMAddr));
  // dbgInfo("Writing to offsets: " + String(SRAMAddr + 10) + " and " + String(SRAMAddr + 15));

  SeqCmdBuff[0] = SEQ_WAIT(WaitClks);
  AD5940_SEQCmdWrite(SRAMAddr + 10, SeqCmdBuff, 1); 
  AD5940_SEQCmdWrite(SRAMAddr + 16, SeqCmdBuff, 1); 
  
  return AD5940ERR_OK;
}

void sensor::EChem_BioZ::configureWaveformParameters(void) {
  // AFE mode settings
  if (config.SinFreq >= 20000.0)
    config.PwrMod = AFEPWR_HP;
  else
    config.PwrMod = AFEPWR_LP;
}

/* Generate init sequence for CA. This runs only one time. */
AD5940Err EChem_BioZ::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;
  float sinFreq;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator here
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Init all to disable state

  bool bLPDACandTIANeeded = config.IMP4WIRE && config.ACcoupled;
  AD5940_ConfigureAFEReferences(bLPDACandTIANeeded, bLPDACandTIANeeded, false, false);

  HSLoopCfg_Type hs_loop = {0};
  hs_loop.HsDacCfg.ExcitBufGain = config.ExcitBufGain;
  hs_loop.HsDacCfg.HsDacGain = config.HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = config.HsDacUpdateRate;

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = config.CtiaSel;
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = config.HstiaRtiaSel;

  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL | SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL | SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_TRTIA;

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
  if (config.SweepCfg.SweepEn == bTRUE) {
    config.SweepCfg.SweepIndex = 0;
    config.FreqofData = config.SweepCfg.SweepStart;
    config.SweepCurrFreq = config.SweepCfg.SweepStart;
    AD5940_SweepNext(&config.SweepCfg, &config.SweepNextFreq);
    sinFreq = config.SweepCurrFreq;
  } else {
    sinFreq = config.SinFreq;
    config.FreqofData = sinFreq;
  }
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sinFreq, config.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(config.DacVoltPP / 800.0f * 2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);

  if (bLPDACandTIANeeded) {
    LPLoopCfg_Type lp_loop = {0};

    lp_loop.LpDacCfg.LpdacSel = LPDAC0;
    lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lp_loop.LpDacCfg.LpDacSW = LPDACSW_VZERO2LPTIA;
    lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lp_loop.LpDacCfg.DataRst = bFALSE;
    lp_loop.LpDacCfg.PowerEn = bTRUE;
    lp_loop.LpDacCfg.DacData6Bit = (uint32_t)((1300 - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    lp_loop.LpDacCfg.DacData12Bit = (int32_t)(lp_loop.LpDacCfg.DacData6Bit * 64); // don't care, not using it

    lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
    lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_HALF;
    lp_loop.LpAmpCfg.LpPaPwrEn = bFALSE;
    lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_20K;
    lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(6) | LPTIASW(7) | LPTIASW(9);
    AD5940_LPLoopCfgS(&lp_loop);
  }

  DSPCfg_Type dsp_cfg = {0};
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = config.ADCPgaGain;

  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));

  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16; /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = config.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = config.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = config.DftNum;
  dsp_cfg.DftCfg.DftSrc = config.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = config.HanWinEn;

  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);

  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_HSTIAPWR | AFECTRL_INAMPPWR | AFECTRL_EXTBUFPWR | AFECTRL_WG |
                      AFECTRL_DACREFPWR | AFECTRL_HSDACPWR | AFECTRL_SINC2NOTCH, bTRUE);

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we
                                      only want it to run one time. */

  /* Stop here */
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen); // create sequence
  if (error == AD5940ERR_OK) {
    config.InitSeqInfo.SeqId = SEQID_1;
    config.InitSeqInfo.SeqRamAddr = config.SeqStartAddr;
    config.InitSeqInfo.pSeqCmd = pSeqCmd;
    config.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(config.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

/* Generate measurement sequence for CA. This runs indefinitely until test is ended. */
AD5940Err EChem_BioZ::generateMeasSequence(void) {
  dbgInfo("Measurement");
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  SWMatrixCfg_Type sw_cfg;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = config.DftSrc;
  clks_cal.DataCount = 1L << (config.DftNum + 2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = config.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = config.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_SEQGenInsert(SEQ_WAIT(16 * 250)); /* wait 250us */
  sw_cfg.Dswitch = config.DswitchSel;
  sw_cfg.Pswitch = config.PswitchSel;
  sw_cfg.Nswitch = config.NswitchSel;
  sw_cfg.Tswitch = config.TswitchSel | SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg);

  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCPWR, bTRUE); /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 50)); 
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE);                                /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));                                             /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, bFALSE); /* Stop ADC convert and DFT */
  
  if (config.IMP4WIRE)
    AD5940_ADCMuxCfgS(config.SenseP, config.SenseN);
  else
    AD5940_ADCMuxCfgS(ADCMUXP_VCE0, ADCMUXN_N_NODE);

  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 50));                // delay for signal settling DFT_WAIT
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE); /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));              /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, bFALSE); /* Stop ADC convert and DFT */

  sw_cfg.Dswitch = SWD_OPEN;
  sw_cfg.Pswitch = SWP_PL | SWP_PL2;
  sw_cfg.Nswitch = SWN_NL | SWN_NL2;
  sw_cfg.Tswitch = SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg); /* Float switches */
  // AD5940_EnterSleepS();         /* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  config.MeasSeqCycleCount = AD5940_SEQCycleTime();
  config.MaxODR = 1 / (((config.MeasSeqCycleCount + 10) / 16.0) * 1E-6);
//   if (config.SamplingInterval < 1/config.MaxODR) {
//     /* We have requested a sampling rate that cannot be achieved with the time it
//        takes to acquire a sample.
//     */
//     config.SamplingInterval = 1/config.MaxODR;
//   }

// fprintf(stderr, "Sampling Interval: %f s MaxODR: %f Hz\n", config.SamplingInterval, config.MaxODR);

  if (error == AD5940ERR_OK) {
    config.MeasureSeqInfo.SeqId = SEQID_0;
    config.MeasureSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
    config.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    config.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(config.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

AD5940Err sensor::EChem_BioZ::AD5940_CalibrateHSRTIA(void) {
  HSRTIACal_Type hsrtia_cal;

  hsrtia_cal.AdcClkFreq = config.AdcClkFreq;
  hsrtia_cal.ADCSinc2Osr = config.ADCSinc2Osr;
  hsrtia_cal.ADCSinc3Osr = config.ADCSinc3Osr;
  hsrtia_cal.bPolarResult = bTRUE; /* We need magnitude and phase here */
  hsrtia_cal.DftCfg.DftNum = config.DftNum;
  hsrtia_cal.DftCfg.DftSrc = config.DftSrc;
  hsrtia_cal.DftCfg.HanWinEn = config.HanWinEn;
  hsrtia_cal.fRcal = config.RcalVal;
  hsrtia_cal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtia_cal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtia_cal.HsTiaCfg.HstiaCtia = config.CtiaSel;
  hsrtia_cal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;
  hsrtia_cal.HsTiaCfg.HstiaRtiaSel = config.HstiaRtiaSel;
  hsrtia_cal.SysClkFreq = config.SysClkFreq;
  hsrtia_cal.fFreq = config.SweepCfg.SweepStart;

  if (config.SweepCfg.SweepEn == bTRUE) {
    uint32_t i;
    config.SweepCfg.SweepIndex = 0; /* Reset index */
    for (i = 0; i < config.SweepCfg.SweepPoints; i++) {
      AD5940_HSRtiaCal(&hsrtia_cal, config.RtiaCalTable[i]);
      dbgInfo(String("Freq: ") + String(hsrtia_cal.fFreq) + String(", RTIA: Mag: ") +
              String(config.RtiaCalTable[i][0]) + String(" Ohm, Phase: ") + String(config.RtiaCalTable[i][1]));
      AD5940_SweepNext(&config.SweepCfg, &hsrtia_cal.fFreq);
    }
    config.SweepCfg.SweepIndex = 0; /* Reset index */
    config.RtiaCurrValue[0] = config.RtiaCalTable[config.SweepCfg.SweepIndex][0];
    config.RtiaCurrValue[1] = config.RtiaCalTable[config.SweepCfg.SweepIndex][1];
  } else {
    hsrtia_cal.fFreq = config.SinFreq;
    AD5940_HSRtiaCal(&hsrtia_cal, config.RtiaCurrValue);
  }
  return AD5940ERR_OK;
}

// Synchronous Averaging Engine
fImpPol_Type EChem_BioZ::takeAveragedMeasurement(uint8_t num_averages) {
  fImpPol_Type finalResult = {0.0f, 0.0f};
  if (num_averages == 0) return finalResult;

  float currentMag = 0.0f;
  float sumMag = 0.0f;
  float sumPhase = 0.0f;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < num_averages; i++) {
    // ==========================================
    // 1. FLUSH FIFO & CLEAR INTERRUPTS BEFORE TRIGGERING
    // ==========================================
    AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);        // Disable FIFO to wipe it
    AD5940_FIFOCtrlS(FIFOSRC_DFT, bTRUE);         // Re-enable FIFO
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH); // Clear any lingering interrupt flags

    // ==========================================
    // 2. TRIGGER MEASUREMENT SEQUENCE
    // ==========================================
    AD5940_SEQMmrTrig(SEQID_0);

    uint32_t timeoutTicks = 0;
    while (AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bFALSE) {
      vTaskDelay(pdMS_TO_TICKS(1));
      timeoutTicks++;
      if (timeoutTicks > 500) break; // 500ms safeguard
    }

    if (AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bFALSE) continue;

    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);

    uint32_t numSamplesInFifo = AD5940_FIFOGetCnt();
    if (numSamplesInFifo < 4) continue;

    // ==========================================
    // 3. READ FRESH DATA
    // ==========================================
    uint32_t fifoBuf[4]; 
    AD5940_FIFORd(fifoBuf, 4);

    if (numSamplesInFifo > 4) {
        uint32_t dummyBuf[numSamplesInFifo - 4];
        AD5940_FIFORd(dummyBuf, numSamplesInFifo - 4);
    }

    for (int j = 0; j < 4; j++) {
      fifoBuf[j] &= 0x3ffff;      
      if (fifoBuf[j] & (1 << 17)) fifoBuf[j] |= 0xfffc0000; 
    }

    const iImpCar_Type* impData = reinterpret_cast<const iImpCar_Type*>(fifoBuf);
    const float vm = std::hypot(static_cast<float>(impData[1].Real), static_cast<float>(impData[1].Image));
    const float vp = std::atan2(-static_cast<float>(impData[1].Image), static_cast<float>(impData[1].Real));
    const float im = std::hypot(static_cast<float>(impData[0].Real), static_cast<float>(impData[0].Image));
    const float ip = std::atan2(-static_cast<float>(impData[0].Image), static_cast<float>(impData[0].Real));
    
    currentMag = (vm / im) * config.RtiaCurrValue[0];
    sumMag += currentMag;
    sumPhase += (vp - ip) + config.RtiaCurrValue[1];
    validSamples++;
  }

  if (validSamples > 0) {
    finalResult.Magnitude = sumMag / validSamples;
    finalResult.Phase = sumPhase / validSamples;
  }
  return finalResult;
}

void EChem_BioZ::printResult(void) {
  const float freq = (config.SweepCfg.SweepEn == bTRUE) ? config.FreqofData : config.SinFreq;
  
  forEach([freq](const fImpPol_Type& imp) {
    // Serial.printf("Freq: %.2f [Hz], Mag: %.5f [Ohm], Phase: %.5f [deg]\n", freq, imp.Magnitude,
    //               imp.Phase * 180 / MATH_PI);
    //Serial.printf("%.5f", imp.Magnitude);
    //Serial.println();
  });
}

bool EChem_BioZ::stop() {
  // Single-shot measurements automatically go back to sleep, 
  // so we just return true to satisfy the SensorManager.
  return true; 
}

void EChem_BioZ::ISR() {
  // Intentionally left empty. 
  // The AD5940 is now polled synchronously in takeAveragedMeasurement().
}