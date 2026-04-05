#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/sensor.h"

#include <queue>

namespace sensor {
#define MAXSWEEP_POINTS 100 /* Need to know how much buffer is needed to save RTIA calibration result */

  typedef struct {
    /* Common configurations for all kinds of Application. */
    BoolFlag bParaChanged; /* Indicate parameters have been set  */
    uint32_t SeqStartAddr; /* Initialization sequence start address in SRAM of AD5940  */

    /* Application related parameters */
    float SysClkFreq;       /* The real frequency of system clock */
    float AdcClkFreq;       /* The real frequency of ADC clock */
    uint32_t FifoThresh;    /* FIFO threshold. Should be N*4 */
    float SamplingInterval; /* decides the period of WakeupTimer who will trigger sequencer periodically. */
    int32_t NumOfData;      /* By default it's '-1'. Stops after NumofData, otherwise never stop. */
    float RcalVal;          /* Rcal value in Ohm */
    uint32_t PwrMod;        /* Control Chip power mode(LP/HP) */

    /* Receive path configuration */
    uint32_t SenseP; // positive sense pin
    uint32_t SenseN; // negative sense pin

    uint32_t DswitchSel;
    uint32_t PswitchSel;
    uint32_t NswitchSel;
    uint32_t TswitchSel;
    uint32_t HstiaRtiaSel; /* Internal RTIA selection */
    uint32_t CtiaSel;      /* Select CTIA in pF unit from 0 to 31pF */
    uint32_t ExcitBufGain; /* Select from  EXCTBUFGAIN_2, EXCTBUFGAIN_0P25 */
    uint32_t HsDacGain;    /* Select from  HSDACGAIN_1, HSDACGAIN_0P2 */
    uint32_t HsDacUpdateRate;
    float DacVoltPP;   /* DAC output voltage in mV peak to peak. */
    float SinFreq;     /* Frequency of excitation signal */
    float Eac;         /* Peak amplitude of sine wave [mV]*/
    uint32_t DftNum;   /* DFT number */
    uint32_t DftSrc;   /* DFT Source */
    BoolFlag HanWinEn; /* Enable Hanning window */

    uint32_t AFEBW;      // select from AFEBW_250KHZ, AFEBW_100KHZ, AFEBW_50KHZ
    uint32_t ADCPgaGain; /* PGA Gain selection */
    uint8_t ADCSinc3Osr; /* SINC3 OSR selection */
    uint8_t ADCSinc2Osr; /* SINC2 OSR selection */

    /* Sweep Function Control */
    SoftSweepCfg_Type SweepCfg;
    float SweepCurrFreq;
    float SweepNextFreq;
    float RtiaCurrValue[2];                  /* Calibrated Rtia value of current frequency */
    float RtiaCalTable[MAXSWEEP_POINTS][2]; /* Calibrated Rtia Value table */
    float FreqofData;                       /* The frequency of latest data sampled */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
    BoolFlag StopRequired;  /* After FIFO is ready, stop the measurement sequence */
    uint32_t FifoDataCount; /* Count how many times BioZ has been measured */

    uint32_t MeasSeqCycleCount; /* How long the measurement sequence will take */
    float MaxODR;               /* Max ODR for sampling in this config */

    BoolFlag IMP4WIRE;  // flag for 4-wire vs 2-wire Bio-Impedance
    BoolFlag ACcoupled;  // flag for AC vs DC coupling

    // Inside BioZConfig_Type struct
    float DualRtiaCal[2][2];  // Index 0: Coil (Mag, Phase), Index 1: SPE (Mag, Phase)
    uint8_t target_mux;       // 0x00=SPE1, 0x01=SPE2, 0xFF=Both Coils
    uint8_t num_averages;     // Number of averages to take
    float coilFrequency;      // Target frequency for coils
    float speFrequency;       // Target frequency for SPEs
    
  } BioZConfig_Type;

  class EChem_BioZ : public Sensor, public SensorQueue<fImpPol_Type> {
  public:
    EChem_BioZ();

    // Control functions
    bool start(void);
    bool globalStart(void);
    bool stop(void);
    void ISR(void);
    bool globalStop(void);
    bool loadParameters(uint8_t* data, uint16_t len);


    // Data processing and retrieval
    void printResult(void);
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<fImpPol_Type>::popBytes(num_items); }
    size_t getNumBytesAvailable(void) const override { return SensorQueue<fImpPol_Type>::size(); }

  private:
    int32_t initAD5940(void);
    AD5940Err setupMeasurement(void);
    void configureWaveformParameters(void);
    AD5940Err configureFrequencySpecifics(float freq);
    AD5940Err AD5940_CalibrateHSRTIA(void);
    AD5940Err updateRegisters(void);

    // Sequence generation functions
    AD5940Err generateInitSequence(void);
    AD5940Err generateMeasSequence(void);

    // Processing functions
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    AD5940Err calibrateFrequency(float targetFreq, float* calDataOut);
    fImpPol_Type takeAveragedMeasurement(uint8_t num_averages);

    BioZConfig_Type config;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor