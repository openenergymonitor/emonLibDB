// emonLibDB.cpp - Library for openenergymonitor
//
// This library provides continuous single- or three-phase monitoring of real power on up to twelve CT channels.
// All of the time-critical code is contained within the ISR, only the slower activities
// are done within the main code as part of the user sketch. These slower activities could include RF transmissions,
// and all Serial output statements (not part of the library).  
//
// This library is suitable for either 50 or 60 Hz operation.
//
// Original Author: Robin Emley (calypso_rae on Open Energy Monitor Forum)
// Converted into a library by Trystan Lea, and converted from EmonLibCM and extended to 3 Voltage channels,
//  12 current/power channels for the emonTx V4 using the Atmel AVR128DB48 by Robert Wall.
// Acknowledging significant input at various stages from:
//   Jörg Becker (@joergbecker32) for his background work on interrupts and the ADC; 
//   @ursi (Andries) and @mafheldt (Mike Afheldt) for suggestions made at 
//     https://community.openenergymonitor.org/t/emonlib-inaccurate-power-factor/3790 and
//     https://community.openenergymonitor.org/t/rms-calculations-in-emonlib-and-learn-documentation/3749/3; 
//   @awjlogan for his suggestions regarding memory use;
//   @cbmarkwardt & @dBC for his suggestion to use the AVR Hardware Multiplier.


// Version 1.0.0 6/5/2023 Initial public release
// Version 1.0.1 25/11/2023  Hardware 'Fast Multiply' removed - found to be slower. 
//   ADC0SampLenLineNeutral was 21, ADC0SampLenLineLine was 19. No change to user interface. 
//   Very minor changes to documentation to reflect faster sampling rates.


// #include "WProgram.h" un-comment for use on older versions of Arduino IDE
#define LEDpin PIN_PB2


#include "emonLibDB.h"

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif
// The next 2 lines only required if printing from the library to debug.
//#undef Serial
//#define Serial Serial3

static uint8_t cyclesPerSec = 50;                                      // mains frequency in Hz (i.e. 50 or 60)
float  datalogPeriodInSeconds = 9.8;                                   // Suggested value for emonCMS
static uint8_t min_startup_cycles = 10;

// Maximum number of Voltage (v) channels used to create arrays
static const uint8_t max_no_of_vInputs = 3;

// Maximum number of Current (c) channels used to create arrays
static const uint8_t max_no_of_cInputs = 12;

// Maximum number of Power (p) channels used to create arrays
static const uint8_t max_no_of_pInputs = 12;

// number of Voltage (v) channels that have been set (active)
static uint8_t no_of_vInputs = 0;

// number of current (c) channels that have been set (active)
static uint8_t no_of_cInputs = 0;

// number of power (p) channels that have been set (active)
static uint8_t no_of_pInputs = 0;

// Total number of active channels
static uint8_t numInputs = 0;

// Line-Line C.T. use
static bool lineLine = false;

// ADC samples per set (V + I)
static volatile uint8_t samplesPerSet;

// for general interaction between the main code and the ISR
static volatile bool cycleComplete;
static volatile bool datalogEventPending = false;
static double line_frequency;                                          // Timed from sample rate & cycle count
static const double VsCal = 8.0087;                                    // Magic number for emonVs + emonTx4
static const double ICal = 3.0;                                        // Magic number for 333 mV input of emonTx4

// Structures for defining voltage & current/power Inputs, calibration values etc
//   and storing intermediate results
struct pCal{
  int8_t low;                                                          // For future use
  double lowPhase;                                                     // Phase error (at low limit and below when implemented)
  int8_t high;                                                         // For future use
  double highPhase;                                                    // For future use
};

struct phaseData
{
  uint8_t positionOfV;                                                 // Position of this voltage sample in scan (zero based)
  double  phaseErrorV;                                                 // Phase Error in degrees
  uint8_t positionOfC;                                                 // Position of this current sample in scan (zero based)
  double  phaseErrorC;                                                 // Phase Error in degrees
  int16_t relativeCsample;                                             // Position of current sample t use relative to this
  int16_t relativeLastVsample;                                         // Position of previous voltage sample to use relative to this
  int16_t relativeThisVsample;                                         // Position of next voltage sample to use relative to this
  double  x;                                                           // Coefficient for interpolaton
  double  y;                                                           // Coefficient for interpolaton
};

struct vIn{
  uint8_t inPin;
  double amplitudeCal = 100.0;                                         // Nominal percentage change from emonVs
  struct pCal phaseCal;
  uint8_t scanPos;                                                     // Position in sample set
  double volatile Vrms;                                                // result of calculations
  bool inUse = false;
  bool volatile acPresent = false;
} vInput[max_no_of_vInputs];                                           // by hardware input order, 0-based = channel-1

struct cIn{
  uint8_t inPin;
  double amplitudeCal = 100.0;                                         // Nominal 100 A CT Rating for 0.333 V output (with use of burden if necessary)
  struct pCal phaseCal;
  uint8_t scanPos;                                                     // Position in sample set
  double volatile Irms;                                                // results of calculations
  bool inUse = false;
} cInput[max_no_of_cInputs];                                           // by hardware input order, 0-based = channel-1

struct pIn{                                                            // Power definition
  struct cIn* cIn;                                                     // Input no. of current input from API
  struct vIn* vIn1;                                                    // Input no. of voltage input 1
  struct vIn* vIn2;                                                    // Input no. of voltage input 2
  double residualEnergy;                                               // left over from value reported
  uint64_t sumEnergy;                                                  // accumulated so far (Wh)
  struct phaseData phaseDataV1;                                        // phase data for V1
  struct phaseData phaseDataV2;                                        // phase data for V2
  int32_t sumPA1;                                                      // 'Partial powers' line-neutral loads
  int32_t sumPB1;
  int32_t sumPA2;                                                      // Second 'Partial powers' line-line loads
  int32_t sumPB2;
  int16_t realPower;
  int16_t apparentPower;
  int32_t wh;
  double pf;
  bool inUse = false;
} pInput[max_no_of_pInputs];                                           // by current input order

// Map pIn struct to cInput
static uint8_t pInMap[max_no_of_cInputs];

// Input pin map
// Map OEM 'channels' (V1,V2,V3,I1...I12, analog) to ADC inputs PD0 - PD6, PE0 - PE3, PF0 - PF3, PF3)
// 
static uint8_t volatile ADCmap[max_no_of_vInputs + max_no_of_cInputs + 1] = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 16, 17, 18, 19, 19};

struct ADC {
  uint8_t adcInPin = 0xff;
  struct pIn* pInput = NULL;
} ADC_Sequence[max_no_of_vInputs + max_no_of_cInputs];                 // The order to read the channels

// Sample array & associated
static constexpr uint16_t RAWSAMPLESSIZE = 1024;                       // MUST be a power of 2
static volatile int16_t rawSamples[RAWSAMPLESSIZE];                    // Array to store almost-raw samples 
static volatile uint16_t sampleCount = 0;                              // actual samples (not sets)
               
volatile int16_t copy_rawSamples[RAWSAMPLESSIZE];
uint8_t volatile once = 68;
bool volatile printit = false;
static uint16_t volatile sampIndex;
               
// ADC data
static uint8_t ADCBits = 12;                                                  // 12 for the AVR128.
static double Vref = 1.024;                                                   // ADC Reference Voltage = 1.024 V for emonTx V4.x

static double frequencyCalibration = 1.0;                                     // Calibrate line frequency

static constexpr uint32_t ClockFreq = 24e6;
static constexpr uint32_t ADCScale = 10000000;
static constexpr uint8_t ADC0SampDly = 0;                                     // Range 0 - 15
static uint8_t   ADC0SampLen;                                                 // Set later
static uint16_t  ADC0prescaleDivider;                                         // Set later

static constexpr uint16_t ADC0SampLenLineNeutral = 18;                        // Range 0 - 255
static constexpr uint16_t ADC0prescaleDividerLineNeutral = ADC_PRESC_DIV32_gc;

static constexpr uint8_t ADC0SampLenLineLine = 16;                            // Range 0 - 255
static constexpr uint16_t ADC0prescaleDividerLineLine = ADC_PRESC_DIV48_gc;

 /* Possible dividers are 2,4,8,12,16,20,24,28,32,48,64,96,128,256 */

static uint16_t ADCDuration;                                           // set later, calculated from ADC settings

// Pulse Counting

#define PULSEINPUTS 3                                                  // No of available interrupts for pulse counting

struct pulse {
  /* N.B. Pulse inputs are of necessity hard-coded in the individual ISRs and in countAllPulses()
          Pulse counting on any channel is only available if the appropriate solder link is made on the hardware,
          and the related "TMP" link is broken.
  */
  uint16_t pulseMinPeriod = 20;                                        // default to 20 ms
  uint8_t edge = FALLING;                                              // edge to increment count 
  uint32_t pulseCount = 0;                                             // Total number of pulses from switch-on 
  uint16_t pulseIncrement = 0;                                         // Incremental number of pulses between readings
  bool pulseEnabled = false;
  bool laststate = HIGH;                                               // Last state of interrupt pin
  volatile bool timing = false;                                        // 'debounce' period running
  volatile uint32_t pulseTime;                                         // Instant of the last interrupt - used for debounce logic
} pulses[PULSEINPUTS];

static volatile bool pulseInterrupt = false; 
static volatile bool digitalInterrupt = false;
static volatile bool analogInterrupt = false;

struct analogue {
  /*
      N.B. The input is hard-coded. The analogue input is only available 
      if the appropriate solder link is made on the hardware, and the related
      "TMP" link is broken.
  */
  bool enabled = false;
  uint16_t count = 0;
  uint8_t scanPos;
} analogueInput;

// Set-up values
//--------------
// These set up the library for different hardware configurations
//
// cyclesPerSec              Defines the mains frequency
// _min_startup_cycles       The period of inactivity whilst the system settles at start-up
// _datalog_period           The interval after which data is reported for logging which data is reported for logging
//
// vInputs                   The calibration values for each voltage input
// cInputs                   The calibration values for each current input
// pInputs                   The link between voltage & current to calculate power

// Calibration values
//-------------------
// Many calibration values are used in this sketch: 
//
// ADCCal                    This sets up the ADC reference voltage
// voltageCal                A per-channel amplitude calibration for each ac voltage channel.
// currentCal                A per-channel amplitude calibration for each current transformer.
// phaseCal                  A per-channel calibration for the phase error correction (V & I)


// With most hardware, the default values are likely to work fine without 
// need for change.  A compact explanation of each of these values now follows:

// Voltage calibration constant. This is a fine calibration adjustment for the mains voltage:
// The emonVs power supply & AC Voltage monitor is designed to step down the voltage from 240V to 0.3 V
// (well within the nominal 1 V (0.333 V rms) input range of the ADC. The power supply & monitor can
// be used on all European and North American domestic supplies, without any changes.

// The theoretical calibration constant is 100. (Think of it as a percentage.) The actual constant for
// a given emonVs is likely to be different by no more than a few percent, normally much less.

// Current calibration constant. This is the mains current that would give 0.333 V
// at the ADC input, i.e. the rated current of the c.t. (with a custom burden if necessary):
// The emonTx & emonPi inputs are designed to accept a voltage output of 0.333 V at rated current.
//  All input channels are the same. The actual constant for a given unit and CT is likely 
//  to be different from the c.t. rated current by no more than a few percent, normally much less.

// phaseCal is the phase lead in degrees, and is specified individually for each voltage input
// and each current transformer (or equivalent).


/**************************************************************************************************
*
*   General variables 
*
*
***************************************************************************************************/


// --------------  general global variables -----------------
// Some of these variables are used in multiple blocks.
// For integer maths, many variables need to be 'int32_t' or in extreme cases 'int64_t'

static uint16_t ADC_Counts = 1 << ADCBits;

static volatile bool firstcycle = true;
    
static uint16_t acDetectedThreshold = ADC_Counts >> 4; // ac voltage detection threshold, ~10% of nominal voltage (given large amount of ripple)

// accumulators & counters for use by the ISR

static uint32_t ADCsamplesPerDatalogPeriod;
static uint32_t sampleSetsInDatalogPeriod;
static uint16_t samplesPerMainsCycle; 

void memset_v(volatile void *s, size_t n)
{
  // Zero a block of memory
  // Usage: memset_v(&mymemory, sizeof(mymemory));
  volatile uint8_t *q = (volatile uint8_t *)s;
  while (n-- > 0)
    *q++ = 0;
}

template <class Accum>
void accumSwap(volatile Accum **p_c, volatile Accum **p_p ) 
{
  // Swap buffers
  // Usage: accumSwap <myAccum> (accumSwapZero(&collecting, &processing);
  volatile Accum *tmp;
  tmp = *p_p;
  *p_p = *p_c;
  *p_c = tmp;
}

template <class Accum>
void accumSwapZero(volatile Accum **p_c, volatile Accum **p_p ) 
{
  // Swap and zero buffers
  // Usage: accumSwap <myAccum> (accumSwapZero(&collecting, &processing);
  volatile Accum *tmp;
  tmp = *p_p;
  *p_p = *p_c;
  *p_c = tmp;
  memset_v(*p_c, sizeof(Accum));         
}

// per long reporting period
struct longAccum
{
  uint32_t sampleSets;
  uint32_t cycles;                  
  int64_t  cumSampleDeltas[max_no_of_vInputs + max_no_of_cInputs];     // for offset removal (I)
  uint64_t sumSampleSquared[max_no_of_vInputs + max_no_of_cInputs];    // period sum
  int64_t  sumPA1[max_no_of_vInputs + max_no_of_pInputs];              // Sums of partial powers - N.B. only current inputs,
  int64_t  sumPB1[max_no_of_vInputs + max_no_of_pInputs];              //   so the 'V-input' sums are always empty
  int64_t  sumPA2[max_no_of_vInputs + max_no_of_pInputs];
  int64_t  sumPB2[max_no_of_vInputs + max_no_of_pInputs];
};

volatile struct longAccum accum_4;
volatile struct longAccum accum_5;
volatile struct longAccum *longCollecting = &accum_4;
volatile struct longAccum *longProcessing = &accum_5;

enum polarities {NEGATIVE, POSITIVE};
// For an enhanced polarity detection mechanism, which includes a persistence check
#define POLARITY_CHECK_MAXCOUNT 1 // 3
static volatile polarities polarityUnconfirmed;   
static volatile polarities polarityConfirmed;                          // for improved zero-crossing detection
static volatile polarities polarityConfirmedOfLastSampleV;             // for zero-crossing detection (this is only used on the first voltage


/**************************************************************************************************
*
*   APPLICATION INTERFACE - Getters & Setters
*
*
***************************************************************************************************/

void EmonLibDB_set_vInput(uint8_t input, double _amplitudeCal, double _phase)
{
    --input;
    if (input < max_no_of_vInputs)
    {
      vInput[input].inPin = input;
      vInput[input].amplitudeCal = _amplitudeCal * VsCal * Vref / ADC_Counts;
      vInput[input].phaseCal.low = 0;
      vInput[input].phaseCal.lowPhase = _phase;
      vInput[input].phaseCal.high = 0;
      vInput[input].phaseCal.highPhase = 0;
    }
}
      
void EmonLibDB_reCalibrate_vInput(uint8_t input, double _amplitudeCal, double _phase)
{
    --input;
    if (input < max_no_of_vInputs)
    {
      vInput[input].amplitudeCal = _amplitudeCal * VsCal * Vref / ADC_Counts;
      if (_phase != vInput[input].phaseCal.lowPhase)
      {
        vInput[input].phaseCal.low = 0;
        vInput[input].phaseCal.lowPhase = _phase;
        vInput[input].phaseCal.high = 0;
        vInput[input].phaseCal.highPhase = 0;

        for (uint8_t i=0; i<no_of_pInputs; i++)   // recalculate Phase data for each power that uses this voltage
        {
          if(pInput[input].vIn1 == &vInput[input] || pInput[input].vIn2 == &vInput[input])
          {
            populatePIn(i);
          }
        }
     }
    }
}

void EmonLibDB_set_cInput(uint8_t input, double _amplitudeCal, double _phase)
{
    --input;
    if (input < max_no_of_cInputs)
    {
      cInput[input].inPin = input;
      cInput[input].amplitudeCal =  _amplitudeCal * ICal * Vref / ADC_Counts;
      
      cInput[input].phaseCal.low = 0;
      cInput[input].phaseCal.lowPhase = _phase;
      cInput[input].phaseCal.high = 0;
      cInput[input].phaseCal.highPhase = 0;
    }
}

void EmonLibDB_reCalibrate_cInput(uint8_t input, double _amplitudeCal, double _phase)
{
    --input;
    if (input < max_no_of_cInputs)
    {
      cInput[input].amplitudeCal = _amplitudeCal * ICal * Vref / ADC_Counts;
      
      if (_phase != cInput[input].phaseCal.lowPhase)
      {
        cInput[input].phaseCal.low = 0;
        cInput[input].phaseCal.lowPhase = _phase;
        cInput[input].phaseCal.high = 0;
        cInput[input].phaseCal.highPhase = 0;
        pInput[input].phaseDataV1.phaseErrorC = pInput[input].cIn->phaseCal.lowPhase;
        calcXY(&(pInput[input].phaseDataV1), cyclesPerSec, ADCDuration, samplesPerSet);
        if (pInput[input].vIn1 != pInput[input].vIn2)
        {
          pInput[input].phaseDataV2.phaseErrorC = pInput[input].cIn->phaseCal.lowPhase;
          calcXY(&(pInput[input].phaseDataV2), cyclesPerSec, ADCDuration, samplesPerSet);
        }
      }
    }
}

void EmonLibDB_set_pInput(uint8_t input, uint8_t inputV1)
{
    if (input <= max_no_of_pInputs)
    {
      --input;
      pInput[no_of_pInputs].cIn = &cInput[input];
      pInput[no_of_pInputs].vIn1 = &vInput[--inputV1];
      pInput[no_of_pInputs].vIn2 = &vInput[inputV1];
      pInput[no_of_pInputs].inUse = true;
      // Map pIn struct to cInput
      pInMap[input] = no_of_pInputs;
      no_of_pInputs++;
    }
}

void EmonLibDB_set_pInput(uint8_t input, uint8_t inputV1, uint8_t inputV2)
{
    --input;
    if (input <= max_no_of_pInputs)
    {
      pInput[no_of_pInputs].cIn = &cInput[input];
      pInput[no_of_pInputs].vIn1 = &vInput[--inputV1];
      pInput[no_of_pInputs].vIn2 = &vInput[--inputV2];
      pInput[no_of_pInputs].inUse = true;
      pInMap[input] = no_of_pInputs;
      no_of_pInputs++;
      lineLine = true;
    }
}

void EmonLibDB_cyclesPerSecond(uint8_t _cyclesPerSec)
{
    cyclesPerSec = _cyclesPerSec;   
}

void EmonLibDB_minStartupCycles(uint8_t _min_startup_cycles)
{
    min_startup_cycles = _min_startup_cycles;
}

void EmonLibDB_datalogPeriod(float _datalogPeriodInSeconds)
{
    if (_datalogPeriodInSeconds < 0.5)
        _datalogPeriodInSeconds = 0.5;
    datalogPeriodInSeconds = _datalogPeriodInSeconds;
    samplesPerMainsCycle = ADCScale / ADCDuration / cyclesPerSec;
    ADCsamplesPerDatalogPeriod = datalogPeriodInSeconds * ADCScale / ADCDuration;
    sampleSetsInDatalogPeriod = (ADCsamplesPerDatalogPeriod 
        - samplesPerMainsCycle) / samplesPerSet;                       // because the period ends at the next zero crossing.
}

void EmonLibDB_fCal(double _frequencyCal)
{
    frequencyCalibration = _frequencyCal;
}

void EmonLibDB_setPulseEnable(bool _enable)
{
    EmonLibDB_setPulseEnable(1, _enable);
}

void EmonLibDB_setPulseEnable(uint8_t input, bool _enable)
{
    pulses[--input].pulseEnabled = _enable;
    if (_enable)
    {
      switch (input) 
      {
        // PULLUPEN = 1, ISC = 1 trigger both
        case (0): PORTA.PIN6CTRL = 0x08 | 0x01; break;                 //'pulse'
        case (1): PORTA.PIN7CTRL = 0x08 | 0x01; break;                 //'digital'       
        case (2): PORTF.PIN3CTRL = 0x08 | 0x01; break;                 //'analogue'
        
      }
    }
    else
    {
      switch (input) 
      {
        // reset all bits
        case (0): PORTA.PIN6CTRL = 0x00; break;                        //'pulse'
        case (1): PORTA.PIN7CTRL = 0x00; break;                        //'digital'
        case (2): PORTF.PIN3CTRL = 0x00; break;                        //'analogue'
      }
    }
}

void EmonLibDB_setPulseMinPeriod(uint16_t _period)
{
    pulses[0].pulseMinPeriod = _period;
    pulses[0].edge = FALLING;
    pulses[0].laststate = HIGH;
}

void EmonLibDB_setPulseMinPeriod(uint8_t input, uint16_t _period, uint8_t _edge)
{
    pulses[--input].pulseMinPeriod = _period;
    pulses[input].edge = _edge;
    switch (_edge)
    {
      case (RISING):  pulses[input].laststate = LOW;  break;
      case (FALLING): pulses[input].laststate = HIGH; break;
    }
}

void EmonLibDB_setPulseCount(uint32_t _pulseCount)
{
    pulses[0].pulseCount = (uint32_t)_pulseCount;                 
}

void EmonLibDB_setPulseCount(uint8_t input, uint32_t _pulseCount)
{
    pulses[--input].pulseCount = _pulseCount;                
}

void EmonLibDB_setWattHour(uint8_t input, int32_t _wh)
{
    pInput[--input].wh = (int32_t)_wh;
}

void EmonLibDB_setAnalogueEnable(bool _enable)
{
    analogueInput.enabled = _enable;
}

void EmonLibDB_ADCCal(double _Vref)
{
    Vref = _Vref;
}   




bool EmonLibDB_acPresent(int8_t input)
{
    return(vInput[--input].acPresent);
}

int16_t EmonLibDB_getRealPower(int8_t input)
{
    if (cInput[--input].inUse)
      return pInput[pInMap[input]].realPower;
    else
      return 0;
}

int16_t EmonLibDB_getApparentPower(int8_t input)
{
    if (cInput[--input].inUse)
      return pInput[pInMap[input]].apparentPower;
    else
      return 0;
}

double EmonLibDB_getPF(int8_t input)
{
    if (cInput[--input].inUse)
      return pInput[pInMap[input]].pf;
    else
      return 0.0;
}

bool EmonLibDB_getCinputInUse(uint8_t input)
{
    return cInput[--input].inUse;
}

double EmonLibDB_getIrms(uint8_t input)
{
    if (cInput[--input].inUse)
      return cInput[input].Irms;
    else
      return 0.0;
}

bool EmonLibDB_getVinputInUse(uint8_t input)
{
    return vInput[--input].inUse;
}

double EmonLibDB_getVrms(uint8_t input)
{
    return vInput[--input].Vrms;
}

double EmonLibDB_getDatalogPeriod(void)
{
    return datalogPeriodInSeconds;
}

double EmonLibDB_getLineFrequency(void)
{
    return line_frequency;
}

int32_t EmonLibDB_getWattHour(int8_t input)
{
    if (cInput[--input].inUse)
      return pInput[pInMap[input]].wh;
    else
      return 0;
}

uint32_t EmonLibDB_getPulseCount(void)
{
    return pulses[0].pulseCount;
}

uint32_t EmonLibDB_getPulseCount(byte input)
{
    return pulses[--input].pulseCount;
}


uint16_t EmonLibDB_getAnalogueCount(void)
{
    return analogueInput.count;
}


void EmonLibDB_Init(void)
{   
    ADC0SampLen = lineLine ? ADC0SampLenLineLine : ADC0SampLenLineNeutral;
    
    ADC0prescaleDivider = lineLine ? ADC0prescaleDividerLineLine : ADC0prescaleDividerLineNeutral;
    uint16_t ADCDivider = 0;
    switch (ADC0prescaleDivider)
    {
      case (ADC_PRESC_DIV2_gc)   : ADCDivider = 2;   break;
      case (ADC_PRESC_DIV4_gc)   : ADCDivider = 4;   break;
      case (ADC_PRESC_DIV8_gc)   : ADCDivider = 8;   break;
      case (ADC_PRESC_DIV12_gc)  : ADCDivider = 12;  break;
      case (ADC_PRESC_DIV16_gc)  : ADCDivider = 16;  break;
      case (ADC_PRESC_DIV20_gc)  : ADCDivider = 20;  break;
      case (ADC_PRESC_DIV24_gc)  : ADCDivider = 24;  break;
      case (ADC_PRESC_DIV28_gc)  : ADCDivider = 28;  break;
      case (ADC_PRESC_DIV32_gc)  : ADCDivider = 32;  break;
      case (ADC_PRESC_DIV48_gc)  : ADCDivider = 48;  break;
      case (ADC_PRESC_DIV64_gc)  : ADCDivider = 64;  break;
      case (ADC_PRESC_DIV96_gc)  : ADCDivider = 96;  break;
      case (ADC_PRESC_DIV128_gc) : ADCDivider = 128; break;
      case (ADC_PRESC_DIV256_gc) : ADCDivider = 256; break;
    }
      
    ADCDuration = int((4.0 / ClockFreq + (15.5 + ADC0SampDly + ADC0SampLen) * ADCDivider / ClockFreq) * ADCScale);
    // Time in tenths of microseconds for one ADC conversion. 46.1 µs appears realistic for L-N, 59.1 µs for Line-Line. (getReadings takes ~20 to ~ 28 ms).
    
    
    // Build ADC_Sequence array: ADC MPX inputs in scan order,
    //  with associated vInput and cInput array index
    // also fill in phaseData for each pInput
    //
    // First, the voltage inputs
    numInputs = 0;
    for (uint8_t i = 0; i < max_no_of_pInputs; i++)
    {
      if (pInput[i].inUse)
      {
        if (ADC_Sequence[numInputs].adcInPin == 0xff && !pInput[i].vIn1->inUse)
        {
          pInput[i].vIn1->scanPos = numInputs;
          pInput[i].vIn1->inUse = true;
          ADC_Sequence[numInputs].adcInPin = ADCmap[pInput[i].vIn1->inPin];
          numInputs++;
          no_of_vInputs++;
        }
        if (pInput[i].vIn2 != pInput[i].vIn1 && ADC_Sequence[numInputs].adcInPin == 0xff
          && !pInput[i].vIn2->inUse)
        {
          pInput[i].vIn2->scanPos = numInputs;
          pInput[i].vIn2->inUse = true;
          ADC_Sequence[numInputs++].adcInPin = ADCmap[pInput[i].vIn2->inPin];
          no_of_vInputs++;
        }
      }
    }

    // then the current inputs
    for (uint8_t i = 0; i < max_no_of_pInputs; i++)
    {
      if (pInput[i].inUse && ADC_Sequence[numInputs].adcInPin == 0xff && !pInput[i].cIn->inUse)
      {
        pInput[i].cIn->scanPos = numInputs;
        pInput[i].cIn->inUse = true;
        ADC_Sequence[numInputs].adcInPin = ADCmap[max_no_of_vInputs + pInput[i].cIn->inPin]; 
        ADC_Sequence[numInputs++].pInput = &pInput[i];
        no_of_cInputs++;
      }
    }
      
    // finally the analogue input
    if (analogueInput.enabled)
    {
      analogueInput.scanPos = numInputs;
      ADC_Sequence[numInputs++].adcInPin = ADCmap[max_no_of_vInputs + max_no_of_cInputs];
      analogueInput.count = 0;
    }
    
    samplesPerSet = numInputs;
 
    allPhaseData();

    ADCsamplesPerDatalogPeriod = datalogPeriodInSeconds * ADCScale / ADCDuration;
    samplesPerMainsCycle = ADCScale / ADCDuration / cyclesPerSec;
    ADCsamplesPerDatalogPeriod = datalogPeriodInSeconds * ADCScale / ADCDuration;
    sampleSetsInDatalogPeriod = (ADCsamplesPerDatalogPeriod 
        - samplesPerMainsCycle) / samplesPerSet;                       // because the period ends at the next zero crossing.
    datalogEventPending = false;
    EmonLibDB_Start();
}

/**************************************************************************************************
*
*   START
*
*
***************************************************************************************************/

void EmonLibDB_Start(void)
{
    firstcycle = true;
/*    
    Set up the ADC to be free-running 
*/

    ADC0.SAMPCTRL = ADC0SampLen;                                       //  -  SAMPLEN  Do not change - selected above
    ADC0.CTRLD |= 0x0;                                                 //  -  SAMPDLY
    
    VREF.ADC0REF = VREF_REFSEL_1V024_gc;
    ADC0.CTRLC = ADC0prescaleDivider;                                  // Do not change - selected above
    ADC0.CTRLA = ADC_ENABLE_bm;
    ADC0.CTRLA |= ADC_RESSEL_12BIT_gc;
    ADC0.CTRLA |= ADC_FREERUN_bm;
    ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc;
    ADC0.INTCTRL |= ADC_RESRDY_bm;
    ADC0.COMMAND = ADC_STCONV_bm;

    sei();                                                             // Enable Global Interrupts
}



/**************************************************************************************************
*
*   Retrieve and apply final processing of data ready for reporting
*
*
***************************************************************************************************/

void EmonLibDB_getReadings()
{

// Use the 'volatile' variables passed from the ISR.

    for (uint8_t i=no_of_vInputs; i<max_no_of_vInputs; i++)
    {
      if (!vInput[i].inUse)
      {
          vInput[i].Vrms = 0;
      }
    }
    for (uint8_t i=no_of_pInputs; i<max_no_of_pInputs; i++)
    {
      if (!pInput[i].inUse)
      {
          cInput[i].Irms = 0;
          pInput[i].residualEnergy = 0;
          pInput[i].sumEnergy = 0;
          pInput[i].realPower = 0;
          pInput[i].apparentPower = 0;
          pInput[i].wh = 0;
          pInput[i].pf = 0;
      }
    }


    for (uint8_t channel = 0; channel < PULSEINPUTS; channel++)
    {
      if (pulses[channel].pulseIncrement)                              // if the ISR has counted some pulses, update the total count
      {
        pulses[channel].pulseCount += pulses[channel].pulseIncrement;
        pulses[channel].pulseIncrement = 0;
      }
    }

    // Calculate the final values, scaling for the number of samples and applying calibration coefficients.
    // The final values are deposited in global arrays for extraction by the 'getter' functions.

    // The rms of a signal plus an offset is sqrt(signal^2 + offset^2).
    // The rms value still contains the fine offset. Correct this by subtracting the "Offset V^2" before the sq. root.

    for (uint8_t i=0; i<no_of_vInputs; i++)    // Voltage channels
    {
      vInput[i].Vrms = ((double)longProcessing->sumSampleSquared[i] / longProcessing->sampleSets);
      vInput[i].Vrms -= ((double)longProcessing->cumSampleDeltas[i] / longProcessing->sampleSets 
                               * longProcessing->cumSampleDeltas[i] / longProcessing->sampleSets);
      vInput[i].Vrms = sqrt(vInput[i].Vrms);
      vInput[i].Vrms *= vInput[i].amplitudeCal;
    }

    line_frequency = (double)longProcessing->cycles * ADCScale /
      (longProcessing->sampleSets * samplesPerSet 
      * ADCDuration) * frequencyCalibration;

    for (uint8_t i=0; i<no_of_pInputs; i++)    // Current & Power channels
    {
      //  root of mean squares, removing fine offset
      //  The rms of a signal plus an offset is sqrt(signal^2 + offset^2).
      //  Here (signal+offset)^2 = sumSampleSquared[] / sampleSets
      //       offset = cumSampleDeltas / sampleSets


      pInput[i].cIn->Irms = ((double)longProcessing->sumSampleSquared[pInput[i].cIn->scanPos] / longProcessing->sampleSets);
      pInput[i].cIn->Irms -= ((double)longProcessing->cumSampleDeltas[pInput[i].cIn->scanPos] / longProcessing->sampleSets
                                    * longProcessing->cumSampleDeltas[pInput[i].cIn->scanPos] / longProcessing->sampleSets);
      pInput[i].cIn->Irms = sqrt(pInput[i].cIn->Irms);
      pInput[i].cIn->Irms *= pInput[i].cIn->amplitudeCal;

      // voltages & currents still contain the fine offsets.
      // Correct this by subtracting the "Offset Power": cumV_deltas * cumI_deltas

      double sumEnergy = longProcessing->sumPA1[pInput[i].cIn->scanPos] * pInput[i].phaseDataV1.x
        + longProcessing->sumPB1[pInput[i].cIn->scanPos] * pInput[i].phaseDataV1.y;
        // Divide by sample count to get power and remove "offset power"
      double powerNow = sumEnergy / longProcessing->sampleSets - (double)longProcessing->cumSampleDeltas[pInput[i].cIn->scanPos]
       / longProcessing->sampleSets * longProcessing->cumSampleDeltas[pInput[i].vIn1->scanPos] / longProcessing->sampleSets;
      powerNow *= pInput[i].vIn1->amplitudeCal * pInput[i].cIn->amplitudeCal;

      double powerNow2 = 0;                                            // Line-Line loads
      if (pInput[i].vIn1 != pInput[i].vIn2)
      {
        double sumEnergy2 = longProcessing->sumPA2[pInput[i].cIn->scanPos] * pInput[i].phaseDataV2.x
        + longProcessing->sumPB2[pInput[i].cIn->scanPos] * pInput[i].phaseDataV2.y;
        powerNow2 = sumEnergy2 / longProcessing->sampleSets - (double)longProcessing->cumSampleDeltas[pInput[i].cIn->scanPos]
               / longProcessing->sampleSets * longProcessing->cumSampleDeltas[pInput[i].vIn2->scanPos] / longProcessing->sampleSets;
        powerNow2 *= pInput[i].vIn2->amplitudeCal * pInput[i].cIn->amplitudeCal;
      }
      powerNow -= powerNow2;
 
      double Vrms = 0.0;
      if (pInput[i].vIn1 == pInput[i].vIn2)
        Vrms = pInput[i].vIn1->Vrms;
      else
        Vrms = 0.0;                                                    // impractical to calculate rms voltage between V1 & V2
      double VA = pInput[i].cIn->Irms * Vrms;

      pInput[i].pf = powerNow / VA;
      if (pInput[i].pf > 1.05 || pInput[i].pf < -1.05 || isnan(pInput[i].pf))
        pInput[i].pf = 0.0;

      pInput[i].realPower = powerNow + 0.5;                            // rounded to nearest Watt

      pInput[i].apparentPower = VA + 0.5;                              // rounded to nearest VA

      double energyNow = powerNow * longProcessing->sampleSets * ADCDuration * samplesPerSet / ADCScale;
      energyNow += pInput[i].residualEnergy;                           // fp for accuracy
      int wattHoursRecent;
      wattHoursRecent = energyNow / 3600;                              // integer assignment to extract whole Wh
      pInput[i].wh += wattHoursRecent;                                 // accumulated WattHours since start-up
      pInput[i].residualEnergy = energyNow - (wattHoursRecent * 3600.0);           // fp for accuracy

    }
    
    // finally the analogue input
    if (analogueInput.enabled)
    {
      analogueInput.count 
        = ((double)longProcessing->cumSampleDeltas[analogueInput.scanPos] 
          / longProcessing->sampleSets) + (ADC_Counts >> 1) + 0.5;  
    }
    else
    {
      analogueInput.count = 0;
    }
    
    memset_v(longProcessing, sizeof(longAccum));
}

bool EmonLibDB_Ready()
{

    countAllPulses();

    if (datalogEventPending) 
    {
        datalogEventPending = false;        
        EmonLibDB_getReadings();
        return true;
    }
    return false;
}


void calcXY(struct phaseData *phData, const uint8_t frequency, const uint16_t sampleInterval, const uint8_t samplesInSet)
{
  /*
  Calculate the samples to choose relative to the present current sample and the interpolation coefficients

    frequency in Hz
    sampleInterval in tenths of µs
    positionOfV & positionOfC in sample set, zero-based
    phaseErrorV & phaseErrorC in degrees

    Samples to use returned in relativeCsample, relativeLastVsample, relativeThisVsample 
      (relative to this current sample, backwards in time is positive)
    Coefficients returned in x & y  - sanity check: x + y ≈ 1.0

    Usage:
    Accumulate "partial powers" PA & PB
      sumPA += sampleI * lastSampleV;
      sumPB += sampleI * thisSampleV;

    Apply combined phase & timing correction
      sumRealPower = sumPA * x + sumPB * y;

    Takes  247 µs (approx)

    y = sin(phase_shift) / sin(sampleLength);
    x = cos(phase_shift) - y * cos(sampleLength);
    For small angles:
    sin(a) ≈ a,   cos(a) ≈ (1 - a²/2)

  */

  double sampleInterval_deg = 360.0 * frequency * sampleInterval * samplesInSet / ADCScale;
  double requiredShiftForV_sets = (phData->phaseErrorC - phData->phaseErrorV) / sampleInterval_deg 
      + (double)(phData->positionOfC - phData->positionOfV) / samplesInSet;

  if (floor(requiredShiftForV_sets) >= 0.0)
  {
    phData->relativeCsample = samplesInSet * (1+floor(requiredShiftForV_sets));
    phData->relativeThisVsample = 0;
  }
  else // (floor(requiredShiftForV_sets) < 0.0)
  {
    phData->relativeCsample = 0;
    phData->relativeThisVsample = samplesInSet * -(1 + floor(requiredShiftForV_sets));
  }

  requiredShiftForV_sets -= floor(requiredShiftForV_sets);
  phData->relativeThisVsample += phData->positionOfC - phData->positionOfV;
  phData->relativeLastVsample = phData->relativeThisVsample + samplesInSet;

  sampleInterval_deg = 360.0 * frequency * sampleInterval * samplesInSet / ADCScale;

  double sampleRate = sampleInterval_deg / 360.0 * TWO_PI; // in radians
  double phase_shift = requiredShiftForV_sets * sampleInterval_deg / 360.0 * TWO_PI;           // Total phase shift in radians
  phData->y = phase_shift / sampleRate;
  phData->x = (1 - phase_shift*phase_shift/2) - phData->y * (1 - sampleRate*sampleRate/2);
}


void populatePIn(uint8_t i)
{
// Transfer data from one or both vInputs & cInput, calculate X & Y phase parameters
  pInput[i].phaseDataV1.positionOfV = pInput[i].vIn1->scanPos;
  pInput[i].phaseDataV1.phaseErrorV = pInput[i].vIn1->phaseCal.lowPhase;
  pInput[i].phaseDataV1.positionOfC = pInput[i].cIn->scanPos;
  pInput[i].phaseDataV1.phaseErrorC = pInput[i].cIn->phaseCal.lowPhase;
  calcXY(&(pInput[i].phaseDataV1), cyclesPerSec, ADCDuration, samplesPerSet);
  if (pInput[i].vIn1 != pInput[i].vIn2)                                // 2nd V for Line-Line measuring
  {
    pInput[i].phaseDataV2.positionOfV = pInput[i].vIn2->scanPos;
    pInput[i].phaseDataV2.phaseErrorV = pInput[i].vIn2->phaseCal.lowPhase;
    pInput[i].phaseDataV2.positionOfC = pInput[i].cIn->scanPos;
    pInput[i].phaseDataV2.phaseErrorC = pInput[i].cIn->phaseCal.lowPhase;
    calcXY(&(pInput[i].phaseDataV2), cyclesPerSec, ADCDuration, samplesPerSet);
  }
}


void allPhaseData(void)
{
  // Calculate phase coefficients for each current/power channel
  for (uint8_t i=0; i<no_of_pInputs; i++)
  {
    populatePIn(i);
  }
}

/**************************************************************************************************
*
*   ADC Interrupt Handling
*
*
***************************************************************************************************/

void newCycleProcessing_withinISR()
{
  /* This routine deals with activities that are only required at the start of
   * each mains cycle or half-cycle.  It forms part of the ISR.
   */ 
  static uint16_t cycleCountForStartup = 0;

  if (polarityConfirmed == POSITIVE) 
  { 
    if (polarityConfirmedOfLastSampleV != POSITIVE)
    {
      cycleCountForStartup++;

      // Discard the first few cycles
      if (firstcycle==true && cycleCountForStartup >= min_startup_cycles)
      {
        firstcycle = false;
        cycleCountForStartup = 0;
        longCollecting->cycles = 0;                           
        memset_v(longCollecting, sizeof(longAccum));
      }
      
      longCollecting->cycles++;

      // Check for end of reporting period
      if (longCollecting->sampleSets >= sampleSetsInDatalogPeriod)     /* report due */ 
      {
        accumSwap(&longCollecting, &longProcessing);
        datalogEventPending = true;
      }
    } // end of processing that is specific to the first Vsample in each +ve half cycle   
  } // end of processing that is specific to samples where the voltage is positive
  
  else // the polarity of this sample is negative
  {     
    if (polarityConfirmedOfLastSampleV != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)

      ;
    } // end of processing that is specific to the first Vsample in each -ve half cycle
  } // end of processing that is specific to samples where the voltage is positive
}
// end of newCycleProcessing_withinISR()


void confirmPolarity()
{
  /* This routine prevents a zero-crossing point16_t from being declared until 
  * a certain number of consecutive samples in the 'other' half of the 
  * waveform have been encountered.  It forms part of the ISR.
  */ 
  volatile static uint8_t count = 0;
  
  if (polarityUnconfirmed != polarityConfirmedOfLastSampleV) 
  { 
    count++; 
  } 
  else 
  {
    count = 0; 
  }

  if (count >= POLARITY_CHECK_MAXCOUNT) 
  {
    count = 0;
    polarityConfirmed = polarityUnconfirmed;
  }
}



// EmonLibDB_interrupt()
//
//   This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished.  In free-running mode, the 
// the ADC has already started its next conversion by the time the ISR is executed.  
// The ISR therefore needs to "look ahead". 
//   At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR 
// which runs at this point16_t therefore needs to capture the results of conversion Type N, 
// and set up the conditions for conversion Type N+2, and so on.  
//   Activities that are required for every new sample are performed here.  Activities
// that are only required at certain stages of the voltage waveform are performed within
// the helper function, newCycleProcessing_withinISR().
//   A second helper function, confirmPolarity() is used to apply a persistence criterion
// when the polarity status of each voltage sample is checked. 
// 
void EmonLibDB_interrupt()  
{                                         
  int16_t volatile ADCsample;
  static uint8_t volatile sample_index = 0;
 
  static uint16_t acSense = 0;

  ADCsample = ADC0.RES;
  uint8_t next = sample_index + 2;                                     // sample_index is the one just retrieved from the ADC; sample_index+1 is the one being converted
  if (next >= samplesPerSet) 
    next -= samplesPerSet;
    
  ADC0.MUXPOS = ADC_Sequence[next].adcInPin;                           // Pre-load the MPX with the next channel to sample
  // Processing common to every sample    
  ADCsample -= (ADC_Counts >> 1);                                      // remove nominal offset (a small offset will remain)
  rawSamples[sampleCount] = ADCsample;

  /*
  For the rms calculation 
  
  Removing the d.c. offset:
  First take off the theoretical (constant) offset to reduce the size of the numbers (as Robin's original method).
  Then accumulate the sum of the resulting values so as to be able at the end of the period to 
  recalculate the true rms based on the rms with the offset and the average remaining offset. The remaining offset 
  should be only a few counts.
  */

  longCollecting->sumSampleSquared[sample_index] += (ADCsample * ADCsample); // cumulative sample^2 
  longCollecting->cumSampleDeltas[sample_index] += ADCsample;

  // deal with activities that are only needed at certain stages of each voltage cycle.
  if (ADC_Sequence[sample_index].adcInPin == 0)                        // use first voltage for defining timing

  {
    // Detect the 1st ac input voltage. This is a 'rough&ready" rectifier/filter, it only needs to be good enough to detect
    //  sufficient voltage to provide assurance that the crossing detector will function properly
    acSense -= acSense >> 2;
    acSense += ADCsample > 0 ? ADCsample : -ADCsample;
    vInput[0].acPresent = acSense > acDetectedThreshold;
    
    if (ADCsample > 0)                                                 // Positive half-cycle
    { 
      polarityUnconfirmed = POSITIVE; 
    }
    else 
    { 
      polarityUnconfirmed = NEGATIVE; 
    }
   
    confirmPolarity();
    newCycleProcessing_withinISR();

    polarityConfirmedOfLastSampleV = polarityConfirmed;                // for identification of half cycle boundaries
  }
  
  if (sample_index >= no_of_vInputs)                                   // It's a current/power
  { 
    
    //  power calcs:
    //  sumPA += sampleI * lastSampleV;
    //  sumPB += sampleI * thisSampleV;
 
    uint16_t cSample     = ((int16_t)sampleCount - ADC_Sequence[sample_index].pInput->phaseDataV1.relativeCsample)     & (RAWSAMPLESSIZE-1);
    uint16_t lastSampleV = ((int16_t)sampleCount - ADC_Sequence[sample_index].pInput->phaseDataV1.relativeLastVsample) & (RAWSAMPLESSIZE-1);
    uint16_t thisSampleV = ((int16_t)sampleCount - ADC_Sequence[sample_index].pInput->phaseDataV1.relativeThisVsample) & (RAWSAMPLESSIZE-1);
    
    longCollecting->sumPA1[sample_index] += (rawSamples[cSample] * rawSamples[lastSampleV]);    // cumulative partial powers 
    longCollecting->sumPB1[sample_index] += (rawSamples[cSample] * rawSamples[thisSampleV]);

    
    if (ADC_Sequence[sample_index].pInput->vIn1 != ADC_Sequence[sample_index].pInput->vIn2)    // Line-Line load in use
    {
      uint16_t cSample     = ((int16_t)sampleCount - ADC_Sequence[sample_index].pInput->phaseDataV2.relativeCsample)     & (RAWSAMPLESSIZE-1);
      uint16_t lastSampleV = ((int16_t)sampleCount - ADC_Sequence[sample_index].pInput->phaseDataV2.relativeLastVsample) & (RAWSAMPLESSIZE-1);
      uint16_t thisSampleV = ((int16_t)sampleCount - ADC_Sequence[sample_index].pInput->phaseDataV2.relativeThisVsample) & (RAWSAMPLESSIZE-1);
      
      longCollecting->sumPA2[sample_index] += (rawSamples[cSample] * rawSamples[lastSampleV]);   // cumulative partial powers 
      longCollecting->sumPB2[sample_index] += (rawSamples[cSample] * rawSamples[thisSampleV]);
    }
  }
  
  sample_index++;
  sampleCount++;
  
  if (sample_index >= samplesPerSet)                                   // this set is done, start again
  {
    
    sample_index = 0;
    longCollecting->sampleSets++; 

  }
  sampleCount &= (RAWSAMPLESSIZE-1);                                   // circular buffer - don't overflow, continue from the beginning.
}

/**************************************************************************************************
*
*   'ADC COMPLETE' ISR
*
*
***************************************************************************************************/
ISR(ADC0_RESRDY_vect) 
{
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  EmonLibDB_interrupt();
}

                         
/**************************************************************************************************
*
*   'PULSE INPUT' ISRs
*
*
***************************************************************************************************/
// The pulse interrupt routines - run each time an edge of a pulse is detected
  

ISR(PORTA_PORT_vect) {
  byte flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; //clear flags
  if (flags & 0x40) {
    pulseInterrupt = true;
  }
  if (flags & 0x80) {
    digitalInterrupt = true;
  }
}

ISR(PORTF_PORT_vect) {  
  PORTF.INTFLAGS = 0x08;  
  analogInterrupt = true;
}


/**************************************************************************************************
*
*   'PULSE INPUT' handler (polled in main loop - not part of ISR)
*
*
***************************************************************************************************/
// Handle pulse de-bounce and count on defined edge

void countAllPulses(void)

{
  if (pulses[0].pulseEnabled & pulseInterrupt)
  {
    pulses[0].timing = true;
    pulses[0].pulseTime = millis();
    pulseInterrupt = false;    
  }
  
  if (pulses[1].pulseEnabled & digitalInterrupt)
  {
    pulses[1].timing = true;
    pulses[1].pulseTime = millis();
    digitalInterrupt = false;    
  }

  if (pulses[2].pulseEnabled & analogInterrupt)
  {
    pulses[2].timing = true;
    pulses[2].pulseTime = millis();
    analogInterrupt = false;    
  }
 
  for (uint8_t channel=0; channel<PULSEINPUTS; channel++)              // Handle pulse de-bounce
  {
    if (pulses[channel].pulseEnabled & pulses[channel].timing)
    {
      uint32_t currentMillis = millis();
      if ((currentMillis - pulses[channel].pulseTime) > pulses[channel].pulseMinPeriod)
      {
        bool newstate;
        switch (channel) 
        {
          case (0): newstate = PORTA.IN & PIN6_bm; break;              //'pulse'
          case (1): newstate = PORTA.IN & PIN7_bm; break;              //'digital'
          case (2): newstate = PORTF.IN & PIN3_bm; break;              //'analogue'
        }
        pulses[channel].timing = false;
        if (pulses[channel].laststate != newstate)
        {
          if ((!newstate && pulses[channel].edge == FALLING)
            || (newstate && pulses[channel].edge == RISING))
          {
            pulses[channel].pulseIncrement++;
          }
          pulses[channel].laststate = newstate;
        }
      }
    }
  }
}
