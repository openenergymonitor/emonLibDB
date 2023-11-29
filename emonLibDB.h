/*
  emonLibDB.h - Library for openenergymonitor
  GNU GPL
*/

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
// Jörg Becker (@joergbecker32) for his background work on interrupts and the ADC; 
// @ursi (Andries) and @mafheldt (Mike Afheldt) for suggestions made at 
//  https://community.openenergymonitor.org/t/emonlib-inaccurate-power-factor/3790 and
//  https://community.openenergymonitor.org/t/rms-calculations-in-emonlib-and-learn-documentation/3749/3; 
// @awjlogan for his suggestions regarding memory use;
// @cbmarkwardt & @dBC for his suggestion to use the AVR Hardware Multiplier.

// Version 1.0.0 6/5/2023 Initial public release
// Version 1.0.1 25/11/2023  Hardware 'Fast Multiply' removed - found to be slower. 
//   ADC0SampLenLineNeutral was 21, ADC0SampLenLineLine was 19. No change to user interface. 
//   Very minor changes to documentation to reflect faster sampling rates.
                                                                           
#ifndef EmonLibDB_h
#define EmonLibDB_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif


void EmonLibDB_cyclesPerSecond(uint8_t _cycles_per_second);
void EmonLibDB_minStartupCycles(uint8_t _min_startup_cycles);
void EmonLibDB_datalogPeriod(float _datalog_period_in_seconds);
void EmonLibDB_ADCCal(double _RefVoltage);
void EmonLibDB_fCal(double _frequencyCal);
void EmonLibDB_set_vInput(uint8_t input, double _amplitudeCal, double _phase);
void EmonLibDB_set_cInput(uint8_t input, double _amplitudeCal, double _phase);
void EmonLibDB_reCalibrate_vInput(uint8_t input, double _amplitudeCal, double _phase);
void EmonLibDB_reCalibrate_cInput(uint8_t input, double _amplitudeCal, double _phase);
void EmonLibDB_set_pInput(uint8_t ADC_InputI, uint8_t ADC_InputV1);
void EmonLibDB_set_pInput(uint8_t ADC_InputI, uint8_t ADC_InputV1, uint8_t ADC_InputV2);
void EmonLibDB_setWattHour(uint8_t input, long _wh);

void EmonLibDB_setPulseCount(uint32_t _pulseCount);
void EmonLibDB_setPulseCount(uint8_t input, uint32_t _pulseCount);
void EmonLibDB_setPulseEnable(uint8_t input, bool _enable);
void EmonLibDB_setPulseEnable(bool _enable);
void EmonLibDB_setPulseMinPeriod(uint16_t _period);
void EmonLibDB_setPulseMinPeriod(uint8_t input, uint16_t _period, uint8_t _edge=FALLING);
void EmonLibDB_setAnalogueEnable(bool _enable);

bool EmonLibDB_acPresent(int8_t input = 1);
int16_t EmonLibDB_getRealPower(int8_t input);
int16_t EmonLibDB_getApparentPower(int8_t input);
double EmonLibDB_getPF(int8_t input);
double EmonLibDB_getIrms(uint8_t input);
double EmonLibDB_getVrms(uint8_t input);
bool EmonLibDB_getVinputInUse(uint8_t input);
bool EmonLibDB_getCinputInUse(uint8_t input);
double EmonLibDB_getDatalogPeriod(void);
double EmonLibDB_getLineFrequency(void);
int32_t EmonLibDB_getWattHour(int8_t input);
uint32_t EmonLibDB_getPulseCount(void);
uint32_t EmonLibDB_getPulseCount(uint8_t input);
uint16_t EmonLibDB_getAnalogueCount(void);

void EmonLibDB_Init();
void EmonLibDB_Start();
bool EmonLibDB_Ready();

// General calculations - used internally 

void allPhaseData(void);
void populatePIn(uint8_t i);
void calcXY(struct phaseData *phData, const uint8_t frequency, const uint16_t sampleInterval, const uint8_t samplesInSet);
void calcPhaseShift(uint8_t lChannel);

// Pulse debounce
void countAllPulses(void);

enum PulseIn {Pulse=1, Dig, ADC};


#endif
