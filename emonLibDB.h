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

// Version 1.0.0 6/5/2023 
                                                                           
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

// **** A P P L I C A T I O N   N O T E   A V R 2 0 1 ***************************
// *
// * Title            : 16bit multiply routines using hardware multiplier
// * Version          : V2.0
// * Last updated     : 10 Jun, 2002
// * Target           : Any AVR with HW multiplier
// *
// * Support email    : avr@atmel.com
// *
// * DESCRIPTION
// *   This application note shows a number of examples of how to implement
// *  16bit multiplication using hardware multiplier. Refer to each of the
// *  funtions headers for details. The functions included in this file
// *  are :
// *
// *  muls16x16_32  - Signed multiply of two 16bits numbers with 32bits result.
// *
// ******************************************************************************

// modified as inline assembly in a C header file for the Arduino by Jose Gama, May 2015
//
// Abridged for OEM emonTx4 & emonPi2 by Robert Wall
//  The full version is available as inlineAVR201def.h
// ******************************************************************************
// *
// * FUNCTION
// *  muls16x16_32
// * DECRIPTION
// *  Signed multiply of two 16bits numbers with 32bits result.
// * USAGE
// *  r19:r18:r17:r16 = r23:r22 * r21:r20
// * STATISTICS
// *  Cycles :  19 + ret
// *  Words :    15 + ret
// *  Register usage: r0 to r2 and r16 to r23 (11 registers)
// * NOTE
// *  The routine is non-destructive to the operands.
// *
// ******************************************************************************
#define muls16x16_32(result, multiplicand, multiplier) \
__asm__ __volatile__ ( \
"  clr  r2 \n\t" \
"  muls  %B2, %B1 \n\t" /* (signed)ah * (signed)bh*/ \
"  movw  %C0, r0 \n\t" \
"  mul  %A2, %A1 \n\t" /* al * bl*/ \
"  movw  %A0, r0 \n\t" \
"  mulsu  %B2, %A1 \n\t" /* (signed)ah * bl*/ \
"  sbc  %D0, r2 \n\t" \
"  add  %B0, r0 \n\t" \
"  adc  %C0, r1 \n\t" \
"  adc  %D0, r2 \n\t" \
"  mulsu  %B1, %A2 \n\t" /* (signed)bh * al*/ \
"  sbc  %D0, r2 \n\t" \
"  add  %B0, r0 \n\t" \
"  adc  %C0, r1 \n\t" \
"  adc  %D0, r2 \n\t" \
"  clr r1 \n\t" \
: "=&a" (result) \
: "a" (multiplicand),  "a" (multiplier) \
);


#endif
