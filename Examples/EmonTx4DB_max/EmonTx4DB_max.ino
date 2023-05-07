/*

"Maximal" sketch to demonstrate emonLibDB

This demonstrates the use of every API function. It is not suitable for immediate use, 
rather it is basis for experimentation and development of sketches to suit particular needs.

This sketch provides an example of every Application Interface function. 
Many in fact set the default value for the emonTx4, and are therefore
not needed in most cases. If you do need to change a value, refer to the 
Application Interface section of the User Documentation (PDF file) for full details.

For more information about application to three-phase and split-phase monitoring, refer to

https://docs.openenergymonitor.org/electricity-monitoring/ac-power-theory/3-phase-power.html
and
https://docs.openenergymonitor.org/electricity-monitoring/ac-power-theory/use-in-north-america.html

Released to accompany emonLibDB, version 1.0.0  6/5/2023 

*/

#define Serial Serial3
#include <Arduino.h>

#include "emonLibDB.h"


const byte LEDpin = PIN_PB2;                   // emonTx V4 LED
uint32_t counter = 0;

bool recalibrate = false;                      //  Do not demonstrate the recalibration functions

void setup() 
{  

  Serial.begin(9600);
  Serial.println("Set baud=115200");
  Serial.end();
  Serial.begin(115200);
  
  Serial.println("\nEmonTx4 EmonLibCM Continuous Monitoring Maximal Demo"); 
  Serial.print("\nValues will be reported every ");  
    Serial.print(EmonLibDB_getDatalogPeriod()); Serial.println(" seconds");
 
  /****************************************************************************
  *                                                                           *
  * Values for timing & calibration of the ADC                                *
  *                                                                           *
  *   Users in the 60 Hz world must change "cycles_per_second"                *
  *                                                                           *
  *   (These must come before the sensors are defined)                        *
  *                                                                           *
  ****************************************************************************/

  EmonLibDB_cyclesPerSecond(50);             // Nominal Line frequency - 50 Hz or 60 Hz
  EmonLibDB_minStartupCycles(10);            // number of cycles to let ADC run before starting first actual measurement
  EmonLibDB_datalogPeriod(9.8);              // period of readings in seconds - normal value for emoncms.org
  EmonLibDB_ADCCal(1.024);                   // ADC Reference voltage
  EmonLibDB_fCal(1.00);                      // Correction for processor clock tolerances

 
  /****************************************************************************
  *                                                                           *
  * Set the properties of the physical sensors                                *
  *                                                                           *
  ****************************************************************************/
 
  EmonLibDB_set_vInput(1, 100.0, 0.16);        // emonVS Input channel 1, voltage calibration 100, phase error 0.16°
  EmonLibDB_set_vInput(2, 100.0, 0.16);        // emonVS Input channel 2, voltage calibration 100, phase error 0.16°
  EmonLibDB_set_vInput(3, 100.0, 0.16);        // emonVS Input channel 3, voltage calibration 100, phase error 0.16°
                                               //  (All 3 may be set, even if those inputs are unused)

  EmonLibDB_set_cInput(1, 100.0, 0.3);         // emonTx4 Current/power Input channel 1, 0.3° phase error for 100 A CT
  EmonLibDB_set_cInput(2,  50.0, 0.3);         // emonTx4 Current/power Input channel 2, 0.3° phase error for  50 A CT
  EmonLibDB_set_cInput(3,  20.0, 0.3);         // emonTx4 Current/power Input channel 3, 0.3° phase error for  20 A CT
  EmonLibDB_set_cInput(4, 100.0, 0.3);         //  (likewise for current/power Inputs 4 - 12)
  EmonLibDB_set_cInput(5, 100.0, 0.3);
  EmonLibDB_set_cInput(6, 100.0, 0.3);
  EmonLibDB_set_cInput(7, 100.0, 0.3);
  EmonLibDB_set_cInput(8, 100.0, 0.3);
  EmonLibDB_set_cInput(9, 100.0, 0.3);
  EmonLibDB_set_cInput(10, 100.0, 0.3);
  EmonLibDB_set_cInput(11, 100.0, 0.3);
  EmonLibDB_set_cInput(12, 100.0, 0.3);


  /****************************************************************************
  *                                                                           *
  * Link voltage and current sensors to define the power                      *
  *  & energy measurements                                                    *
  *                                                                           *
  * For best precision and performance, include only the following lines      *
  *    that apply to current/power inputs being used.                         *
  *                                                                           *
  ****************************************************************************/

  EmonLibDB_set_pInput(1, 1);                  // CT1, V1
  EmonLibDB_set_pInput(2, 1);                  // CT2, V2 (etc)
  EmonLibDB_set_pInput(3, 1);
  EmonLibDB_set_pInput(4, 1);  
  EmonLibDB_set_pInput(5, 1);
  EmonLibDB_set_pInput(6, 1);

  EmonLibDB_set_pInput(7, 1);                  // CT7, V1  
  EmonLibDB_set_pInput(8, 2);                  // CT8, V2
  EmonLibDB_set_pInput(9, 3);                  // CT9, V3 (etc)
  EmonLibDB_set_pInput(10, 1);  
  EmonLibDB_set_pInput(11, 2);
//  EmonLibDB_set_pInput(12, 3);


  /* How to measure Line-Line loads 
      - either 3-phase or single phase, split phase:
  */
/*
  EmonLibDB_set_pInput(1, 1, 2);               // CT1 between V1 & V2    
  EmonLibDB_set_pInput(2, 2, 3);               // CT2 between V2 & V3
  EmonLibDB_set_pInput(3, 3, 1);               // CT3 between V3 & V1  (etc)  
  EmonLibDB_set_pInput(4, 1, 2);  
  EmonLibDB_set_pInput(5, 2, 3);
  EmonLibDB_set_pInput(6, 3, 1);

  EmonLibDB_set_pInput(7, 1, 2);  
  EmonLibDB_set_pInput(8, 2, 3);
  EmonLibDB_set_pInput(9, 3, 1);
  EmonLibDB_set_pInput(10, 1, 2);  
  EmonLibDB_set_pInput(11, 2, 3);
  EmonLibDB_set_pInput(12, 3, 1);
*/

  /****************************************************************************
  *                                                                           *
  * Pulse Counting                                                            *
  *                                                                           *
  * Pulse counting on any channel is only available if the appropriate        *
  * solder link is made on the hardware, and the related "TMP" link is        *
  * broken.                                                                   *
  * The 'Analogue' input is not available if the extender card is fitted.     *
  ****************************************************************************/
 
  EmonLibDB_setPulseEnable(true);              // Enable counting on "Pulse" input
  EmonLibDB_setPulseMinPeriod(20);             // Contact bounce must not last longer than 20 ms
  EmonLibDB_setPulseCount(0);                  // Initialise to pulse count to zero

  EmonLibDB_setPulseEnable(Dig, false);        // Disable counting on "Digital" input
  EmonLibDB_setPulseMinPeriod(2, 20, FALLING); // Contact bounce must not last longer than 20 ms, trigger on the falling edge

  EmonLibDB_setPulseEnable(ADC, false);        // Disable counting on "Analog" input
  EmonLibDB_setPulseMinPeriod(3, 0, RISING);   // No contact bounce expected, trigger on the rising edge
  EmonLibDB_setPulseCount(3, 123);             // Initialise the pulse count for this to 123 counts

  
  /****************************************************************************
  *                                                                           *
  * Analogue Input                                                            *
  *                                                                           *
  * The 'Analogue' input is not available if the extender card is fitted.     *
  *                                                                           *
  ****************************************************************************/
  
  EmonLibDB_setAnalogueEnable(false);          // Disable analogue input (cannot be used when the extender card is fitted.

  
  /****************************************************************************
  *                                                                           *
  * Pre-set Energy counters                                                   *
  *                                                                           *
  ****************************************************************************/
  EmonLibDB_setWattHour(1, 0);                 // Wh counter set to zero
  EmonLibDB_setWattHour(2, 0);                 //  (likewise for all 12 channels)

  EmonLibDB_Init();                            // Start continuous monitoring.
}

void loop()             
{

  if (recalibrate)                                         // recalibrate should be set when new calibration values become available
  {
      
    EmonLibDB_reCalibrate_vInput(1, 100.12, 0.18);         // emonVS Input channel 1, new values for voltage cal & phase error 
    EmonLibDB_reCalibrate_cInput(3, 50, 0.28);             // Current Input channel 3, new values for current cal & phase error

    recalibrate = false;                                   // Do it once only.
  }
  
  if (EmonLibDB_Ready())   
  {

    delay(100);
    Serial.print("\nReport "); delay(20); Serial.println(++counter); 
    Serial.print(EmonLibDB_acPresent()?"AC present ":"AC missing ");
    delay(50);

    Serial.print("V1 = ");Serial.print(EmonLibDB_getVrms(1));
    Serial.print(" V2 = ");Serial.print(EmonLibDB_getVrms(2));
    if (EmonLibDB_getVinputInUse(3))                                   // if Voltage 3 is being used...
      Serial.print(" V3 = ");Serial.print(EmonLibDB_getVrms(3));
    Serial.print(" f = ");Serial.println(EmonLibDB_getLineFrequency());
    
    Serial.print("Pulses1 = "); Serial.print(EmonLibDB_getPulseCount(1));
    Serial.print("  Pulses2 = "); Serial.print(EmonLibDB_getPulseCount(2));
    Serial.print("  Pulses3 = "); Serial.println(EmonLibDB_getPulseCount(3));
    
    Serial.print("Analog in = "); Serial.println(EmonLibDB_getAnalogueCount()); 
    
    for (uint8_t ch=1; ch<=12; ch++)
    {
      if (EmonLibDB_getCinputInUse(ch))
      {
        Serial.print("Ch ");Serial.print(ch);
        Serial.print(" I=");Serial.print(EmonLibDB_getIrms(ch),3);
        Serial.print(" W=");Serial.print(EmonLibDB_getRealPower(ch));
        Serial.print(" VA=");Serial.print(EmonLibDB_getApparentPower(ch));
        Serial.print(" Wh=");Serial.print(EmonLibDB_getWattHour(ch));
        Serial.print(" pf=");Serial.print(EmonLibDB_getPF(ch),4);      
        Serial.println();
        delay(20);
      }
    }   
    
    delay(50);
  
  }
}
