/*

"Minimal" sketch to demonstrate emonLibDB

This demonstrates the the simplest sketch for a basic 6-channel emonTx4 with single-phase emonVs

It is intended as abasis for experimentation and development of sketches to suit particular needs.

If you do need to change a value, or add additional features, refer to the 
Application Interface section of the User Documentation (PDF file) for full details, and to the
accompanying "Maximal" and "RF" sketches for examples.

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
  
  Serial.println("\nEmonTx4 EmonLibCM Continuous Monitoring Minimal Demo"); 
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
  EmonLibDB_datalogPeriod(9.8);              // period of readings in seconds - normal value for emoncms.org

 
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
  EmonLibDB_set_cInput(4, 100.0, 0.3);         //  (likewise for current/power Inputs 4 - 6)
  EmonLibDB_set_cInput(5, 100.0, 0.3);
  EmonLibDB_set_cInput(6, 100.0, 0.3);


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

  
  /****************************************************************************
  *                                                                           *
  * Analogue Input                                                            *
  *                                                                           *
  * The 'Analogue' input is not available if the extender card is fitted.     *
  *                                                                           *
  ****************************************************************************/
  
  EmonLibDB_setAnalogueEnable(true);           // Enable analogue input (cannot be used when the extender card is fitted.

  EmonLibDB_Init();                            // Start continuous monitoring.
}

void loop()             
{

  if (EmonLibDB_Ready())   
  {

    delay(100);
    Serial.print("\nReport "); delay(20); Serial.println(++counter); 
    Serial.print(EmonLibDB_acPresent()?"AC present ":"AC missing ");
    delay(50);

    Serial.print("V1 = ");Serial.print(EmonLibDB_getVrms(1));
    Serial.print(" f = ");Serial.println(EmonLibDB_getLineFrequency());
    
    Serial.print("Pulses = "); Serial.print(EmonLibDB_getPulseCount(1));
    
    Serial.print(" Analog in = "); Serial.print(EmonLibDB_getAnalogueCount()); 
    Serial.println();
    
    for (uint8_t ch=1; ch<=6; ch++)
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
