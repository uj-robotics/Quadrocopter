//TODO: ogarnac roznice ATMega32 vs ARM
//TODO: zaklepac przerwania na ARM
//TODO: nauczyc sie I2C, zeby opowiedziec


#include <Wire.h>
#include <DueTimer.h>
#include "reference_frame.h"
#include "sensors.h"


double time_elapsed;

unsigned int LED_RED=4;
unsigned int LED_GREEN=2;

// Updates (in interruption) readings for sensors //
void update_readings(){
    time_elapsed+=500*(1E-6);
    
    ReferenceFrame::getReferenceFrame().update(time_elapsed);
    
    digitalWrite(LED_RED, digitalRead(LED_RED)^1);
}

void setup() 
{ 
   pinMode(LED_RED, OUTPUT);
   pinMode(LED_GREEN, OUTPUT);
   time_elapsed = 0.0;
 
   Serial.begin(9600);     
   
   // == Init I2C Interface == //
   i2cInterface::init();
   
   // == Init SensorManager == //
   SensorsManager::getSensorsManager().init();
   SensorsManager::getSensorsManager().OnePointCallibration();

   // == Init ReferenceFrame == //
   ReferenceFrame::getReferenceFrame().init();
   
   // == Add interruption for sensor readings == //
   Timer.getAvailable().attachInterrupt(update_readings).start(500000); // Every 500ms update sensor readings

   // == Inicjalizacja zakonczona pomyslnie == //
   digitalWrite(LED_GREEN, HIGH);
}





void loop() 
{
   double * error = ReferenceFrame::getReferenceFrame().getError();
   
   Serial.print(error[0]);
   Serial.print(" ");
   Serial.print(error[1]);
   Serial.print(" ");
   Serial.print(error[2]); 
   Serial.print("\n");
   delay(300);
   
   
   /*Serial.print(refFrame->x_measure);
   Serial.print(" ");
   Serial.print(refFrame->y_measure);
   Serial.print(" ");
   Serial.print(refFrame->z_measure); 
   Serial.print("\n");*/
   delay(300);
}

