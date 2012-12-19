#include <sensors.h>
#include <Wire.h>

double accelerometer_data[3];
double gyro_data[3];

void setup() 
{ 
  
   Serial.begin(9600);  
   
   i2cInterface::init();
   SensorsManager::getSensorsManager().init();
}




void loop() 
{
  static double time_elapsed=0.0;
  SensorsManager & sm=SensorsManager::getSensorsManager();    
  sm.update(time_elapsed);
  
  /*
   Serial.print("ACCEL: ");
   Serial.print(sm.getAcceleration()[0]);
   Serial.print("\t");
   Serial.print(sm.getAcceleration()[1]);
   Serial.print("\t");
   Serial.print(sm.getAcceleration()[2]);
   Serial.print("\n");
  
  
   Serial.print("GYRO: ");
   Serial.print(sm.getAngleAcceleration()[0]);
   Serial.print("\t");
   Serial.print(sm.getAngleAcceleration()[1]);
   Serial.print("\t");
   Serial.print(sm.getAngleAcceleration()[2]);
   Serial.print("\n");
   */
     
   Serial.print("NORTH: ");
   Serial.print(sm.getNorth()[0]);
   Serial.print("\t");
   Serial.print(sm.getNorth()[1]);
   Serial.print("\t");
   Serial.print(sm.getNorth()[2]);
   Serial.print("\n");
 
   delay(300); time_elapsed+=300*(10E-6);
}

