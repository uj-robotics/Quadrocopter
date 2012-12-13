#include <Wire.h>
double accelerometer_data[3];
#include "sensors.h"


ADXL345Sensor accSensor;

void setup() {    
   Serial.begin(9600);  
   i2cInterface::init();
   accSensor.init();
   init_itg3200();
   
}



int gyro_data[3];

void read_itg3200() {
  byte bytes[6];
  memset(bytes,0,6);

  //read 6 bytes from the ITG3200
  i2cInterface::i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, bytes);  //now unpack the bytes
  for (int i=0;i<3;++i) {
  gyro_data[i] = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
  }
}
void loop() {
   accSensor.get_readings(accelerometer_data);
   Serial.print("ACCEL: ");
   Serial.print(accelerometer_data[0]);
   Serial.print("\t");
   Serial.print(accelerometer_data[1]);
   Serial.print("\t");
   Serial.print(accelerometer_data[2]);
   Serial.print("\n");
  
   read_itg3200();

   Serial.print("GYRO: ");
   Serial.print(gyro_data[0]);
   Serial.print("\t");
   Serial.print(gyro_data[1]);
   Serial.print("\t");
   Serial.print(gyro_data[2]);
   Serial.print("\n");
   delay(300);
}