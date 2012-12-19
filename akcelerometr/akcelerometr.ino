#include <sensors.h>
#include <Wire.h>

double accelerometer_data[3];
double gyro_data[3];

ADXL345Sensor accSensor;


void setup() 
{ 
  
   Serial.begin(9600);  
   
   i2cInterface::init();
   accSensor.init();
   
   init_itg3200();   
}

void read_itg3200() 
{
  byte bytes[6];
  memset(bytes,0,6);

  //read 6 bytes from the ITG3200
  //now unpack the bytes
  i2cInterface::i2c_read(ITG3200_ADDRESS, 
  
                         ITG3200_REGISTER_XMSB, 
                         6, 
                         &bytes[0]);  
  
  double scale = 2000.0/double(2<<15);
  for (int i=0; i<3; ++i) {
    short int tmp = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
    
    gyro_data[i]=tmp/(14.375);//*scale;
  }
}

void loop() 
{
  
   accSensor.get_readings(accelerometer_data);
   Serial.print("ACCEL: ");
   Serial.print(accelerometer_data[0]);
   Serial.print("\t");
   Serial.print(accelerometer_data[1]);
   Serial.print("\t");
   Serial.print(accelerometer_data[2]);
   Serial.print("\n");
  
   Serial.print((2<<13));
  
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
