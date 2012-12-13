#include "sensors.h"

void i2cInterface::init(){
  Wire.begin();
}

//Funkcje pomocnicze do obslugi magistrali IC2
void i2cInterface::i2c_write(int address, byte reg, byte data) {
  // Send output register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  // Connect to device and send byte
  Wire.write(data); // low byte
  Wire.endTransmission();
}
void i2cInterface::i2c_read(int address, byte reg, int count, byte* data) {
 int i = 0;

 // Send input register address
 Wire.beginTransmission(address);
 Wire.write(reg);
 Wire.endTransmission();
 // Connect to device and request bytes
 Wire.beginTransmission(address);
 Wire.requestFrom(address,count);
 while(Wire.available()){ // slave may send less than requested 
   char c = Wire.read(); // receive a byte as character
   data[i] = c;
   i++;
 }
 Wire.endTransmission();
}


const int ADXL345Sensor::ADXL345_ADDRESS=(0xA6>> 1);
const int ADXL345Sensor::ADXL345_REGISTER_XLSB = (0x32);
const int ADXL345Sensor::ADXL_REGISTER_PWRCTL =(0x2D);
const int ADXL345Sensor::ADXL_PWRCTL_MEASURE =(1 << 3);
const int ADXL345Sensor::ADXL_DATA_FORMAT (0x31);
const double ADXL345Sensor::ADXL_MAX_RANGE = 10.0;
const double ADXL345Sensor::ADXL_MAX_READ=256.0;     

#include <Wire.h>
#include "sensors.h"


void ADXL345Sensor::get_readings(double * data) {
 byte bytes[6];
 memset(bytes,0,6);

 //read 6 bytes from the ADXL345
 i2cInterface::i2c_read(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL345_REGISTER_XLSB, 6, bytes);
 double scale = m_data_range/(1024.0);
 //now unpack the bytes
 for (int i=0;i<3;++i) {   
   data[i] = ((int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8))*scale;
   data[i] = data[i]>ADXL345Sensor::ADXL_MAX_RANGE ? (data[i]-ADXL345Sensor::ADXL_MAX_READ) : data[i];
 }
}

 ADXL345Sensor::ADXL345Sensor(): m_update_freq(0.0001), m_data_range(4)
 {}
 void ADXL345Sensor::init(){
    byte data = 0;
    i2cInterface::i2c_write(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL_DATA_FORMAT, 0x00); //wybierz typ formatu danych (dokladnosc)
    i2cInterface::i2c_write(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL_REGISTER_PWRCTL, ADXL345Sensor::ADXL_PWRCTL_MEASURE); //wlaczenie w prawidlowy tr
    i2cInterface::i2c_read(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL_REGISTER_PWRCTL, 1, &data);
 }
 void ADXL345Sensor::get_readings(double * acc);






void init_itg3200() {
  byte data = 0;

  //Set DLPF to 42 Hz (change it if you want) and
  //set the scale to "Full Scale"
  i2cInterface::i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_42HZ);

  //Sanity check! Make sure the register value is correct.
  i2cInterface::i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, 1, &data);

  Serial.println((unsigned int)data);
}


