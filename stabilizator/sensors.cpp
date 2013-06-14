#include "Arduino.h"
#include "sensors.h"
#include <Wire.h>

void i2cInterface::init()
{
  Wire.begin();
}

//Funkcje pomocnicze do obslugi magistrali IC2
void i2cInterface::i2c_write(int address, 
							 byte reg, 
							 byte data) 
{
  // Send output register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  // Connect to device and send byte
  Wire.write(data); // low byte
  Wire.endTransmission();
}

void i2cInterface::i2c_read(int address, 
							byte reg, 
							int count, 
							byte* data) 
{
 int i = 0;

 // Send input register address
 Wire.beginTransmission(address);
 Wire.write(reg);
 Wire.endTransmission();
 
 // Connect to device and request bytes
 Wire.beginTransmission(address);
 Wire.requestFrom(address,count);
 
 while(Wire.available()) // slave may send less than requested 
 { 
   char c = Wire.read(); // receive a byte as character
   data[i] = c;
   i++;
 }
 Wire.endTransmission();
}

  


void ADXL345Sensor::get_readings(double * data) 
{
  byte bytes[6];
  
  
  memset(bytes,0,6);

  //read 6 bytes from the ADXL345
  i2cInterface::i2c_read(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL345_REGISTER_XLSB, 6, bytes);



  //now unpack the bytes

  int tmp=0;
  short int tmp2=0;
  for (int i=0;i<3;++i) 
  {   
    tmp2=((int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8));//*scale;
    data[i]=(double)tmp2;//*scale;
  }
  
}

 ADXL345Sensor::ADXL345Sensor(): m_update_freq(0.0001), m_data_range(4)
 {}
 
 
 
 
 void ADXL345Sensor::init()
 {
    byte data = 0;
    i2cInterface::i2c_write(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL_DATA_FORMAT, 0x00); //wybierz typ formatu danych (dokladnosc)
    i2cInterface::i2c_write(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL_REGISTER_PWRCTL, ADXL345Sensor::ADXL_PWRCTL_MEASURE); //wlaczenie w prawidlowy tr
    i2cInterface::i2c_read(ADXL345Sensor::ADXL345_ADDRESS, ADXL345Sensor::ADXL_REGISTER_PWRCTL, 1, &data);
 }


  
  
  ITG3200Sensor::ITG3200Sensor(): m_update_freq(0.0001), m_data_range(2000){}


void ITG3200Sensor::init()
{    
  byte data = 0;
  //Set DLPF to 42 Hz (change it if you want) and
  //set the scale to "Full Scale"
  i2cInterface::i2c_write(ITG3200Sensor::ITG3200_ADDRESS, ITG3200Sensor::ITG3200_REGISTER_DLPF_FS, ITG3200Sensor::ITG3200_FULLSCALE | ITG3200Sensor::ITG3200_42HZ);
  //Sanity check! Make sure the register value is correct.
  i2cInterface::i2c_read(ITG3200Sensor::ITG3200_ADDRESS, ITG3200Sensor::ITG3200_REGISTER_DLPF_FS, 1, &data);
}

void ITG3200Sensor::get_readings(double * data) 
{
  byte bytes[6];
  memset(bytes,0,6);
  //read 6 bytes from the ITG3200
  //now unpack the bytes
  i2cInterface::i2c_read(ITG3200Sensor::ITG3200_ADDRESS, 
  
                         ITG3200Sensor::ITG3200_REGISTER_XMSB, 
                         6, 
                         &bytes[0]);  

  for (int i=0; i<3; ++i) {
    short int tmp = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
    
    data[i]=tmp;///(14.375);
  }

}

//Ultrasonar MB1260

void MB1260Sensor::init()
{
	pinMode(this->MB1260_PULSE_WIDTH_PIN, INPUT);
}

double MB1260Sensor::get_update_freq() const
{
	return this->m_update_freq;
}

void MB1260Sensor::get_readings(double* data)
{
	int pulse = pulseIn(this->MB1260_PULSE_WIDTH_PIN, HIGH);
	*data = static_cast<double>(pulse) / 58;
}


 
 
SensorsManager::SensorsManager(){
    for(int i=0;i<m_sensors;++i) m_last_update[i]=0;
} 
 
SensorsManager& SensorsManager::getSensorsManager(){
               static SensorsManager sm;
               return sm;
}


/*
* @brief Inicjalizacja wszystkich zmiennych
*/
void SensorsManager::init(){
     m_acc.init(); 
     m_gyro.init();  
     m_compass.init(); 
}


/*
* @brief funkcja robi update jesli przekracza czas od ostatniego updatu (specyficzny czas samplowania dla kazdego czujnika
* chwilowo taki sam)
*/
void SensorsManager ::update(double t){
     //akcelerometr:
    // if(t-m_last_update[0] > m_acc.get_update_freq()){ 
                           m_last_update[0]=t; 
                           m_acc.get_readings(m_raw_data[0]); 
                           for(int i=0;i<3;++i) m_data[0][i] = (m_raw_data[0][i] - m_acc_params[i][0])*m_acc_params[i][1];
     //}
     //zeroskop:
    // if(t-m_last_update[1] > m_gyro.get_update_freq()){ 
                           m_last_update[1]=t; 
                           m_gyro.get_readings(m_raw_data[1]);
                           for(int i=0;i<3;++i) m_data[1][i] = (m_raw_data[1][i] - m_gyro_params[i][0])*m_gyro_params[i][1];
    // }
     
     //if(t-m_last_update[2] > m_compass.get_update_freq()){ 
                           m_last_update[2]=t; m_compass.get_readings(m_raw_data[2]); 
                           for(int i=0;i<3;++i) m_data[2][i] = (m_raw_data[2][i] - m_compass_params[i][0])*m_compass_params[i][1];
     //}
     
}



  HMC5843Sensor::HMC5843Sensor(): m_update_freq(0.001)
  {

  }
  void HMC5843Sensor::init(){
    i2cInterface::i2c_write( HMC5843Sensor::HMC5843_ADDRESS, HMC5843Sensor::HMC5843_REGISTER_MEASMODE, HMC5843Sensor::HMC5843_MEASMODE_CONT);
  }
  
  void HMC5843Sensor::get_readings(double * data)
  {
     byte bytes[6];
     memset(bytes,0,6);

      //read 6 bytes from the HMC5843
     i2cInterface::i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_XMSB, 6, bytes);
     short int tmp;
      //now unpack the bytes
     for (int i=0;i<3;++i) {
      tmp = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
      data[i] = (double)tmp;
     }
  }

 

