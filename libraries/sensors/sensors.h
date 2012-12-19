/** 

* @file sensors.h 
* @brief definicje klas wszystkich sensorow + funkcje pomocnicze
*
* @author --
*
* @date 13/12/2012
*/


#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include <Wire.h>

//stub glownej klasy
class SensorsManager
{
  SensorsManager(){}  //singleton 
public:
  SensorsManager & getSensorsManager(){}
  void update(double t){}
};

class i2cInterface
{
public:
  static void init();
  /** 
  * @brief Pisanie na magistrale i2c, uzywane przez wszystkie czujniki
  */ 
  static void i2c_write(int address, byte reg, byte data);
  /** 
  * @brief Czytanie z magistrali i2c
  */ 
  static void i2c_read(int address, byte reg, int count, byte* data);  
};


/** 
* @class Sensor
* @brief Interfejs sensora
*/ 
class Sensor
{
public:
       Sensor(){}
       virtual void init()=0;
       /**
       * @brief Odczyt
       * @param double * tablica o wymiarowosci odczytu (np akcelerometr ma 3 osie)
       */ 
       virtual void get_readings(double *) =0;
};



//moze model singleton?
class ADXL345Sensor: public Sensor
{
      double m_update_freq; //co ile samplowac [ms]
      double m_data_range; //+ - ilosc g
       static const int ADXL345_ADDRESS=(0xA6>> 1);
       static const int ADXL345_REGISTER_XLSB = (0x32);
       static const int ADXL_REGISTER_PWRCTL =(0x2D);
       static const int ADXL_PWRCTL_MEASURE =(1 << 3);
       static const int ADXL_DATA_FORMAT =(0x31);
       static const double ADXL_MAX_RANGE = 10.0;
      static const double ADXL_MAX_READ=2000;    
      
       
public:
      ADXL345Sensor();
      void init();
      void get_readings(double * data);
}; 
 
 
 #define ITG3200_ADDRESS (0xD0 >> 1)
//request burst of 6 bytes from this address
#define ITG3200_REGISTER_XMSB (0x1D)
#define ITG3200_REGISTER_DLPF_FS (0x16)
#define ITG3200_FULLSCALE (0x03 << 3)
#define ITG3200_42HZ (0x03)
#define ITG3200_MAX_READ 10000

void init_itg3200();

class Magnetometer
{
private:
  static const int HMC5843_ADDRESS = (0x3C >> 1);
  static const int HMC5843_REGISTER_XMSB = (0x03);
  static const int HMC5843_REGISTER_MEASMODE = (0x02);
  static const int HMC5843_MEASMODE_CONT = (0x00);

public:
  int data[6];

  Magnetometer();

  void read();
};

#endif
