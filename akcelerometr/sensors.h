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
       virtual double get_update_freq() const=0;
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
      double get_update_freq() const{ return m_update_freq; }
      void get_readings(double * data);
};


class HMC5843Sensor: public Sensor
{
  double m_update_freq;
  static const int HMC5843_ADDRESS = (0x3C >> 1);
  static const int HMC5843_REGISTER_XMSB = (0x03);
  static const int HMC5843_REGISTER_MEASMODE = (0x02);
  static const int HMC5843_MEASMODE_CONT = (0x00);

public:
  int data[6];
  double get_update_freq() const{ return m_update_freq; }
  HMC5843Sensor();
  void init();
  void get_readings(double * data);
};

class ITG3200Sensor: public Sensor
{
      double m_update_freq; //co ile samplowac [ms]
      double m_data_range; //+ - ilosc g
      
      static const int ITG3200_ADDRESS =  (0xD0 >> 1);
      static const int ITG3200_REGISTER_XMSB = (0x1D);
      static const int ITG3200_REGISTER_DLPF_FS = (0x16);
      static const int  ITG3200_FULLSCALE =(0x03 << 3);
      static const int  ITG3200_42HZ=(0x03);
      static const int ITG3200_MAX_READ=10000; 
      
       
public:
      ITG3200Sensor();
      void init();
      double get_update_freq() const{ return m_update_freq; }
      void get_readings(double * data);
};

class MB1260Sensor : public Sensor
{
public:
	MB1260Sensor(): m_update_freq(0.1) {}
	void init();
	double get_update_freq() const;
	void get_readings(double* data);

private:
	double m_update_freq;
	static const int MB1260_PULSE_WIDTH_PIN = 7;
};

//TODO: rozbic SensorsManager na akcelerometr/gyroskop.. itp (ktore beda mialy ws obie np "ITG3200Sensor")), one powinny
//zajmowac sie swoja kalibracja w srodku i zwracac juz skalibrowane wyniki
class SensorsManager
{
  SensorsManager();  //singleton 
  static const int m_sensors=3;
  ITG3200Sensor m_gyro; ADXL345Sensor m_acc; HMC5843Sensor m_compass;
  
  
  
  double m_last_update[4];
  double m_raw_data[4][3];
  double m_data[4][3];
  
  
  
  //mozna wrzucic do klasy obslugujacej akcelerometr bezposrednio, to w przyszlosci
  double m_acc_params[3][2]; //zakres i zero (ustawiane przez OnePointCallibration(), potem przez gettery)
  double m_gyro_params[3][2];
  double m_compass_params[3][2];
public:
  //trywialna kalibracja, mierzy zero majac skale. Do zrobienia : regresja liniowa . O dziwo kompas jest spoko skalibrowany.
  //Sonar tez o ile jest tuba
  void OnePointCallibration(){
       m_acc_params[0][0] = 0.0;
       m_acc_params[0][1] = 4.0/1024.0; //skala akcelerometru
       m_acc_params[1][0]=0.0;
       m_acc_params[1][1] = 4.0/1024.0; 
       m_acc_params[2][0]=0.0;
       m_acc_params[2][1]=4.0/1024.0;


       m_gyro_params[0][0] = 0.0;
       m_gyro_params[0][1] = 1/14.375; 
       m_gyro_params[1][0]=0.0;
       m_gyro_params[1][1] = 1/14.375; //skala zyroskopu
       m_gyro_params[2][0]=0.0;
       m_gyro_params[2][1]=1/14.375; 
       
       m_compass_params[0][0] = 0.0;
       m_compass_params[0][1] = 1.0;
       m_compass_params[1][0]=0.0;
       m_compass_params[1][1] = 1.0;
       m_compass_params[2][0]=0.0;
       m_compass_params[2][1]=1.0; //skala kompasu          
       
       double max_latency = 0.1; //[miliseconds]
       double readings[3][3]; 
       
       for(int i=0;i<3;++i) {delay(max_latency*1000); m_acc.get_readings(readings[i]);}
       for(int i=0;i<3;++i) m_acc_params[i][0] = (readings[0][i]+readings[1][i]+readings[2][i])/3.0;
       m_acc_params[2][0]-=(1.0/m_acc_params[2][1]); //+1g na Z     

       for(int i=0;i<3;++i) {delay(max_latency*1000); m_gyro.get_readings(readings[i]);}
       for(int i=0;i<3;++i) m_gyro_params[i][0] = (readings[0][i]+readings[1][i]+readings[2][i])/3.0;
      
       
  }
  void init();
   const double * getNorth() const{ return m_data[2]; }
  const double * getAngleAcceleration() const{ return m_data[1];}
  const double * getAcceleration() const{ return m_data[0]; }
  static SensorsManager & getSensorsManager();
  void update(double t);
};

 

 

 
#endif
