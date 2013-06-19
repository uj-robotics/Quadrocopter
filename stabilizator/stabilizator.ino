#include <Wire.h>
#include <Servo.h>

#include "reference_frame.h"
#include "sensors.h"
#include "EnginesManager.h"
#include "eulers.h"
#include "Timer.h"

//EnginesManager Engines;

double time_elapsed;
int time_bruno;

unsigned int LED_RED=4;
unsigned int LED_GREEN=2;
unsigned int LED_YELLOW=3;

// PID stale //
double KP_x ,KP_y;
double KD_x, KD_y;
double KI_x, KI_y;

double u_x, u_y, u_z, u_h; // wartosci kontrolowane u_x,u_y to roznica w predkosci zgodny_z_osi - przeciwny_do_osi, u_z nie ma, u_h to wielkosc bazowa
double u_base = 30.0; //ile zawsze powinno byc

//pamiec
double e_x_prev=0.0, e_y_prev=0.0;
double E_x_sum =0.0, E_y_sum =0.0;
double de_x, de_y;

//north reference

void set_tunings_y(double KP, double KI, double KD)
{
  double SampleTimeInSec = ((double)Timer::getTimer().PID_MS)/1000;
  KP_y = KP;
  KI_y =  KI* SampleTimeInSec;
  KD_y = KD / SampleTimeInSec;
}
void set_tunings_x(double KP, double KI, double KD)
{
  double SampleTimeInSec = ((double)Timer::getTimer().PID_MS)/1000;
  KP_x = KP;
  KI_x = KI * SampleTimeInSec;
  KD_x = KD / SampleTimeInSec;
}

// Updates (in interruption) readings for sensors //
void update_readings(){
  time_elapsed+=5000*(1E-3);
  ReferenceFrame::getReferenceFrame().update(time_elapsed, Timer::getTimer().SAMPLING_MS / 1000.0);
}
//reference acceleration
double acceleration_ref[]={
  0.0,0.0,1.0};
double compass_ref[] = {
  -50,-317,-172.0};
int count;

double* accAvg;
double* gyroAvg;

void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);

  time_elapsed = 0.0;
  time_bruno = 0;

  Serial.begin(9600);

  // == Init I2C Interface == //
  i2cInterface::init();

  // == Init SensorManager == //
  SensorsManager::getSensorsManager().init();
  SensorsManager::getSensorsManager().OnePointCallibration();

  // == Init ReferenceFrame == //
  ReferenceFrame::getReferenceFrame().init(Timer::getTimer().SAMPLING_MS, u_base);

  EnginesManager& Engines = EnginesManager::getEnginesManager();
  
  Engines.BL.SetSpeed(u_base);
  Engines.BL.Start();
  Engines.BR.SetSpeed(u_base);
  Engines.BR.Start();
  Engines.FR.SetSpeed(u_base);
  Engines.FR.Start();
  Engines.FL.SetSpeed(u_base);
  Engines.FL.Start();

  // == Inicjalizacja zakonczona pomyslnie == //
  count =0;
  //delay(100);

  // == Inicjalizacja timera == //
  Timer::getTimer().startTimer(TC2, 0, TC6_IRQn, Timer::getTimer().SAMPLING_MS);
  Timer::getTimer().startTimer(TC2, 1, TC7_IRQn, Timer::getTimer().PID_MS);


  // == Inicjalizacja stalych PID == //
  
  //set_tunings_x(0.3, 0.14, 0.006);
  //set_tunings_y(0.3, 0.14, 0.006);
  set_tunings_x(0.22, 0.14, 0.009);
  //set_tunings_y(0.22, 0.14, 0.009);
  set_tunings_y(0.21,0.18,0.011);

  accAvg = new double[3];
  gyroAvg = new double[3];
  for(int i = 0; i < 3; ++i)
    accAvg[i] = gyroAvg[i] = 0;
}

const double* ang;

void TC6_Handler()
{
  TC_GetStatus(TC2, 0);
  time_elapsed += 500;
  time_bruno += 1;

  ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
  rf.update(time_elapsed, Timer::getTimer().SAMPLING_MS / 1000.0);

  //SensorsManager& sm = SensorsManager::getSensorsManager();

  ang = rf.getAngle();

  if(time_bruno % 5 == 0)
  {
    Serial.print(ang[0]);
    Serial.print(";");
    Serial.print(ang[1]);
    Serial.print(";");
    Serial.print(ang[2]);
    Serial.println(";");
  }
}

void TC7_Handler()
{
   TC_GetStatus(TC2, 1);
   EnginesManager& Engines = EnginesManager::getEnginesManager(); 
   
   if(time_bruno >= 10000 / Timer::getTimer().SAMPLING_MS || time_bruno < 0)
   {
     EnginesManager& Engines = EnginesManager::getEnginesManager();
     Engines.BL.Stop(); Engines.BR.Stop(); Engines.FR.Stop(); Engines.FL.Stop();
     return;
   }
   
   ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
   const double * error = rf.getError();

   E_x_sum += error[0];
   E_y_sum += error[1];

   // === Policz pochodne bledu === ///
   de_x = rf.getAngleAcceleration()[0];//error[0] - e_x_prev;
   de_y = rf.getAngleAcceleration()[1];//error[1] - e_y_prev;



   // === Policz sygnaly kontrolne ==== //
   u_x = error[0]*KP_x + de_x*KD_x + E_x_sum*KI_x;
   //u_x = 0;
   u_y = error[1]*KP_y + de_y*KD_y + E_y_sum*KI_y;
   //u_y = 0;
   u_h = u_base ;//+ sm.getAccelerationLength()*(sm.getAcceleration()[2]>0.0 ? 1.0 :  -1.0)*KP_h;


   // === Stabilizacja pitch i roll === //
   Engines.BL.SetSpeed((int)(u_h + u_x/2.0 + u_y/2.0 ));
   Engines.FR.SetSpeed((int)(u_h - u_x/2.0 - u_y/2.0));
   Engines.BR.SetSpeed((int)(u_h - u_x/2.0 + u_y/2.0));
   Engines.FL.SetSpeed((int)(u_h + u_x/2.0 - u_y/2.0));
   
  /*accAvg[0] += acc[0];
   accAvg[1] += acc[1];
   accAvg[2] += acc[2];
   gyroAvg[0] += gyro[0];
   gyroAvg[1] += gyro[1];
   gyroAvg[2] += gyro[2];
   
   if(time_bruno % 500 == 0)
   {
   Serial.print("Acc: ");
   Serial.print(accAvg[0] / 500);
   Serial.print(" ");
   Serial.print(accAvg[1] / 500);
   Serial.print(" ");
   Serial.print(accAvg[2] / 500);
   Serial.print(",   Gyro: ");
   Serial.print(gyroAvg[0] / 500);
   Serial.print(" ");
   Serial.print(gyroAvg[1] / 500);
   Serial.print(" ");
   Serial.println(gyroAvg[2] / 500);
   accAvg[0] = accAvg[1] = accAvg[2] = 0;
   gyroAvg[0] = gyroAvg[1] = gyroAvg[2] = 0;
   }*/ 
}

void loop() {
}

