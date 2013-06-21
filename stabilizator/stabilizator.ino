#include <Wire.h>
#include <Servo.h>

#include "reference_frame.h"
#include "sensors.h"
#include "engines_manager.h"
#include "eulers.h"
#include "timer.h"


double time_elapsed;
int time_bruno;

unsigned int LED_RED=4;
unsigned int LED_GREEN=2;
unsigned int LED_YELLOW=3;

// PID constants
double KP_x ,KP_y;
double KD_x, KD_y;
double KI_x, KI_y;

double u_x, u_y, u_z, u_h; // Control signals on each axis, where magnitude is difference from u_base
double u_base = 30.0; //Control signal base (no height measurements so it can hover)

// PID variables
double e_x_prev=0.0, e_y_prev=0.0;
double E_x_sum =0.0, E_y_sum =0.0;
double de_x, de_y;

// Set PID constants taking into account time step
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

void setup()
{

  //Setups LEDS for diagnosing 
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
  
  // == Init Engines == //
  Engines.BL.setSpeed(u_base);
  Engines.BL.start();
  Engines.BR.setSpeed(u_base);
  Engines.BR.start();
  Engines.FR.setSpeed(u_base);
  Engines.FR.start();
  Engines.FL.setSpeed(u_base);
  Engines.FL.start();

  // == Init successfull == //
  count =0;
  
  // == Init timers == //
  Timer::getTimer().startTimer(TC2, 0, TC6_IRQn, Timer::getTimer().SAMPLING_MS);
  Timer::getTimer().startTimer(TC2, 1, TC7_IRQn, Timer::getTimer().PID_MS);


  // ==  Set PID constants == //
  set_tunings_x(0.22, 0.14, 0.023);
  set_tunings_y(0.24,0.19,0.025);

  accAvg = new double[3];
  gyroAvg = new double[3];
  for(int i = 0; i < 3; ++i)
    accAvg[i] = gyroAvg[i] = 0;
}

const double* ang;

//One of two main timers : fetching data from sensors
void TC6_Handler()
{
  TC_GetStatus(TC2, 0);
  time_elapsed += 500;
  time_bruno += 1;

  ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
  rf.update(time_elapsed, Timer::getTimer().SAMPLING_MS / 1000.0);

  ang = rf.getAngle();
  
  /*Write angles for visualisation*/
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

//Second timer : PID regulator (separated because PID is slower, because engines take a lot of CPU time)
void TC7_Handler()
{
   TC_GetStatus(TC2, 1);
   EnginesManager& Engines = EnginesManager::getEnginesManager(); 
   
   //For security reasons
   if(time_bruno >= 20000 / Timer::getTimer().SAMPLING_MS || time_bruno < 0)
   {
     Engines.BL.stop(); Engines.BR.stop(); Engines.FR.stop(); Engines.FL.stop();
     return;
   }
   
   ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
   const double * error = rf.getError();

   E_x_sum += error[0];
   E_y_sum += error[1];

   // === Calculate error derivatives (angle velocity)  === ///
   de_x = rf.getAngleVel()[0];
   de_y = rf.getAngleVel()[1];



   // === Calculate control signals ==== //
   u_x = error[0]*KP_x + de_x*KD_x + E_x_sum*KI_x;
   u_y = error[1]*KP_y + de_y*KD_y + E_y_sum*KI_y;
   u_h = u_base ; //TODO : add height PID


   // === PID signals === //
   Engines.BL.setSpeed((int)(u_h + u_x/2.0 + u_y/2.0 ));
   Engines.FR.setSpeed((int)(u_h - u_x/2.0 - u_y/2.0));
   Engines.BR.setSpeed((int)(u_h - u_x/2.0 + u_y/2.0));
   Engines.FL.setSpeed((int)(u_h + u_x/2.0 - u_y/2.0));
}

void loop() {
}

