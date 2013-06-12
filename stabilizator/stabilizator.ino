//TODO: ogarnac roznice ATMega32 vs ARM
//TODO: zaklepac przerwania na ARM
//TODO: nauczyc sie I2C, zeby opowiedziec


#include <Wire.h>
#include <Servo.h>


#include "reference_frame.h"
#include "sensors.h"
#include "EnginesManager.h"

EnginesManager Engines;

double time_elapsed;
unsigned int LED_RED=4;
unsigned int LED_GREEN=2;
unsigned int LED_YELLOW=3;

// Updates (in interruption) readings for sensors //
void update_readings(){
    time_elapsed+=500*(1E-6);
    
    ReferenceFrame::getReferenceFrame().update(time_elapsed);
}

int count;
void setup() 
{ 
   pinMode(LED_RED, OUTPUT);
   pinMode(LED_YELLOW, OUTPUT);
   pinMode(LED_GREEN, OUTPUT);
   
   digitalWrite(LED_RED, LOW);
   digitalWrite(LED_YELLOW, LOW);
   digitalWrite(LED_GREEN, LOW);
   
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
  // Timer.getAvailable().attachInterrupt(update_readings).start(500000); // Every 500ms update sensor readings

   // == Init motors == //
   
   Engines.BL.SetSpeed(10);
   Engines.BL.Start(); 
   Engines.BR.SetSpeed(10);
   Engines.BR.Start(); 
   Engines.FR.SetSpeed(10);
   Engines.FR.Start(); 
   Engines.FL.SetSpeed(10);
   Engines.FL.Start(); 

   // == Inicjalizacja zakonczona pomyslnie == //
   count =0;
   delay(1000);
}


// brak algebry

//kontroler: PID (zaklada ze motorki kreca sie tak zeby anulowac moment pedu)

//P :
double KP_x = 30.0,KP_y=30.0,KP_z=0.0, KP_h = 10.0;
double KD_x = 10.0, KD_y = 10.0, KD_z = 0.0;
//!!!!!!!!!uwaga te stale sa tez zalezne od samplowania!!!!!!!!
double KI_x = 0.005, KI_y = 0.005, KI_z = 0.1;
double u_x, u_y, u_z, u_h; // wartosci kontrolowane u_x,u_y to roznica w predkosci zgodny_z_osi - przeciwny_do_osi, u_z nie ma, u_h to wielkosc bazowa
double e_x_prev=0.0, e_y_prev=0.0, e_z_prev = 0.0;
double E_x_sum =0.0, E_y_sum =0.0, E_z_sum=0.0;
double u_base = 9.0; //ile zawsze powinno byc
double de_x, de_y, de_z;

/*
* Klasa odpowiedzialna za podazanie za wektorem
*/
class VectorPIDTracket{
public:

};


// mamy kompas!

void loop() 
{
  delay(5);
  count +=1;  
  if(count>800){
     digitalWrite(LED_YELLOW, count%2==0 ? HIGH: LOW);
     Engines.BL.Stop(); 
     Engines.BR.Stop(); 
     Engines.FR.Stop(); 
     Engines.FL.Stop();     
     digitalWrite(LED_RED,HIGH);
     return; 
  
  }
  
  
  // === Updatuj sensory ==== //
  SensorsManager & sm=SensorsManager::getSensorsManager();  
  update_readings();
  double * error = ReferenceFrame::getReferenceFrame().getError();

  
  Serial.print(error[0]);
  Serial.print(";");
  Serial.print(error[1]);
  Serial.println();

   // === Calkuj blad === //
   E_x_sum += error[0];
   E_y_sum += error[1];
   E_z_sum += error[2];
  
   // === Policz pochodne bledu === ///
   de_x = error[0] - e_x_prev;
   de_y = error[1] - e_y_prev;
   de_z = error[2] - e_z_prev;
   
   // === Policz sygnaly kontrolne ==== //
   u_x = error[0]*KP_x + de_x*KD_x + E_x_sum*KI_x;
   u_y = error[1]*KP_y + de_y*KD_y + E_y_sum*KI_y;
   u_h = error[3]*KP_h;   
   u_h = u_base ;//+ sm.getAccelerationLength()*(sm.getAcceleration()[2]>0.0 ? 1.0 :  -1.0)*KP_h;
 
   // === Zapamietaj stare bledy (do pochodnej) === //  
   e_x_prev = error[0];
   e_y_prev = error[1];
   e_z_prev = error[2];

 
   // === Stabilizacja pitch i roll === //
   Engines.BL.SetSpeed((int)(u_h - u_y/2.0 +u_x/2.0 ));
   Engines.FR.SetSpeed((int)(u_h + u_y/2.0) - u_x/2.0);
   Engines.BR.SetSpeed((int)(u_h + u_y/2.0) + u_x/2.0);
   Engines.FL.SetSpeed((int)(u_h - u_y/2.0 - u_x/2.0));
}


