#include <Servo.h>
#include <EnginesManager.h>
EnginesManager Engines;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Engines.FL.SetSpeed(45);
  Engines.FR.SetSpeed(45);
  Engines.BL.SetSpeed(45);
  Engines.BR.SetSpeed(45);
  
  Engines.FL.Start();
  Engines.FR.Start();
  Engines.BL.Start();
  Engines.BR.Start();
  
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly: 
  Engines.FL.Stop();
  Engines.FR.Stop();
  Engines.BL.Stop();
  Engines.BR.Stop();
}
