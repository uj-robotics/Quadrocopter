#include <Wire.h>
#include <Servo.h>

#include "reference_frame.h"
#include "sensors.h"
#include "engines_manager.h"
#include "eulers.h"
#include "timer.h"

//operations counters
double time_elapsed;
int countdown;

unsigned int LED_RED=4;
unsigned int LED_GREEN=2;
unsigned int LED_YELLOW=3;

//PID constants
double KP_x ,KP_y;
double KD_x, KD_y;
double KI_x, KI_y;

double u_x, u_y, u_z, u_h; //control signals on each axis, where magnitude is difference from u_base
double u_base = 30.0; //control signal base (no height measurements so it can hover)

//PID variables
double e_x_prev = 0.0, e_y_prev = 0.0;
double E_x_sum = 0.0, E_y_sum = 0.0;
double de_x, de_y;

//sets PID constants taking into account time step
void setTuningsY(double KP, double KI, double KD)
{
	double SampleTimeInSec = ((double)Timer::getTimer().PID_MS)/1000;
	KP_y = KP;
	KI_y = KI * SampleTimeInSec;
	KD_y = KD / SampleTimeInSec;
}

void setTuningsX(double KP, double KI, double KD)
{
	double SampleTimeInSec = ((double)Timer::getTimer().PID_MS)/1000;
	KP_x = KP;
	KI_x = KI * SampleTimeInSec;
	KD_x = KD / SampleTimeInSec;
}

//updates (in interruption) readings for sensors //
void updateReadings()
{
	time_elapsed += 5000*(1E-3);
	ReferenceFrame::getReferenceFrame().update(time_elapsed, Timer::getTimer().SAMPLING_MS / 1000.0);
}

void setup()
{
	//setups LEDS for diagnosing 
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_YELLOW, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);

	digitalWrite(LED_RED, LOW);
	digitalWrite(LED_YELLOW, LOW);
	digitalWrite(LED_GREEN, LOW);

	time_elapsed = 0.0;
	countdown = 0;

	Serial.begin(9600);

	//inits I2C Interface
	i2cInterface::init();

	//inits SensorManager
	SensorsManager::getSensorsManager().init();
	SensorsManager::getSensorsManager().OnePointCallibration();

	//inits ReferenceFrame
	ReferenceFrame::getReferenceFrame().init(Timer::getTimer().SAMPLING_MS, u_base);

	EnginesManager& engines = EnginesManager::getEnginesManager();

	//inits Engines
	engines.BL.setSpeed(u_base);
	engines.BL.start();
	engines.BR.setSpeed(u_base);
	engines.BR.start();
	engines.FR.setSpeed(u_base);
	engines.FR.start();
	engines.FL.setSpeed(u_base);
	engines.FL.start();

	//inits timers
	Timer::getTimer().startTimer(TC2, 0, TC6_IRQn, Timer::getTimer().SAMPLING_MS);
	Timer::getTimer().startTimer(TC2, 1, TC7_IRQn, Timer::getTimer().PID_MS);

	//sets PID constants
	setTuningsX(0.22, 0.14, 0.023);
	setTuningsY(0.24,0.19,0.025);
}

const double* ang;

//one of two main timers : fetching data from sensors
void TC6_Handler()
{
	TC_GetStatus(TC2, 0);
	time_elapsed += 500;
	countdown += 1;

	ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
	rf.update(time_elapsed, Timer::getTimer().SAMPLING_MS / 1000.0);

	ang = rf.getAngle();

	//writes angles for visualisation
	if(countdown % 5 == 0)
	{
		Serial.print(ang[0]);
		Serial.print(";");
		Serial.print(ang[1]);
		Serial.print(";");
		Serial.print(ang[2]);
		Serial.println(";");
	}
}

//second timer : PID regulator (separated because PID is slower, because engines take a lot of CPU time)
void TC7_Handler()
{
	TC_GetStatus(TC2, 1);
	EnginesManager& engines = EnginesManager::getEnginesManager(); 

	//for security reasons - stops timer after 20s
	if(countdown >= 20000 / Timer::getTimer().SAMPLING_MS || countdown < 0)
	{
		engines.BL.stop(); engines.BR.stop(); engines.FR.stop(); engines.FL.stop();
		return;
	}

	ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
	const double * error = rf.getError();

	E_x_sum += error[0];
	E_y_sum += error[1];

	//calculates error derivatives (angular rate)
	de_x = rf.getAngleVel()[0];
	de_y = rf.getAngleVel()[1];

	//calculates control signals
	u_x = error[0]*KP_x + de_x*KD_x + E_x_sum*KI_x;
	u_y = error[1]*KP_y + de_y*KD_y + E_y_sum*KI_y;
	u_h = u_base ; //TODO : add height PID

	//PID signals
	engines.BL.setSpeed((int)(u_h + u_x/2.0 + u_y/2.0 ));
	engines.FR.setSpeed((int)(u_h - u_x/2.0 - u_y/2.0));
	engines.BR.setSpeed((int)(u_h - u_x/2.0 + u_y/2.0));
	engines.FL.setSpeed((int)(u_h + u_x/2.0 - u_y/2.0));
}

void loop() {}
