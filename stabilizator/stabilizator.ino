#include <Wire.h>
#include <Servo.h>

#include "reference_frame.h"
#include "sensors.h"
#include "engines_manager.h"
#include "eulers.h"
#include "timer.h"

//EnginesManager Engines;

double time_elapsed;
int time_bruno;

unsigned int LED_RED=4;
unsigned int LED_GREEN=2;
unsigned int LED_YELLOW=3;

// PID parameters
double KP_x ,KP_y;
double KD_x, KD_y;
double KI_x, KI_y;

double u_x, u_y, u_z, u_h; // wartosci kontrolowane u_x,u_y to roznica w predkosci zgodny_z_osi - przeciwny_do_osi, u_z nie ma, u_h to wielkosc bazowa
double u_base = 30.0; //podstawowa prędkość silników

//pamiec
double e_x_prev=0.0, e_y_prev=0.0;
double E_x_sum =0.0, E_y_sum =0.0;
double de_x, de_y;

void setTuningsY(double KP, double KI, double KD)
{
	double SampleTimeInSec = ((double)Timer::getTimer().PID_MS)/1000;
	KP_y = KP;
	KI_y =  KI* SampleTimeInSec;
	KD_y = KD / SampleTimeInSec;
}

void setTuningsX(double KP, double KI, double KD)
{
	double SampleTimeInSec = ((double)Timer::getTimer().PID_MS)/1000;
	KP_x = KP;
	KI_x = KI * SampleTimeInSec;
	KD_x = KD / SampleTimeInSec;
}

//reference acc
double acc_ref[] = {0.0,0.0,1.0};
double compass_ref[] = {-50,-317,-172.0};
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

	EnginesManager& engines = EnginesManager::getEnginesManager();

	engines.BL.setSpeed(u_base);
	engines.BL.start();
	engines.BR.setSpeed(u_base);
	engines.BR.start();
	engines.FR.setSpeed(u_base);
	engines.FR.start();
	engines.FL.setSpeed(u_base);
	engines.FL.start();

	// == Inicjalizacja zakonczona pomyslnie == //
	count = 0;

	// == Inicjalizacja timera == //
	Timer::getTimer().startTimer(TC2, 0, TC6_IRQn, Timer::getTimer().SAMPLING_MS);
	Timer::getTimer().startTimer(TC2, 1, TC7_IRQn, Timer::getTimer().PID_MS);


	// == Inicjalizacja stalych PID == //

	//set_tunings_x(0.3, 0.14, 0.006);
	//set_tunings_y(0.3, 0.14, 0.006);
	setTuningsX(0.22, 0.14, 0.023);
	//set_tunings_y(0.22, 0.14, 0.009);
	setTuningsY(0.24,0.19,0.025);

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

	//Wyswietlam pomiary kata
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
	EnginesManager& engines = EnginesManager::getEnginesManager(); 

	//zatrzymuje silniki po 20 sekundach
	if(time_bruno >= 20000 / Timer::getTimer().SAMPLING_MS || time_bruno < 0)
	{
		engines.BL.stop(); engines.BR.stop(); engines.FR.stop(); engines.FL.stop();
		return;
	}

	ReferenceFrame& rf = ReferenceFrame::getReferenceFrame();
	const double* error = rf.getError();

	E_x_sum += error[0];
	E_y_sum += error[1];

	// === Policz pochodne bledu === ///
	de_x = rf.getAngleVel()[0];  //error[0] - e_x_prev;
	de_y = rf.getAngleVel()[1];  //error[1] - e_y_prev;



	// === Policz sygnaly kontrolne ==== //
	u_x = error[0]*KP_x + de_x*KD_x + E_x_sum*KI_x;
	u_y = error[1]*KP_y + de_y*KD_y + E_y_sum*KI_y;
	u_h = u_base;


	// === Stabilizacja pitch i roll === //
	engines.BL.setSpeed((int)(u_h + u_x/2.0 + u_y/2.0 ));
	engines.FR.setSpeed((int)(u_h - u_x/2.0 - u_y/2.0));
	engines.BR.setSpeed((int)(u_h - u_x/2.0 + u_y/2.0));
	engines.FL.setSpeed((int)(u_h + u_x/2.0 - u_y/2.0));
}

void loop() {}


