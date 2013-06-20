
#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include "sensors.h"
#include "eulers.h"

class ReferenceFrame
{
private:
	int BUFFER_SIZE;
	const double GYRO_TRUST;
	const double BUFFER_MS;
	const double THRUST_G ;	//ile na silnikach daje g
	double uBase;

	double* buffer_acc_X;	//bufor do low-pass filter na osi X
	double* buffer_acc_Y;	//bufor do low-pass filter na osi Y
	double* buffer_acc_Z;	//bufor do low-pass filter na osi X
	int buff_index;			//indeks do buforow

	double* acc;			//przyspieszenie z akcelerometru
	double* angle_vel;		//predkosc katowa z zyroskopu
	double* north;			//TODO: polnoc z kompasu

	double* angle;			//kat wlasciwy
	double * error;			//blad wzgledem pozycji ustabilizowanej

	double* acc_ref;
	double* angle_ref;
	double* north_ref;

	ReferenceFrame();
	double calcAcc(double*, double);

public:
	//zwraca obiekt klasy singleton
	static ReferenceFrame& getReferenceFrame()
	{
		static ReferenceFrame rf;
		return rf;
	}

	void init(double, double);
	void update(double, double);
	void setuBase(double);
	const double* getError() { return this->error; }
	const double * getAngle() { return this->angle; }
	const double* getAcc() { return this->acc; }
	const double* getAngleVel() { return this->angle_vel; }
};


#endif

