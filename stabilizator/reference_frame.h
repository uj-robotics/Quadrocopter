
#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include "sensors.h"
#include "eulers.h"

/*
* Class responsible for estimating position of the quadrocopter (basing on filters and sensors)
*/
class ReferenceFrame
{
private:
	int BUFFER_SIZE;
	const double GYRO_TRUST; //constant for accounting for drag
	const double BUFFER_MS; //low pass filter : window size for accumulating readings
	const double THRUST_G ;	//ile na silnikach daje g
	double uBase;

	double* buffer_acc_X;	//bufor do low-pass filter na osi X
	double* buffer_acc_Y;	//bufor do low-pass filter na osi Y
	double* buffer_acc_Z;	//bufor do low-pass filter na osi X
	int buff_index;			//indeks do buforów

	double* acc;			//przyspieszenie z akcelerometru
	double* angle_vel;		//prêdkoœæ k¹towa z ¿yroskopu
	double* north;			//TODO: pó³noc z kompasu

	double* angle;			//k¹t w³aœciwy
	double * error;			//b³¹d wzglêdem pozycji ustabilizowanej

	double* acc_ref;
	double* angle_ref;      //our quadrocopter will be positioning it self to this angle
	double* north_ref;

	ReferenceFrame();
	double calcAcc(double*, double);

public:

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

