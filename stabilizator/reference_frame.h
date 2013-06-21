
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
	const double THRUST_G ;	//g on engines
	double uBase;

	double* buffer_acc_X;	//low-pass filter buffer for x-axis
	double* buffer_acc_Y;	//low-pass filter buffer for y-axis
	double* buffer_acc_Z;	//low-pass filter buffer for z-axis
	int buff_index;			//index assigned to all buffers

	double* acc;			//acceleration from accelerometer
	double* angle_vel;		//angular rate from gyroscope
	double* north;			//TODO: north from compass

	double* angle;			//final angle
	double * error;

	double* acc_ref;
	double* angle_ref;      //our quadrocopter will be positioning itself to this angle
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

