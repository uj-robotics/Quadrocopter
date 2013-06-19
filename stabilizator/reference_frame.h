
#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include "sensors.h"
#include "eulers.h"

class ReferenceFrame{
private:
	int BUFFER_SIZE; const double GYRO_TRUST; const double BUFFER_MS; const double THRUST_G ; //THRUST_G - ile na silnikach daje g

	double uBase;
        double* buffer_acc_X;
	double* buffer_acc_Y;
	double* buffer_acc_Z;
	double* angle_vel;
	double* angle,*north,*acc;
	double * error;
    int BufferIndex;
	double* acc_ref, *angle_ref, *north_ref;
	ReferenceFrame();
    double calcAngleVel(double*, double);
	double calcAcc(double*, double);
public:

	static ReferenceFrame& getReferenceFrame(){
		static ReferenceFrame rf;
		return rf;
	}

	void init(double, double);
	void update(double, double);

	void setuBase(double val){
		this->uBase = val;
	}

    const double* getError(){ return this->error; }
    const double * getAngle() { return this->angle; }
    const double* getAcc() { return this->acc; }
    const double* getAngleVel() { return this->angle_vel; }

};


#endif

