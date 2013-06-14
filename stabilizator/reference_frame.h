
#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include "sensors.h"
#include "eulers.h"

class ReferenceFrame{
private:
	int BUFFER_SIZE; const double GYRO_TRUST; const double BUFFER_MS;

  
        double* BufferAccelerationX;
	double* BufferAccelerationY;
	double* BufferAccelerationZ;
	double* AngleAcceleration;
	double* Angle,*North,*Acceleration;
	double * Error;
    int BufferIndex;
	double* AccelerationRef, *AngleRef, *NorthRef;
	ReferenceFrame();
    double calcAngleAcceleration(double*, double);
	double calcAcceleration(double*, double);
public:

	static ReferenceFrame& getReferenceFrame(){
		static ReferenceFrame rf;
		return rf;
	}

	void init(double);
	void update(double, double);


    const double* getError(){ return this->Error; }
    const double * getAngle() { return this->Angle; }
	const double* getAcceleration() { return this->Acceleration; }
	const double* getAngleAcceleration() { return this->AngleAcceleration; }

};


#endif

