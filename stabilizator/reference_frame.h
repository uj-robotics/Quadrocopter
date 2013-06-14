
#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include "sensors.h"
/*
* Klasa ktora ocenia pozycje quadrocoptera na podstawie odczytow z silnika: akcelerometr, zyroskop, barometr
*/
class ReferenceFrame{
private:
	// We measure reference frame position as g vector 
	int BUFFER_SIZE;
        const double BUFFER_MS;
        double* BufferAccelerationX;
	double* BufferAccelerationY;
	double* BufferAccelerationZ;
	double* AngleAcceleration;
	double* Acceleration;
	int BufferIndex;
	double* AccelerationRef;
	ReferenceFrame();
public:

	static ReferenceFrame& getReferenceFrame(){
		static ReferenceFrame rf;
		return rf;
	}
	/// Initiliaze reference frame
	void init(double);
	/// Zwraca blad jako 4 wymiarowa tablice
	double* getError();
	/// Uaktualnia dane z sensorow
	void update(double, double);

	const double* getAcceleration() { return this->Acceleration; }
	const double* getAngleAcceleration() { return this->AngleAcceleration; }
	double calcAngleAcceleration(double*, double);
	double calcAcceleration(double*, double);
};


#endif

