#include "sensors.h"
#include "reference_frame.h"
// 
// Klasa ktora ocenia pozycje quadrocoptera na podstawie odczytow z silnika: akcelerometr, zyroskop, barometr
//

ReferenceFrame::ReferenceFrame() : BUFFER_MS(400)
{

}

void ReferenceFrame::init(double sampling)
{
        this->BUFFER_SIZE = (int)(BUFFER_MS / sampling); 
	this->BufferAccelerationX = new double[BUFFER_SIZE];
	this->BufferAccelerationY = new double[BUFFER_SIZE];
	this->BufferAccelerationZ = new double[BUFFER_SIZE];
	this->AngleAcceleration = new double[3];
	this->Acceleration = new double[3];
        for(int i=0;i<3;++i) this->AngleAcceleration[i] = this->Acceleration[i] = 0.0;
        for(int i=0;i<BUFFER_SIZE;++i) this->BufferAccelerationX[i] = this->BufferAccelerationY[i] = this->BufferAccelerationZ[i] = 0.0;

}

double* ReferenceFrame::getError()
{
     double * error = new double[3];
         
     error[0] = this->Acceleration[0] - this->AccelerationRef[0];
     error[1] = this->Acceleration[1] - this->AccelerationRef[1];
     error[2] = this->Acceleration[2] - this->AccelerationRef[2];
     
     return error;
}

double ReferenceFrame::calcAngleAcceleration(double* Array, double RawData)
{
	return RawData;
}

double ReferenceFrame::calcAcceleration(double* Array, double RawData)
{
	Array[this->BufferIndex] = RawData;

	double sum = 0;
	for(int i = 0; i < this->BUFFER_SIZE; ++i)
		sum += Array[i];

	return (double)sum / BUFFER_SIZE;
}

void ReferenceFrame::update(double t, double dt)
{
	SensorsManager& sm = SensorsManager::getSensorsManager();
	sm.update(t);
	const double* newAccData = sm.getAcceleration();
	this->Acceleration[0] = calcAcceleration(this->BufferAccelerationX, newAccData[0]);
	this->Acceleration[1] = calcAcceleration(this->BufferAccelerationY, newAccData[1]);
	this->Acceleration[2] = calcAcceleration(this->BufferAccelerationZ, newAccData[2]);

	const double* newAngData = sm.getAngleAcceleration();
	this->AngleAcceleration[0] = calcAngleAcceleration(NULL, newAngData[0]);
	this->AngleAcceleration[1] = calcAngleAcceleration(NULL, newAngData[1]);
	this->AngleAcceleration[2] = calcAngleAcceleration(NULL, newAngData[2]);

	BufferIndex = (++BufferIndex) % BUFFER_SIZE;
}
