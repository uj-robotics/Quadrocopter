#include "sensors.h"
#include "reference_frame.h"


ReferenceFrame::ReferenceFrame() :
	BUFFER_MS(400), GYRO_TRUST(0.9),THRUST_G(100.0) {}


void ReferenceFrame::setuBase(double val)
{
	this->uBase = val;
}
/*
*Initialize reference frame
*@param sampling [in s] time between sensor sampling
*@param ubase default drag
*/
void ReferenceFrame::init(double sampling, double ubase)
{
	this->setuBase(ubase);

	this->BUFFER_SIZE = (int)(BUFFER_MS / sampling);
	this->buffer_acc_X = new double[BUFFER_SIZE];
	this->buffer_acc_Y = new double[BUFFER_SIZE];
	this->buffer_acc_Z = new double[BUFFER_SIZE];
	this->angle_vel = new double[3];
	this->acc = new double[3];
	this->acc_ref = new double[3];
	this->north_ref = new double[3];
	this->north = new double[3];
	this->angle_ref = new double[3];
	this->angle = new double[3];
	this->error = new double[3];

	for(int i=0;i<3;++i) this->north[i] = this->north_ref[i] =  this->error[i] = this->angle[i] = this->angle_vel[i] = this->acc[i] = this->angle_ref[i]= this->acc_ref[i] = 0.0;
	this->acc_ref[2] = 1.0;
	for(int i=0;i<BUFFER_SIZE;++i) this->buffer_acc_X[i] = this->buffer_acc_Y[i] = this->buffer_acc_Z[i] = 0.0;

}

/*
* Low-pass filter
* @param Array specified buffer array
* @param raw_data new data to be added to measurements
*/
double ReferenceFrame::calcAcc(double* Array, double raw_data)
{
	Array[this->buff_index] = raw_data;

	double sum = 0;
	for(int i = 0; i < this->BUFFER_SIZE; ++i)
		sum += Array[i];

	return (double)sum / BUFFER_SIZE;
}

/*
* Update reference frame
* @param t actual time [in s]
* @param dt time from the last update [in ms]
*/
void ReferenceFrame::update(double t, double dt)
{
	SensorsManager& sm = SensorsManager::getSensorsManager();
	sm.update(t);

	//TODO: calculates compass data
	const double* newNorthData = sm.getNorth();
	this->north[0] = newNorthData[0];
	this->north[1] = newNorthData[1];
	this->north[2] = newNorthData[2];

	//calculates acceleration (low-pass filter)
	const double* new_acc = sm.getAcc();
	this->acc[0] = calcAcc(this->buffer_acc_X, new_acc[0]);
	this->acc[1] = calcAcc(this->buffer_acc_Y, new_acc[1]);
	this->acc[2] = calcAcc(this->buffer_acc_Z, new_acc[2]);

	buff_index = (++buff_index) % BUFFER_SIZE;

	//calculates angular rate
	const double* new_ang = sm.getAngleVel();
	this->angle_vel[0] = new_ang[0];
	this->angle_vel[1] = new_ang[1];
	this->angle_vel[2] = new_ang[2];

	double acc_accounted[] = { this->acc[0], this->acc[1], this->acc[2] + min(0.0, this->uBase/THRUST_G)  };

	//assigns Euler's angles
	double* eulers  = get_eulers(this->acc_ref, acc_accounted);

	//TODO: assigns compass
	eulers[1] = 0.0;

	//calculates final angle with complementary filter
	double coef = 0.49 / (0.49 + dt);
	this->angle[0] = coef * (this->angle[0] + this->angle_vel[0] * dt) + (1-coef) * eulers[0];
	this->angle[1] = coef * (this->angle[1] + this->angle_vel[1] * dt) + (1-coef) * eulers[2];
	this->angle[2] = coef * (this->angle[2] + this->angle_vel[2] * dt) + (1-coef) * eulers[1];

	//calculates current errors
	this->error[0] = this->angle[0] - this->angle_ref[0];
	this->error[1] = this->angle[1] - this->angle_ref[1];
	this->error[2] = this->angle[2] - this->angle_ref[2];

	delete[] eulers;
}

