#include "sensors.h"
#include "reference_frame.h"
//
// Klasa ktora ocenia pozycje quadrocoptera na podstawie odczytow z silnika: akcelerometr, zyroskop, barometr
//

ReferenceFrame::ReferenceFrame() : 
BUFFER_MS(400), GYRO_TRUST(0.9),THRUST_G(100.0)
{

}

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



//UWAGA: W pdf jest ze stala a ma byc taka sama dla low i high pass filter (to moze byc wazne teoretycznie)
//Wiec dodaj TAU jako stala , a w init() licz z TAU i DT sobie A


//poprawic dodac high-pass-filter
double ReferenceFrame::calcAngleVel(double* Array, double RawData)
{
  return RawData;
}

//poprawic wg .pdf  low-pass filter
double ReferenceFrame::calcAcc(double* Array, double RawData)
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

  // Przypisz kompas
  const double* newNorthData = sm.getNorth();
  this->north[0] = newNorthData[0];
  this->north[1] = newNorthData[1];
  this->north[2] = newNorthData[2];

  // Policz akceleracje (low-pass filter)
  const double* new_acc = sm.getAcc();
  this->acc[0] = calcAcc(this->buffer_acc_X, new_acc[0]);
  this->acc[1] = calcAcc(this->buffer_acc_Y, new_acc[1]);
  this->acc[2] = calcAcc(this->buffer_acc_Z, new_acc[2]);


  // Policz predkosc katowa (high-pass filter)
  const double* new_ang = sm.getAngleVel();
  this->angle_vel[0] = calcAngleVel(NULL, new_ang[0]);
  this->angle_vel[1] = calcAngleVel(NULL, new_ang[1]);
  this->angle_vel[2] = calcAngleVel(NULL, new_ang[2]);

  BufferIndex = (++BufferIndex) % BUFFER_SIZE;

  // Policz nowy kat
  double ngx,ngy,ngz;
  ngx = this->angle[0] + this->angle_vel[0]*dt;
  ngy = this->angle[1] + this->angle_vel[1]*dt;
  ngz = this->angle[2] + this->angle_vel[2]*dt;


  double accAccounted[] = {
    this->acc[0], this->acc[1], this->acc[2] + min(0.0, this->uBase/THRUST_G)  };

  double * eulers  = get_eulers(this->acc_ref, accAccounted); //dodac kompas
  
  eulers[1] = 0.0; //brak kompasu

  double coef = 0.49 / (0.49 + dt);

  this->angle[0] = coef * (this->angle[0] + this->angle_vel[0] * dt) + (1-coef) * eulers[0];
  this->angle[1] = coef * (this->angle[1] + this->angle_vel[1] * dt) + (1-coef) * eulers[2];
  this->angle[2] = coef * (this->angle[2] + this->angle_vel[2] * dt) + (1-coef) * eulers[1];

  // Policz nowe bledy
  this->error[0] = this->angle[0] - this->angle_ref[0];
  this->error[1] = this->angle[1] - this->angle_ref[1];
  this->error[2] = this->angle[2] - this->angle_ref[2];

  delete[] eulers;
}

