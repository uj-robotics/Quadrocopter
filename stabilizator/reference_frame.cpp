#include "sensors.h"
#include "reference_frame.h"
//
// Klasa ktora ocenia pozycje quadrocoptera na podstawie odczytow z silnika: akcelerometr, zyroskop, barometr
//

ReferenceFrame::ReferenceFrame() : BUFFER_MS(400), GYRO_TRUST(0.9),THRUST_G(30.0)
{

}

void ReferenceFrame::init(double sampling, double ubase)
{
	this->setuBase(ubase);

    this->BUFFER_SIZE = (int)(BUFFER_MS / sampling);
	this->BufferAccelerationX = new double[BUFFER_SIZE];
	this->BufferAccelerationY = new double[BUFFER_SIZE];
	this->BufferAccelerationZ = new double[BUFFER_SIZE];
	this->AngleAcceleration = new double[3];
	this->Acceleration = new double[3];
    this->AccelerationRef = new double[3];
    this->NorthRef = new double[3];
    this->North = new double[3];
    this->AngleRef = new double[3];
    this->Angle = new double[3];
    this->Error = new double[3];

    for(int i=0;i<3;++i) this->North[i] = this->NorthRef[i] =  this->Error[i] = this->Angle[i] = this->AngleAcceleration[i] = this->Acceleration[i] = this->AngleRef[i]= this->AccelerationRef[i] = 0.0;
    this->AccelerationRef[2] = 1.0;
    for(int i=0;i<BUFFER_SIZE;++i) this->BufferAccelerationX[i] = this->BufferAccelerationY[i] = this->BufferAccelerationZ[i] = 0.0;

}



//UWAGA: W pdf jest ze stala a ma byc taka sama dla low i high pass filter (to moze byc wazne teoretycznie)
//Wiec dodaj TAU jako stala , a w init() licz z TAU i DT sobie A


//poprawic dodac high-pass-filter
double ReferenceFrame::calcAngleAcceleration(double* Array, double RawData)
{
	return RawData;
}

//poprawic wg .pdf  low-pass filter
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

        // Przypisz kompas
        const double* newNorthData = sm.getNorth();
        this->North[0] = newNorthData[0];
        this->North[1] = newNorthData[1];
        this->North[2] = newNorthData[2];
        
	// Policz akceleracje (low-pass filter)
	const double* newAccData = sm.getAcceleration();
	this->Acceleration[0] = calcAcceleration(this->BufferAccelerationX, newAccData[0]);
	this->Acceleration[1] = calcAcceleration(this->BufferAccelerationY, newAccData[1]);
	this->Acceleration[2] = calcAcceleration(this->BufferAccelerationZ, newAccData[2]);

	

    // Policz predkosc katowa (high-pass filter)
	const double* newAngData = sm.getAngleAcceleration();
	this->AngleAcceleration[0] = calcAngleAcceleration(NULL, newAngData[0]);
	this->AngleAcceleration[1] = calcAngleAcceleration(NULL, newAngData[1]);
	this->AngleAcceleration[2] = calcAngleAcceleration(NULL, newAngData[2]);

	BufferIndex = (++BufferIndex) % BUFFER_SIZE;

    // Policz nowy kat
    double ngx,ngy,ngz;
    ngx = this->Angle[0] + this->AngleAcceleration[0]*dt;
    ngy = this->Angle[1] + this->AngleAcceleration[1]*dt;
    ngz = this->Angle[2] + this->AngleAcceleration[2]*dt;

	
	double AccelerationAccounted[] = {this->Acceleration[0], this->Acceleration[1], this->Acceleration[2] + max(1.0, this->uBase/THRUST_G)};
	
    double * eulers  = get_eulers(this->AccelerationRef, AccelerationAccounted);
    double * eulers_to_g = get_eulers_to_g(AccelerationAccounted);
    // Uwaga zalozenie, ze AccelerationRef jest unormowany
    if(abs(this->AccelerationRef[2]-1.0)<0.001){
      eulers[0] = eulers_to_g[0]; eulers[2] = eulers_to_g[2];
    }

    // Uwaga: inna konwencja algebry (get_eulers)
    this->Angle[0] = GYRO_TRUST * ngx + (1-GYRO_TRUST)*eulers[0];
    this->Angle[1] = GYRO_TRUST * ngy + (1-GYRO_TRUST)*eulers[2];
    this->Angle[2] = ngz ;// Jak wszystko sie uda mozna tu dodac kompas (jest juz wszystko przygotowane)

    // Policz nowe bledy
    this->Error[0] = this->Angle[0] - this->AngleRef[0];
    this->Error[1] = this->Angle[1] - this->AngleRef[1];
    this->Error[2] = this->Angle[2] - this->AngleRef[2];

    delete[] eulers;
    delete[] eulers_to_g;

}
