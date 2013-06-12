#include "sensors.h"
#include "reference_frame.h"
// 
// Klasa ktora ocenia pozycje quadrocoptera na podstawie odczytow z silnika: akcelerometr, zyroskop, barometr
//

void ReferenceFrame::init(){
   this->x_ref = this->y_ref = this->z_ref = this->h_ref = 0.0;
   this->z_ref = 1.0; 
}    


void ReferenceFrame::update(double t){
     SensorsManager & sm=SensorsManager::getSensorsManager();    
     /// === Update readings based on time === ///
     sm.update(t);
     
     this->x_measure = sm.getAcceleration()[0];
     this->y_measure = sm.getAcceleration()[1];
     this->z_measure = sm.getAcceleration()[2];      
     
     //Interesuje nas wersor wiec
     double len = sqrt(this->x_measure*this->x_measure +  this->y_measure*this->y_measure + this->z_measure*this->z_measure);
     this->x_measure/=len;
     this->y_measure/=len;
     this->z_measure/=len;
     
     // TODO: implement barometer
     this->h_measure = this->h_ref; 
}

double* ReferenceFrame::getError(){
     double * error = new double[4];
         
     error[0] = this->x_measure - this->x_ref;
     error[1] = this->y_measure - this->y_ref;
     error[2] = this->z_measure - this->z_ref;
     error[3] = this->h_measure - this->h_ref;
     
     return error;
}
