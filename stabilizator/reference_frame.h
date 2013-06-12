
#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include "sensors.h"
/*
* Klasa ktora ocenia pozycje quadrocoptera na podstawie odczytow z silnika: akcelerometr, zyroskop, barometr
*/
class ReferenceFrame{
public:
  // We measure reference frame position as g vector 
  double x_ref, y_ref, z_ref, h_ref; //4d u position vector
  double x_measure, y_measure, z_measure, h_measure; 
  ReferenceFrame(){}
public:
       
  static ReferenceFrame& getReferenceFrame(){
    static ReferenceFrame rf;
    return rf;
  }
  /// Initiliaze reference frame
  void init();
  /// Zwraca blad jako 4 wymiarowa tablice
  double* getError();
  /// Uaktualnia dane z sensorow
  void update(double);
      
};


#endif

