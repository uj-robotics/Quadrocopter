#ifndef EULERS_H
#define EULERS_H
// Helpers //
#define PI 3.14
#include <Wire.h>
#include <Servo.h>

/// Some algebra
double length(const double *vect);
void norm3(double * vect);
double to_angle(double rad);
double dotproduct(double* v1, double* v2);
double * crossproduct(double* v1 , double* v2);
double get_angle_between(double *v1, double *v2);

/// Convert to euler angles
double * to_euler(double x,double y,double z,double angle);


/*
Heading = rotation about y axis
Attitude = rotation about z axis
Bank = rotation about x axis
*/


/// Most important method - get euler angles
double *get_eulers(const double * vect1,const double * vect2);
double* get_eulers_to_g(const double *);
#endif
