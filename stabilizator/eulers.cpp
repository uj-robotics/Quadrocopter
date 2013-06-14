#include <math.h>

#include "eulers.h"

// Helpers //
#define PI 3.14
double length(const double *vect){
    return sqrt(vect[0]*vect[0] + vect[1]*vect[1]+vect[2]*vect[2]);
}
void norm3(double * vect){
    double len = length(vect);
    vect[0]/=len;
    vect[1]/=len;
    vect[2]/=len;
}
double to_angle(double rad){
    return (rad/PI)*180.0;
}



double dotproduct(double* v1, double* v2){
       return v1[0]*v2[0] + v1[1]*v2[1] + v2[2]*v1[2];
}
double * crossproduct(double* v1 , double* v2){
       double c1 = v1[1]*v2[2] - v1[2]*v2[1];
       double c2 = -v1[0]*v2[2] + v1[2]*v2[0];
       double c3 = v1[0]*v2[1] - v1[1]*v2[0];
       double * a = new double[3];
       a[0] = c1; a[1] = c2; a[2]= c3;
       return a;
}




double get_angle_between(double *v1, double *v2){
    return acos(dotproduct(v1,v2));
}

/*
* Convert to euler angles
*/
double * to_euler(double x,double y,double z,double angle) {
	double s = sin(angle);
	double c = cos(angle);
	double t = 1 - c;

	double * euler = new double[3];


    double heading, attitude, bank;
	if ((x*y*t + z*s) > 0.998) { // north pole singularity detected
		heading = 2*atan2(x*sin(angle/2),cos(angle/2));
		attitude = PI/2;
		bank = 0;
	}
	else if ((x*y*t + z*s) < -0.998) { // south pole singularity detected
		heading = -2*atan2(x*sin(angle/2),cos(angle/2));
		attitude = -PI/2;
		bank = 0;
	}
	else{
        heading = atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t);
        attitude = asin(x * y * t + z * s) ;
        bank = atan2(x * s - y * z * t , 1 - (x*x + z*z) * t);
    }
    euler[0] = to_angle(heading);
    euler[1] = to_angle(attitude);
    euler[2] = to_angle(bank);

    return euler;
}

/*
Heading = rotation about y axis
Attitude = rotation about z axis
Bank = rotation about x axis
*/

double *get_eulers(const double * vect1,const  double * vect2){
    //copy to internal representation
    double v1[]={vect1[0],vect1[1],vect1[2]};
    double v2[] = {vect2[0],vect2[1],vect2[2]};

    norm3(v1);
    norm3(v2);

    double * axis = crossproduct(v1,v2);

    norm3(axis);

    double angle =  get_angle_between(v1,v2);
    double * euler= to_euler(axis[0],axis[1],axis[2], angle);

    delete [] axis;

    return euler;
}

double * get_eulers_to_g(const double * vect){
        double * eulers = new double[3];
		
		double v2[] = {vect[0],vect[1],vect[2]};
		norm3(v2);
		
        eulers[0] = to_angle(atan2(v2[0],sqrt(v2[1]*v2[1]+v2[2]*v2[2])));
        eulers[2] = -to_angle(atan2(v2[1],sqrt(v2[0]*v2[0]+v2[2]*v2[2])));
        eulers[0] =0.0;
        return eulers;
}

/*
#include <iostream>
using namespace std;
#define LOG(x) cout<<#x<<"="<<x;
template<class T>
void write_array(T* s, T* e){ T* ptr = s; while(s<e){ cout<<*s++<<","; } cout<<endl; }

int main(void){
        double v2[] = {0.6,0.6,1.0};
        double v1[] = {0.0,0.0,1.0};

        norm3(v2);

        double * eulers = get_eulers(v1,v2);

        write_array(eulers, eulers+3);

        double eulers_lol[] = {
            to_angle(atan2(v2[0],sqrt(v2[1]*v2[1]+v2[2]*v2[2]))),
            -to_angle(atan2(v2[1],sqrt(v2[0]*v2[0]+v2[2]*v2[2]))),0.0};


        write_array(eulers_lol, eulers_lol+3);


        return 1;
}*/
