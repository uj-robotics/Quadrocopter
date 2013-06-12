#include <iostream>
#include <cmath>
using namespace std;

double dotproduct(double* v1, double* v2){
       return v1[0]*v2[0] + v1[1]*v2[1] + v2[2]*v1[2];       
}
double * crossproduct(double* v1 , double* v2){
       double c1 = v1[1]*v2[2] - v1[2]*v2[1];
       double c2 = v1[2]*v2[0] - v1[0]*v2[2];
       double c3 = v1[0]*v2[1] - v1[1]*v2[0];
       return new double[]{c1,c2,c3};
}

int main(){
    
    
    
    return 0;   
}
