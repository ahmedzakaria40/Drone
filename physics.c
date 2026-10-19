#include <stdio.h>
double f(double x){
    return x ;
}
double Euler_integrator(double *S , int t,double h ){
    if (t == 0){
        return S[0] ;
    }
    double prev = Euler_integrator(S,t-1,h);
    return prev + h * f(prev) ; 
}
int main(void){ 
    return 0 ;
}
