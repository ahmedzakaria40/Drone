#include <math.h>
#include "physics.h"
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
DroneState Derivative(DroneState s , double tau ,double T , Parameters p){
    DroneState d ;
    d.x = s.vx ;
    d.y=s.vy ;
    d.theta =s.omega;

    d.vx = -(T/p.mass) * sin(s.theta) - (p.drag/p.mass) * s.vy ;
    d.vy= (T/p.mass) * cos(s.theta) - p.g - (p.drag / p.mass) * s.vy ;
    d.omega = tau / p.Inertia ; 
    return d ;
}
int main(void){
    return 0;
}
