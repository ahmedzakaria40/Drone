#include <math.h>
#include <stdio.h>
#include "physics.h"

DroneState derivative(DroneState s , double tau ,double T , Parameters p){
    DroneState d ;
    d.x = s.vx ;
    d.y=s.vy ;
    d.theta =s.omega;

    d.vx = -(T/p.mass) * sin(s.theta) - (p.drag/p.mass) * s.vx ;
    d.vy= (T/p.mass) * cos(s.theta) - p.g - (p.drag / p.mass) * s.vy ;
    d.omega = tau / p.Inertia ; 
    return d ;
}
DroneState euler_step(DroneState s , ControlInput u , Parameters p , double h){
    DroneState d=derivative(s, u.torque ,u.thrust, p);
    DroneState n ;
    n.x=s.x + h*d.x ;
    n.y=s.y + h*d.y ;
    n.vx=s.vx + h*d.vx ;
    n.vy=s.vy + h*d.vy ;
    n.theta = s.theta + h*d.theta ;
    n.omega = s.omega + h*d.omega ;
    return n ;
}

int main(void){
    DroneState n={0} ;
    FILE *f= fopen("traj.csv","w");
    fprintf(f,"t,x,y,theta\n") ;
    Parameters p ={1.0,0.01,0.0,9.81} ;
    ControlInput u ={12.0 ,0.0} ;
    for (int i =0 ; i<100 ; i++){
        n=euler_step(n,u,p,0.01);
        fprintf(f,"%f %f %f %f\n",(i+1)*0.01, n.x ,n.y,n.vx);
    }
    fclose(f) ;
    return 0;
}

