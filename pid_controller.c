#include <stdio.h>
#include "pid.h"
# define TIME_STEP 0.01

void pid_init(pid*p, float kp , float ki , float kd ){
    p->kp =kp ;
    p->kd=kd ;
    p->ki=ki ;
    p->prev_error = 0 ;
    p->integral_sum = 0 ;
}

pid *pid_update(float setpoint , float measured, pid *pid_controller){
    float e = setpoint - measured ; 
    float proportionel = pid_controller->kp * e ;
    
    float integra = ((pid_controller->prev_error + e)/2)*TIME_STEP ; // he used rule of trapeze
    float integrateur = pid_controller->ki * integra  ;

    float deriva = (e - pid_controller->prev_error)/TIME_STEP ;
    float derivateur = pid_controller->kd * deriva ;

    pid_controller->output = proportionel + derivateur + integrateur ;
    pid_controller->prev_error = e ;
    return pid_controller ; 
} // la dimention de ouput est N (Newton) 
int main(void) {
    pid c;
    pid_init(&c, 4, 0, 4);
    float x = 0, v = 0;                         // the system: a unit mass
    c.prev_error = 1.0f - x;                    // avoid a derivative spike on the first call

    for (int k = 0; k < 800; k++) {
        float u = pid_update(1.0f, x, &c)->output;   // call, then read the command
        v += u * TIME_STEP;                          // u is an acceleration
        x += v * TIME_STEP;                          // velocity moves the position
        printf("%f,%f\n", k * TIME_STEP, x);
    }
    return 0;
}
