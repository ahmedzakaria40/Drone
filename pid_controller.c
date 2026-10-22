#include <stdio.h>
#include "pid.h"
# define TIME_STEP 0.01
float pid_controller(float setpoint , float measured ,float preverror , pid *pid){
    float e = setpoint - measured ; 
    float proportionel = pid->kp * e ;
    
    float integra = ((preverror + e)/2)*TIME_STEP ; // he used rule of trapeze
    float integrateur = pid->ki * integra  ;

    float deriva = (e - preverror)/TIME_STEP ;
    float derivateur = pid->kd * deriva ;

    float total_input = proportionel + derivateur + integrateur ;

    return total_input ; 
}

