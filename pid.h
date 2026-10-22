typedef struct {
    float kp ; // correcteur proportionnel 
    float kd ;// correcteur derivateur 
    float ki ;// correcteur integrateur 
    float integral_sum ; 
    float prev_error ;
    float output ; // controller output ;
}pid ;

