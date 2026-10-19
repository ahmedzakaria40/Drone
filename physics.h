typedef struct {
    double x ;
    double y; 
    double vx ;
    double vy ;
    double  theta ;
    double omega ;
}DroneState; /*present drone coordinations */

typedef struct {
    double mass ; 
    double Inertia ; 
    double drag ; 
    double g ; 
}Parameters ; 

typedef struct {
    double thrust ; 
    double torque ; 
}ControlInput ;
