typedef struct {
    double x ;
    double y; 
    double vx ;
    double vy ;
    double  theta ; /* is the angular of the force of T */
    double omega ; /* omega (ω) is the angular velocity par rapport à z axis */
}DroneState; /*present drone coordinations */

typedef struct {
    double mass ; 
    double Inertia ; 
    double drag ; /* drag is the force opposed to the velocity vector */ 
    double g ; /* this is constant g=9,8 N with N is newton */ 
}Parameters ; 

typedef struct {
    double thrust ; 
    double torque ; 
}ControlInput ;
