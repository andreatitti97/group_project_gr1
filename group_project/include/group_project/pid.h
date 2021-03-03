/* PID class implementation
 * 
 * Dal main viene chiamato il PID() constructor. Le funzioni sono pubblche, ci si puo accedere
 * dopo aver dichiarato la classe nel fail main tramite per esempio PID pidx = PID() --> guardare test_follow_mul...cpp
 * 
 * Costruita la classe PID viene istanziato un puntatore alla classe PIDImpl, la quale è privata e l'utenet non puo 
 * accedervi direttamente ma solo tramite gli output delle due funzioni dichiarate nella parte pubblica di PID.
 * 
 * La classe PIDImpl è definita nel file pid.cpp


*/



#ifndef _PID_H_
#define _PID_H_





class PIDImpl;

class PID
{
    public:
    
    // dt : loop time interval
    //max : saturation to max value 
    //min : saturation to min value 
    //Kp  : Proportional gain 
    //Kd : derivative gain
    // Ki : integral gain;
    
    //Class constructor
    PID(double dt, double max, double min, double Kp, double Kd, double Ki);
    
    //Return output control value
    
    double calculate(double des_value, double actual_value); //if velocity is not known --> it is evaluated inisde the evaluation
    
    double position_control_knowing_velocity(double des_pos_value, double actual_pos_value, double des_vel, double actual_vel);
    
    //Destructor
    ~PID();
    
    private:
    //call the class PID implementation
    
       PIDImpl *pimpl; //mi riferisco alla classe PIDimpl tramite il puntatore *impl
    
};

#endif

