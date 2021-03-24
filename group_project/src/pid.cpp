
/*
This file contain the PID implementation class.
It is generate every time the PID class is called in the main script
*/

/*
 * PIDimpl ha lo stesso costruttore della classe PID, in quanto allocata tramite puntatore pimpl.
 * Le funzioni sono le stesse di PID
 *
 * Nella parte private sono contenute le variabili che vengono allocate quando nel main viene chiamato il costruttore
 * della classe PID
 *
 * Nella parte di implemntazione si puo vedere che quando viene chiamato il costruttore della classe PID
 * viene attribuito al puntatore pimpl una nuova classe di PIDIMpl (quindi ogni volta che si costruisce la classe PID, vieme costruita anche la
 * classe PIDImpl)
 *
 * Le funzioni che vengono chiamate da PID traimte l'operatore punto (es: in main --> PID pid_x = PID()
 *                                                                        pid_x.calculate()
 *
 * Dalla classe PID posso accedere alle funzioni le quali vengono effettivamente chiamate dalla classe PIDImpl dove
 * sono definite.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "group_project/pid.h"
#include "group_project/Structure.h"
#include "group_project/KF.h"
using namespace std;


class PIDImpl
{
    public:

    //Class constructor, prende i termini dati in input alla class PID
        PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);

    //Destructor
        ~PIDImpl();

    //Control output if actual velocity is not known
        double calculate(double des_value, double actual_value);

    //Control output if actual velocity is known
        double position_control_knowing_velocity(double des_pos_value, double actual_pos_value, double des_vel, double actual_vel);

    private:

       double _dt;
       double _max;
       double _min;
       double _Kp;
       double _Kd;
       double _Ki;
       double _previous_des_value;
       double _pre_error;
       double _integral;


};


//Class implementation and function
//Constructor della classe PID --> costruisce ancge la classe PIDImpl associata al puntatore pimpl
PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki)
{
    pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);
}
//FUnction definition
//accedo alla funzione calculate PID tramite puntatore pimpl.
//Dalla classe PID accedo solo all'output della funzione
double PID::calculate(double des_value, double actual_value)
{
    return pimpl -> calculate(des_value, actual_value);
}

//FUnction definition
double PID::position_control_knowing_velocity(double des_pos_value, double actual_pos_value, double des_vel, double actual_vel)
{
    return pimpl -> position_control_knowing_velocity(des_pos_value, actual_pos_value, des_vel, actual_vel);
}

//Il distruttore dealloca anche la classe PIDIMPL
PID::~PID()
{
    delete pimpl;
}


/* Implementation */
//COstruttore con intialization
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
//Definisco le variabili passate da PID a PIDIMPL
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _previous_des_value(0),
    _pre_error(0),
    _integral(0)
{
}

//Implementation delle funzioni nella classe PIDIMPL, difatto l'utenet non puo accedere alle variabili intenre ma solo
// all'output
double PIDImpl::calculate(double des_value, double actual_value)
{
     if ( _previous_des_value < des_value + 0.5)  //eliminate integral sum when the setpoint change because it increase error
     {
         //cout<< "_previous_des_value: "<<_previous_des_value <<endl;
         //cout<< "des_value: "<<des_value <<endl;
         _integral = 0;
     }
    //calculate error
    double error = des_value - actual_value;

    //proportional term
    double Pout = _Kp * error;

    //Derivative Term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    //Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;


    // Save error to previous error
    _pre_error = error;
    _previous_des_value = des_value;


    return output;
}


double PIDImpl::position_control_knowing_velocity(double des_pos_value, double actual_pos_value, double des_vel, double actual_vel)
{
    //calculate error
    double error = des_pos_value - actual_pos_value;

    //proportional term
    double Pout = _Kp * error;

    //Derivative Term

    double Dout = _Kd * (des_vel - actual_vel);

    //Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

     //_integral = 0;
    return output;
}



PIDImpl::~PIDImpl()
{
}

#endif

