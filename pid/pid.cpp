#include "pid.h"

PID::PID(double Kp, double Ki, double Kd, double min, double max, double integral_limit){
    
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _min = min;
    _max = max;
    _previous_error = 0;
    _integral = 0;
    _integral_limit = integral_limit;
   
}

double PID::calculate(double actual, double target, double dt){

    //calculate error
    double error = target - actual;
    
    //proportional term
    this->p_out = _Kp * error;
    
    //integral term
    _integral += error * dt;
    if(_integral > _integral_limit){
        _integral = _integral_limit;
    }else if(_integral < -_integral_limit){
        _integral = -_integral_limit;
    }
    this->i_out = _Ki * _integral;
    
    //derivative term
    double derivative = (error - _previous_error) / dt;
    this->d_out = _Kd * derivative;
    
    //calculate total output
    double output = this->p_out + this->i_out + this->d_out;
    
    //restrict to max/min
    if( output > _max )
    output = _max;
    else if( output < _min )
    output = _min;
    
    //save error to previous error
    _previous_error = error;
    
    return output;
       
    
    
}