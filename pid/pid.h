#include "mbed.h"

class PID {
    public:
        PID(double Kp, double Ki, double Kd, double min, double max, double integral_limit);
        double calculate(double actual, double target, double dt);
    
    private:
        double _Kp;
        double _Ki;
        double _Kd;
        double _min;
        double _max;
        double _previous_error;
        double _integral;
        double _integral_limit;
        
};