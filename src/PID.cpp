#include "PID.h"

#define sgn(_n) (_n > 0) * 1 + (_n < 0) * -1

PID::PID(double _kP, double _kI, double _kD, const char _id) :
    target(0), ID(_id), kP_(_kP), kI_(_kI), kD_(_kD), min_output_(-12000), max_output_(12000), 
    max_time_(9999), max_completion_error_(5), min_derivative_(0), integral_limit_(9999), derivative_gain_(0.8) {}

double PID::Calculate(double _sensorVal)
{
    uint32_t time_difference = millis() - last_time_;
    last_time_ = millis();

    error_ = target - _sensorVal; //Calculate error_.
    
    integral_ += error_ * time_difference;

    //Calculate integral_ (If conditions are met).
    integral_ = (abs(integral_) > integral_limit_) ? integral_limit_ * sgn(integral_) : integral_;
    if ((sgn(integral_) != sgn(error_) || fabs((float)error_) > 50)) integral_ = 0;

    derivative_ = ((_sensorVal - last_value_) / time_difference) * derivative_gain_ + last_derivative_ * (1 - derivative_gain_); // Derivative filtering

    last_derivative_ = derivative_;
    last_value_ = _sensorVal;
    
    double output = kP_ * error_ + kI_ * integral_ + kD_ * derivative_;;//Calculate output.
    
    //Restrict output to max/min.
    if (output > max_output_)
        output = max_output_;
    else if (output < min_output_)
        output = min_output_;

    //Save previous error_.
    past_error_ = error_;
    
    return output;
}

bool PID::Done() 
{
    /**
     * TODO: More complete conditions
     * 
     * If Robot is stuck, and unable to move (change in error_ is very small)
     * If Robot is low on battery or sucks ('current time' - start_time is greater than max_time_) 
    **/
      
      
    if (abs(error_) <= max_completion_error_) //If error_ is within reasonable range
        return true;

    return false;
}

void PID::SetTarget(double _target)
{
    target = _target;
}

void PID::StartTimer()
{
    start_time_ = millis();
}

void PID::ResetPID()
{
    error_ = 0;
    past_error_ = 0;
    integral_ = 0;
    derivative_ = 0;
    start_time_ = 0;
}

//Gets the target
int PID::GetTarget()
{
    return target;
}