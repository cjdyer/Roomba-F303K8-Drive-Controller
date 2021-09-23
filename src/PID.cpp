#include "PID.h"

#define sgn(_n) (_n > 0) * 1 + (_n < 0) * -1

PID::PID(double _kP, double _kI, double _kD, const char _id) :
    target(0), ID(_id), kP_(_kP), kI_(_kI), kD_(_kD), min_output_(-12000), 
    max_output_(12000), max_time_(9999), max_completion_error_(5), integral_limit_(9999), min_derivative_(0) {}

double PID::Calculate(double _sensorVal)
{
    error_ = target - _sensorVal; //Calculate error_.
    
    //Calculate integral_ (If conditions are met).
    if(abs(error_) > 650)
        integral_ = 0;
    else if (error_ == 0)
        integral_ = 0;
    else if(abs(integral_) > integral_limit_)
        integral_ = integral_limit_ * sgn(integral_);
    else
        integral_ += error_;

    derivative_ = error_ - past_error_;//Calculate derivative_.
    
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