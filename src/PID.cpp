#include "PID.h"

int8_t sgn(int32_t _n)
{
    return (_n > 0) * 1 + (_n < 0) * -1;
}

PID::PID(const float _kP, const float _kI, const float _kD) :
    target(0), kP_(_kP), kI_(_kI), kD_(_kD), min_output_(255), max_output_(255), max_completion_error_(5), 
    min_derivative_(0), integral_limit_(1000000), max_time_(255), derivative_gain_(0.8) {}

double PID::calculate(int32_t _sensor_value)
{
    uint32_t current_time = micros();
    uint32_t time_difference = current_time - last_time_;
    last_time_ = current_time;

    error_ = target - _sensor_value; //Calculate error_.
    
    integral_ += error_ * time_difference;

    //Calculate integral_ (If conditions are met).
    integral_ = (abs(integral_) > integral_limit_) ? integral_limit_ * sgn(integral_) : integral_;
    if ((sgn(integral_) != sgn(error_) || abs(error_) > 500)) integral_ = 0;

    derivative_ = ((_sensor_value - last_sensor_value_) / time_difference) * derivative_gain_ + last_derivative_ * (1 - derivative_gain_); // Derivative filtering

    last_derivative_ = derivative_;
    last_sensor_value_ = _sensor_value;
    
    double output = kP_ * error_ + kI_ * integral_ + kD_ * derivative_;//Calculate output.
    
    //Restrict output to max/min.
    if (output > max_output_)
        output = max_output_;
    else if (output < -min_output_)
        output = -min_output_;

    //Save previous error_.
    past_error_ = error_;
    
    return output;
}

bool PID::done() 
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

void PID::setTarget(int32_t _target)
{
    target = _target;
}

void PID::startTimer()
{
    start_time_ = micros();
}

void PID::reset()
{
    error_ = 0;
    past_error_ = 0;
    integral_ = 0;
    derivative_ = 0;
    start_time_ = 0;
}

int PID::getTarget()
{
    return target;
}