#include "PID.h"

int8_t sgn(int32_t _n)
{
    return (_n > 0) * 1 + (_n < 0) * -1;
}

PID::PID(const float _kP, const float _kI, const float _kD) : kP_(_kP), kI_(_kI), kD_(_kD) {}

int16_t PID::calculate(int32_t _sensor_value)
{
    static int32_t last_sensor_value;
    static double last_derivative;
    static uint32_t last_time;

    uint32_t current_time = micros();
    uint16_t time_difference = current_time - last_time;
    last_time = current_time;

    error_ = target - _sensor_value; //Calculate error_
    
    integral_ += error_ * time_difference;

    //Calculate integral_ (If conditions are met).
    integral_ = (abs(integral_) >= integral_limit_) * (integral_limit_ * sgn(integral_)) + (abs(integral_) < integral_limit_) * integral_;
    integral_ = !(sgn(integral_) != sgn(error_) || abs(error_) > 50) * integral_;

    derivative_ = (abs(_sensor_value - last_sensor_value) / time_difference) * derivative_gain_ + last_derivative * (1 - derivative_gain_); // Derivative filtering

    last_derivative = derivative_;
    last_sensor_value = _sensor_value;
    
    int16_t output = kP_ * error_ + kI_ * integral_ + kD_ * derivative_;//Calculate output.
    
    //Restrict output to max/min.
    output = (max_output_ - output) * (output > max_output_) - (min_output_ + output) * (output < -min_output_) + output;
    
    return output;
}

bool PID::done() 
{
    /**
     * TODO: More complete conditions
     * 
     * If Robot is stuck, and unable to move (change in error_ is very small)
     * If Robot is low on battery or sucks ('current time' - start_time is greater than max_time_) 
     * If Robot move is time sensitive (same as above) 
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
    integral_ = 0;
    derivative_ = 0;
    start_time_ = 0;
}

int PID::getTarget()
{
    return target;
}