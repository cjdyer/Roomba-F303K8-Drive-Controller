#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID 
{    
public:
    /**
     * Initialize new PID object with PID constants
     *
     * @param _kP Proportional multiplier
     * @param _kI Integral multiplier
     * @param _kD Derivative multipler
     * @param _id ID of component PID is controlling
    **/
    PID (double _kP, double _kI, double _kD, const char _id);
    

    /**
     * Calculate power output for motor, given sensor value
     *
     * @param _sensorVal current value of affiliated sensor
     * 
     * @return The power for related motor
    **/
    double Calculate(double _sensorVal);
    
    /**
     * Has the PID control finished?
     * 
     * @return true is PID is completed, flase if not
    **/
    bool Done();
    
    /**
     * Set a new target (set point) for the PID controller
     *
     * @param _target the desired finishing sensor value
    **/
    void SetTarget(double _target);
    
    /**
     * Starts the PID timer
     * This allows for done() due to timeout
    **/
    void StartTimer();

    /**
     * Reset the error, integral, and derivative terms
     * 
     * This is for when a completely new target is being set,
     * and previos values need to be cleared.
    **/
    void ResetPID();

    /**
     * Getter function for the PID's target
     * 
     * @return the PID target
    **/
    int GetTarget();

public:
    const char ID;
    
protected:
    double target;
        
private:
    const float kP_;
    const float kI_;
    const float kD_;

    const uint8_t min_output_;
    const uint8_t max_output_;
    const uint8_t max_time_;
    const uint8_t max_completion_error_;
    const uint8_t min_derivative_;
    const uint16_t integral_limit_;
    const double derivative_gain_;

    double error_;
    double past_error_;
    double integral_;
    double derivative_;
    double start_time_;
    double last_value_;
    double last_time_;
    double last_derivative_;

;
};

#endif