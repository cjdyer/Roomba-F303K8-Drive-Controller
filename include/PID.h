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
    **/
    PID (const float _kP, const float _kI, const float _kD);
    

    /**
     * Calculate power output for motor, given sensor value
     *
     * @param _sensorVal current value of affiliated sensor
     * 
     * @return The power for related motor
    **/
    double calculate(int32_t _sensorVal);
    
    /**
     * Has the PID control finished?
     * 
     * @return true is PID is completed, flase if not
    **/
    bool done();
    
    /**
     * Set a new target (set point) for the PID controller
     *
     * @param _target the desired finishing sensor value
    **/
    void setTarget(int32_t _target);
    
    /**
     * Starts the PID timer
     * This allows for done() due to timeout
    **/
    void startTimer();

    /**
     * Reset the error, integral, and derivative terms
     * 
     * This is for when a completely new target is being set,
     * and previos values need to be cleared.
    **/
    void reset();

    /**
     * Getter function for the PID's target
     * 
     * @return the PID target
    **/
    int getTarget();
    
protected:
    int32_t target;
        
private:
    const float kP_;
    const float kI_;
    const float kD_;

    const uint8_t min_output_;
    const uint8_t max_output_;
    const uint8_t max_completion_error_;
    const uint8_t min_derivative_;
    const uint16_t integral_limit_;
    const uint16_t max_time_;
    const float derivative_gain_;

    int32_t error_; // Should theoretically be int64_t but.... (in the case of INT32_MAX - (-INT32_MAX))
    int32_t past_error_;
    int32_t integral_;
    uint32_t last_time_;
    uint32_t start_time_;

    double derivative_;
    double last_sensor_value_;
    double last_derivative_;
};

#endif