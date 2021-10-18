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
    PID(const float _kP, const float _kI, const float _kD);
    

    /**
     * Calculate power output for motor, given sensor value
     *
     * @param _sensorVal current value of affiliated sensor
     * 
     * @return The power for related motor
    **/
    int16_t calculate(int32_t _sensorVal);
    
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
    
protected:
    int32_t target;
        
private:
    const float kP_;
    const float kI_;
    const float kD_;

    static constexpr uint8_t min_output_ = 255;
    static constexpr uint8_t max_output_ = 255;
    static constexpr uint8_t max_completion_error_ = 5;
    static constexpr uint8_t min_derivative_ = 0;
    static constexpr uint16_t max_time_ = 255;
    static constexpr uint32_t integral_limit_ = 100000;
    static constexpr float derivative_gain_ = 0.8;

    int32_t error_; // Should theoretically be int64_t but.... (in the case of INT32_MAX - (-INT32_MAX))
    int32_t integral_;
    int32_t derivative_;
    uint32_t start_time_;
};

#endif