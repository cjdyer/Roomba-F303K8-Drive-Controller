#ifndef MOTOR_PID_H
#define MOTOR_PID_H

class motorPID
{
public:
	long encoderPos = 0; // Current encoder position since the last clearance
	double speedTotal = 0;		  // Sum of speed measurements
	double speedError = 0;		  // Difference between
	double speedError_pre = 0;	  // Error
	double speedErrorSum = 0;	  // Sum of errors
	double speed = 0;			  // Calculated wheel speed in RPM
	double speedDesired = 0;	  // PID Set Point
	double speedPWM = 0;		  // RPM
	double kp = 0;				  //0.02; // Proportional coefficient
	double ki = 0;				  //16.0; // Integral coefficient
	double kd = 0;				  //0.01; // Derivative coefficient

	float lastKnownPos = 0; // Last Known Position
	float ticks_per_millisecond = 0;

	long wheelSpeedDistance = 0; // RPM
};

#endif