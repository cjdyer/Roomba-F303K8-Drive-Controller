#include "madgwick_filter.h"

#include <Arduino.h>

// factor for converting a radian number to an equivalent number in degrees
const float RAD2DEG = (float) 4068 / 71;

// get pose in euler angles and quaternion form
void velocity_to_angles(const float dt_s, const float _beta, float ax, float ay, float az, const float gx, const float gy, const float gz, float &angle_x, float &angle_y, float &angle_z) {
    static float recipNorm;
	static float s0, s1, s2, s3;
	static float qDot1, qDot2, qDot3, qDot4;
	static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // normalize accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;       

    // normalize step magnitude
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= _beta * s0;
    qDot2 -= _beta * s1;
    qDot3 -= _beta * s2;
    qDot4 -= _beta * s3;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt_s;
	q1 += qDot2 * dt_s;
	q2 += qDot3 * dt_s;
	q3 += qDot4 * dt_s;

	// normalize quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    angle_x = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * RAD2DEG;
	angle_y = asinf(-2.0f * (q1*q3 - q0*q2)) * RAD2DEG;
	angle_z = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * RAD2DEG;
}

// Fast inverse square-root
float invSqrt(const float x) {
	union {
		float f;
		uint32_t i;
	} conv = {x};	// member 'f' set to value of 'x'
		
	conv.i = 0x5f3759df - (conv.i >> 1);
	conv.f *= (1.5f - (0.5f * x * conv.f * conv.f));	// 1st iteration of the newton method
	// conv.f *= (1.5f - (0.5f * x * conv.f * conv.f));	// 2nd iteration of the newton method
	
	return conv.f;
}