#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

void velocity_to_angles(const float dt_s, const float __beta, const float ax, const float ay, const float az, const float gx, const float gy, const float gz, float &angle_x, float &angle_y, float &angle_z);

float invSqrt(float x);

#endif