#include "Imu.h"
#include "madgwick_filter.h"

uint32_t t, dt, t0;
float dt_s;

float ax, ay, az, gx_rps, gy_rps, gz_rps;
float roll_angle, pitch_angle, yaw_angle, roll_angle_accel, pitch_angle_accel, initial_heading;

uint8_t gyro_iterations = 0;

IMU imu;
// Beta initial value - 2.5 to ensure convergence of algorithm states
// Set Beta to a lower value after 10 seconds  
// 0.033 ideal dynamic performance | 0.01 Ideal static performance
float beta = 2.5;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    imu.begin();  
}

void loop()
{
    t  = micros();
	dt = (t - t0);  // in us
    dt_s = (float) (dt) * 1.e-6;	// in s
    t0 = t;

	// read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);

	velocity_to_angles(dt_s, beta, ax, ay, az, gx_rps, gy_rps, gz_rps, roll_angle, pitch_angle, yaw_angle);

    static uint32_t timer_50ms = millis();
    if (millis() - timer_50ms > 50) 
    {
        timer_50ms = millis();
        if (gyro_iterations > 21)
        {
            Serial.print("Roll :  ");
            Serial.print(roll_angle, 5);  
            Serial.print("  Pitch :  ");
            Serial.print(pitch_angle, 5);  
            Serial.print("  Yaw :  ");
            Serial.println(yaw_angle - initial_heading, 5);
        }
        if (gyro_iterations <= 20) // Wonky but legit logic
        {
            if (gyro_iterations == 20)
            {
                beta = 0.033; 
                initial_heading = yaw_angle; // After Madgwick filter is in steady-state set heading.
                gyro_iterations++;
            }
            else 
            {
                gyro_iterations++;
            }
        }
    }
}