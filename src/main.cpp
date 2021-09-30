#include "Imu.h"
#include "MadgwickAHRS.h"

uint32_t t, dt, t0;
float dt_s;

int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
float roll_angle, pitch_angle, yaw_angle, roll_angle_accel, pitch_angle_accel;

uint8_t gyro_iterations = 0;
float yaw_init;

IMU imu;
MADGWICK_AHRS madgwickFilter(10);

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

	madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, 0, 0, 0, roll_angle, pitch_angle, yaw_angle);

    static uint32_t timer_500ms = millis();
    if (millis() - timer_500ms > 500) 
    {
        timer_500ms = millis();

        if (gyro_iterations > 21)
        {
            Serial.print("Roll :  ");
            Serial.print(roll_angle, 5);  
            Serial.print("  Pitch :  ");
            Serial.print(pitch_angle, 5);  
            Serial.print("  Yaw :  ");
            Serial.println(yaw_angle - yaw_init, 5);
        }
        else if (gyro_iterations > 20)
        {
            madgwickFilter.set_beta(0.033);
            yaw_init = yaw_angle;
            Serial.println("Gyro Ready");
            gyro_iterations++;
        }
        else 
        {
            gyro_iterations++;
        }
    }
}