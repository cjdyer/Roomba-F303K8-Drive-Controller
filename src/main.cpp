#include "Imu.h"
#include "madgwick_filter.h" 
#include "MotorManager.h"

uint32_t t, dt, t0;
float dt_s;

float ax, ay, az, gx_rps, gy_rps, gz_rps;
float yaw_angle, initial_heading;
float position_x, position_y;

uint8_t gyro_iterations = 0;

IMU imu;
MotorManager motorManager;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("here");

    imu.begin();  

    uint32_t timer_1s = millis();

    while(true) // calibrate madgwick
    {
        t  = micros();
        dt = (t - t0);  // in us
        dt_s = (float) (dt) * 1.e-6;	// in s
        t0 = t;

        imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
        // Beta initial value - 2.5 to ensure convergence of algorithm states
        // Set Beta to a lower value after 10 seconds  
        // 0.033 ideal dynamic performance | 0.01 Ideal static performance
        velocity_to_angles(dt_s, 2.5, ax, ay, az, gx_rps, gy_rps, gz_rps, yaw_angle); // Only Yaw is currently calculated

        if (millis() - timer_1s > 1000) { break; } // calibrate for 1s
    }

    initial_heading = yaw_angle;
    motorManager.driveTo(100, 0);
    Serial.println("Init Complete");

    motorManager.attachInterrupts();
}

void loop()
{
    t  = micros();
	dt = (t - t0);  // in us
    dt_s = (float) (dt) * 1.e-6;	// in s
    t0 = t;

	// Read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
    // Calculate heading 
	velocity_to_angles(dt_s, 0.033, ax, ay, az, gx_rps, gy_rps, gz_rps, yaw_angle);
    // Run motors/PIDs
    // motorManager.run(dt_s, yaw_angle);
    // motorManager.getPosition(position_x, position_y);

    static uint32_t timer_50ms = millis();
    if (millis() - timer_50ms > 50) 
    {
        timer_50ms = millis();
        Serial.print("X :  ");
        Serial.print(position_x, 2);  
        Serial.print("  Y :  ");
        Serial.println(position_y, 2);
    }
}