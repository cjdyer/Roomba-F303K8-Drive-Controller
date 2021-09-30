#include "custom_IMU.h"
#include "MadgwickAHRS.h"

#define BETA		0.033
#define BETA_INIT	10

const float RAD2DEG = (float) 4068 / 71;

uint32_t t0, t;
int32_t dt;
float dt_s;

int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;

float roll_angle, pitch_angle, yaw_angle;	// euler angles
float roll_angle_accel, pitch_angle_accel;

bool init_set = false;
bool beta_settled = false;
float yaw_angle_init = -1000;

IMU imu;
MADGWICK_AHRS madgwickFilter(BETA_INIT);

void accelAngles(float& roll_angle_accel, float& pitch_angle_accel);

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    imu.begin();
}

void loop()
{
    // update time
	t = micros();
	dt = (t - t0);  // in us
	dt_s = (float) (dt) * 1.e-6;	// in s

    t0 = t;
	
	// read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);

	// TODO: Check if there is a benefit from magnetometer data
	madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, 0, 0, 0, roll_angle, pitch_angle, yaw_angle);

    if (!beta_settled)
    {
        accelAngles(roll_angle_accel, pitch_angle_accel);
        if ((abs(roll_angle_accel - roll_angle) < 0.5) && (abs(pitch_angle_accel - pitch_angle) < 0.5) && ((abs(yaw_angle - yaw_angle_init) / dt_s) < 0.05)) {
            madgwickFilter.set_beta(BETA);
                
            Serial.println("Initial pose estimated.");

            beta_settled = true;
        }
        yaw_angle_init = yaw_angle;
    }
    else
    {
        static uint32_t t0_serial = millis();
        if (millis() - t0_serial > 500) {
            t0_serial = millis();
        
            Serial.print("Roll :  ");
            Serial.print(roll_angle, 5);  
            Serial.print("  Pitch :  ");
            Serial.print(pitch_angle, 5);  
            Serial.print("  Yaw :  ");
            Serial.println(yaw_angle - yaw_angle_init, 5);
        }
    } 
}

void accelAngles(float& roll_angle_accel, float& pitch_angle_accel) {
	roll_angle_accel = atan2(ay, az) * RAD2DEG;
	pitch_angle_accel = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * RAD2DEG;
}