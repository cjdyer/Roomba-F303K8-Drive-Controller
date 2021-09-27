/****************************************************************
 * Example6_DMP_Quat9_Orientation.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, April 25th, 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * ** This example is based on InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
 * ** We are grateful to InvenSense for sharing this with us.
 * 
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in:
 * ** Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/

#include "IMU.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10

ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// accelerometer resolution
float accelRes;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx, my, mz;

// quadcopter pose
float roll_angle, pitch_angle, yaw_angle;	// euler angles
float pose_q[4];	// quaternion

// filtered gyro rates
float roll_rate, pitch_rate, yaw_rate;

volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

bool imuCalibration();

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    SPI.begin();

	
    imuCalibration();
    imu.read_accelRes(accelRes);
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

    // perform sensor fusion with Madgwick filter to calculate pose
	// TODO: Check if there is a benefit from magnetometer data
	madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, 0, 0, 0, roll_angle, pitch_angle, yaw_angle, pose_q);
}

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration() {
	static uint32_t t_imuCalibration;
	t_imuCalibration = micros();
	
	static uint32_t t_calibrateGyro = 0, t_calibrateAccel = 0, t_calibrateMag = 0;
    if ((rc_channelValue[THROTTLE] < 1100) && (rc_channelValue[YAW] < 1100)) {
        // hold right stick bottom center and left stick bottom-left to start gyro calibration	(2s)
        t_calibrateAccel = 0;
        t_calibrateMag = 0;
        if (t_calibrateGyro == 0) {
            t_calibrateGyro = t_imuCalibration;
            return false;
        }
        else if ((t_imuCalibration - t_calibrateGyro) > 2000000) {
            // turn on LED to indicate calibration
            updateLED(LED_PIN, 2);
            imu.calibrate_gyro(imuInterrupt, 5.0, 1, data_eeprom.offset_gx_1000dps, data_eeprom.offset_gy_1000dps, data_eeprom.offset_gz_1000dps);
            t_calibrateGyro = 0;
            // turn off LED
            updateLED(LED_PIN, 0);
            return true;
        }
        else {
            return false;
        }
    }
    else if ((rc_channelValue[THROTTLE] > 1900) && (rc_channelValue[YAW] < 1100)) {
        // hold right stick bottom center and left stick top-left to start accel calibration	(2s)
        t_calibrateGyro = 0;
        t_calibrateMag = 0;
        if (t_calibrateAccel == 0) {
            t_calibrateAccel = t_imuCalibration;
            return false;
        }
        else if ((t_imuCalibration - t_calibrateAccel) > 2000000) {
            // turn on LED to indicate calibration
            updateLED(LED_PIN, 2);
            imu.calibrate_accel(imuInterrupt, 5.0, 16, data_eeprom.offset_ax_32g, data_eeprom.offset_ay_32g, data_eeprom.offset_az_32g);
            EEPROM.put(ADDRESS_EEPROM, data_eeprom);
            t_calibrateAccel = 0;
            // turn off LED
            updateLED(LED_PIN, 0);
            return true;
        }
        else {
            return false;
        }
    }
    else if ((rc_channelValue[THROTTLE] > 1900) && (rc_channelValue[YAW] > 1900)) {
        // hold right stick bottom center and left stick top-right to start mag calibration	(2s)
        t_calibrateGyro = 0;
        t_calibrateAccel = 0;
        if (t_calibrateMag == 0) {
            t_calibrateMag = t_imuCalibration;
            return false;
        }
        else if ((t_imuCalibration - t_calibrateMag) > 2000000) {
            imu.calibrate_mag(imuInterrupt, 60, 500, data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
            EEPROM.put(ADDRESS_EEPROM, data_eeprom);
            t_calibrateMag = 0;
            return true;
        }
        else {
            return false;
        }
    }
	
	t_calibrateGyro = 0;
	t_calibrateAccel = 0;
	t_calibrateMag = 0;
	
	return false;
}