#ifndef CUSTOM_IMU_H
#define CUSTOM_IMU_H

#include <Arduino.h>
#include <SPI.h>
#include "alt_IMU_REGS.h"

class ICM
{
public:
    ICM(void);

    bool init();
    
    void write_register(uint16_t addr, uint8_t data);
    void read_register(uint16_t addr, uint8_t numBytes, uint8_t *data);
    void select_bank(uint8_t bank);

    void read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data);
    void write_mag_register(uint8_t addr, uint8_t data);
    void set_mag_transfer(bool read);
    uint32_t set_mag_mode(uint8_t magMode);

    uint32_t set_accel_bandwidth(uint8_t accelBw);
    uint32_t set_accel_fullscale(uint8_t accelFs);
    uint32_t set_gyro_bandwidth(uint8_t gyroBw);
    uint32_t set_gyro_fullscale(uint8_t gyroFs);

    void calibrate_gyro(float time_s);
    void calibrate_accel(float time_s);
    void calibrate_mag(float time_s);

    uint32_t get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz);
    uint32_t set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz);

    uint32_t get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az);
    uint32_t set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az);

    uint32_t min_max_mag(float time_s, int16_t &min_mx, int16_t &max_mx, int16_t &min_my, int16_t &max_my, int16_t &min_mz, int16_t &max_mz);
    bool read_mag(int16_t &mx, int16_t &my, int16_t &mz);

    void read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps);

    uint32_t mean_accel_gyro(float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz);
    bool read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);


    /* SPI variables */
    const uint8_t M_CS_PIN;
    SPIClass &m_port;
    const uint32_t M_CLOCK;
    const uint8_t M_BIT_ORDER;
    const uint8_t M_DATA_MODE;

    float m_accelRes;
    float m_gyroRes;
    float m_gyroRes_rad;
    const float m_magRes = 4912.0f / 32752.0f;    /*    Measurement range of each axis +-4912 uT is saved in 16 bit output +-32752    */
    
    int16_t m_g;    /*  Acceleration of gravity in LSB  */

    int16_t offset_gx_1000dps;
    int16_t offset_gy_1000dps;
    int16_t offset_gz_1000dps;
    int16_t offset_ax_32g; 
    int16_t offset_ay_32g; 
    int16_t offset_az_32g;

    /* Magnetometer hard iron distortion correction */
    float m_offset_mx = 0, m_offset_my = 0, m_offset_mz = 0;
    /* Magnetometer soft iron distortion correction */
    float m_scale_mx = 1, m_scale_my = 1, m_scale_mz = 1;

    float offset_mx, offset_my, offset_mz;
	float scale_mx, scale_my, scale_mz;
};

#endif