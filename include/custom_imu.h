#ifndef CUSTOM_IMU_H
#define CUSTOM_IMU_H

#include <Arduino.h>
#include <SPI.h>
#include "alt_IMU_REGS.h"

class ICM
{
public:
    ICM(void);

    void init();
    
    void write_register(uint16_t addr, uint8_t data);
    void read_register(uint16_t addr, uint8_t numBytes, uint8_t *data);
    void select_bank(uint8_t bank);

    void set_accel_bandwidth(uint8_t accelBw);
    void set_accel_fullscale(uint8_t accelFs);
    void set_gyro_bandwidth(uint8_t gyroBw);
    void set_gyro_fullscale(uint8_t gyroFs);

    void calibrate_gyro(float time_s);
    void calibrate_accel(float time_s);

    void get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz);
    void set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz);

    void get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az);
    void set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az);

    void read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps);

    void mean_accel_gyro(float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz);
    void read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);


    /* SPI variables */
    const uint8_t M_CS_PIN;
    SPIClass &m_port;
    const uint32_t M_CLOCK;
    const uint8_t M_BIT_ORDER;
    const uint8_t M_DATA_MODE;

    float m_accelRes;
    float m_gyroRes;
    float m_gyroRes_rad;
    
    int16_t m_g;    /*  Acceleration of gravity in LSB  */

    int16_t offset_gx_1000dps;
    int16_t offset_gy_1000dps;
    int16_t offset_gz_1000dps;
    int16_t offset_ax_32g; 
    int16_t offset_ay_32g; 
    int16_t offset_az_32g;
};

#endif