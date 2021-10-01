#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "utility/spi_com.h"

class IMU
{
public:
    IMU();

    void begin();
    void read_accel_gyro_rps(float &_accel_x, float &_accel_y, float &_accel_z, float &_gyro_rps_x, float &_gyro_rps_y, float &_gyro_rps_z);

private:
    void read_register(const uint16_t _address, const uint8_t _number_bytes, uint8_t *_data);
    void write_register(const uint16_t _address, const uint8_t _data);
    void select_bank(uint8_t bank);

    void transfer(uint8_t data);
    void transfer(void *_data, const uint8_t _number_bytes);

    void calibrate(const float time_s);

    void get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz);
    void set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz);

    void get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az);
    void set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az);


    void mean_accel_gyro(const float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz);
    void read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

    static constexpr uint8_t M_CS_PIN = 10;
    spi_t _spi;

    // gyro resolution in Radians
    // 3.28 LSB/dps - from datasheet
    // 1000 dps     - set by user
    // 4068 / 71    - Radians to degrees approx.
    static constexpr float m_gyroRes = 355.0f / 667152.0f; // (1000 / 32800)  / (4068 / 71) 
    static constexpr float m_accelRes = 16.0f / 32768.0f;
};

#endif