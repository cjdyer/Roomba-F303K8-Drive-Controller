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
    void select_bank(uint8_t _bank);

    void transfer(uint8_t _data);
    void transfer(void *_data, const uint8_t _number_bytes);

    void calibrate(const float _time_s);

    void get_gyro_offsets(int16_t &_gyro_offset_x, int16_t &_gyro_offset_y, int16_t &_gyro_offset_z);
    void set_gyro_offsets(int16_t _gyro_offset_x, int16_t _gyro_offset_y, int16_t _gyro_offset_z);

    void get_accel_offsets(int16_t &_accel_offset_x, int16_t &_accel_offset_y, int16_t &_accel_offset_z);
    void set_accel_offsets(int16_t _accel_offset_x, int16_t _accel_offset_y, int16_t _accel_offset_z);

    void mean_accel_gyro(const float _calibration_time, int16_t &_mean_accel_x, int16_t &_mean_accel_y, int16_t &_mean_accel_z, int16_t &_mean_gyro_x, int16_t &_mean_gyro_y, int16_t &_mean_gyro_z);
    void read_accel_gyro(int16_t &_accel_x, int16_t &_accel_y, int16_t &_accel_z, int16_t &_gyro_x, int16_t &_gyro_y, int16_t &_gyro_z);

    static constexpr uint8_t cs_pin_ = 10;
    spi_t _spi;

    // gyro resolution in Radians
    // 3.28 LSB/dps - from datasheet
    // 1000 dps     - set by user
    // 4068 / 71    - Radians to degrees approx.
    static constexpr float gyro_res_ = 355.0f / 667152.0f; // (1000 / 32800)  / (4068 / 71) 
    static constexpr float accel_res_ = 16.0f / 32768.0f;
};

#endif