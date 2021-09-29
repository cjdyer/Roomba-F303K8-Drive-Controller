#ifndef CUSTOM_IMU_H
#define CUSTOM_IMU_H

#include <Arduino.h>
#include <SPI.h>

class ICM
{
public:
    ICM(void);

    void init();
    
    void write_register(uint16_t addr, uint8_t data);
    void read_register(uint16_t addr, uint8_t numBytes, uint8_t *data);
    void select_bank(uint8_t bank);

    void calibrate(const float time_s);

    void get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz);
    void set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz);

    void get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az);
    void set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az);

    void read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps);

    void mean_accel_gyro(const float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz);
    void read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

    SPIClass &m_port;
    const SPISettings spi_settings;
    static constexpr uint8_t M_CS_PIN = 10;

    static constexpr float RAD2DEG =  4068.0f / 71.0f;
    static constexpr float m_accelRes = 8.0f / 32768.0f;
    static constexpr float m_gyroRes = 1000.0f / 32768.0f;
    static constexpr float m_gyroRes_rad = m_gyroRes / RAD2DEG;
    
    static constexpr int16_t m_g = 4096; //0x1000 gravity offset
};

#endif