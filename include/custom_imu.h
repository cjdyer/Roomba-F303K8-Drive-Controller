#ifndef CUSTOM_IMU_H
#define CUSTOM_IMU_H

#include <Arduino.h>
#include <SPI.h>

class ICM
{
public:
    ICM(void);

    void init();
    
    void read_register(uint16_t addr, uint8_t numBytes, uint8_t *data);
    void write_register(uint16_t addr, uint8_t data);
    void select_bank(uint8_t bank);

    void read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data);
    void write_mag_register(uint8_t addr, uint8_t data);
    void set_mag_transfer(bool read);

    void calibrate(const float time_s);
    bool calibrate_mag(float time_s, int32_t mag_minimumRange, float &offset_mx, float &offset_my, float &offset_mz, float &scale_mx, float &scale_my, float &scale_mz);

    void get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz);
    void set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz);

    void get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az);
    void set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az);

    void read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps);

    void mean_accel_gyro(const float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz);
    void read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

    bool read_mag_ut(float &mx_uT, float &my_uT, float &mz_uT);

    bool read_mag(int16_t &mx, int16_t &my, int16_t &mz);
    void min_max_mag(float time_s, int32_t mag_minimumRange, int16_t &min_mx, int16_t &max_mx, int16_t &min_my, int16_t &max_my, int16_t &min_mz, int16_t &max_mz);

    SPIClass &m_port;
    const SPISettings spi_settings;
    static constexpr uint8_t M_CS_PIN = 10;

    float m_offset_mx = 0, m_offset_my = 0, m_offset_mz = 0;
    float m_scale_mx = 1, m_scale_my = 1, m_scale_mz = 1;

    static constexpr float RAD2DEG =  4068.0f / 71.0f;
    static constexpr float m_accelRes = 8.0f / 32768.0f;
    static constexpr float m_gyroRes = 1000.0f / 32768.0f;
    static constexpr float m_gyroRes_rad = m_gyroRes / RAD2DEG;
    static constexpr float m_magRes = 4912.0f / 32752.0f; 
    
    static constexpr int16_t m_g = 4096; //0x1000 gravity offset
};

#endif