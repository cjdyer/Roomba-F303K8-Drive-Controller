#include "Imu.h"
#include "imu_registers.h"

IMU::IMU()
{
    pinMode(M_CS_PIN, OUTPUT);
    digitalWrite(M_CS_PIN, HIGH);

    _spi.pin_miso = digitalPinToPinName(MISO); // Configure spi pins 
    _spi.pin_mosi = digitalPinToPinName(MOSI);
    _spi.pin_sclk = digitalPinToPinName(SCK);
    _spi.pin_ssel = NC;
}

void IMU::begin()
{
    spi_init(&_spi, 7e6, SPI_MODE_3, MSBFIRST); // Begin SPI

    write_register(IMU_REG_PWR_MGMT_1, IMU_BIT_CLK_PLL | IMU_BIT_TEMP_DIS); // Enable clock, disable sleep and disable temp sensor
    write_register(IMU_REG_USER_CTRL, IMU_BIT_I2C_IF_DIS);                  // Enable SPI - Disable I2C
    delay(30);                                                              // Clock PLL start time ( Soft accel start time - average 10ms )
    write_register(IMU_REG_ACCEL_CONFIG, IMU_ACCEL_FULLSCALE_16G | IMU_ACCEL_ENABLE_DLPF | IMU_ACCEL_BW_6HZ);       // Configure Accel
    write_register(IMU_REG_GYRO_CONFIG_1, IMU_GYRO_FULLSCALE_1000DPS | IMU_BIT_GYRO_FCHOICE | IMU_GYRO_BW_12100HZ); // Configure Gyro                                                              // Accelerometer start time

    // high time = more risk of over calibration
    // low time = more risk of random correlation and worse drift
    calibrate(0.5); // Open to change in this time
}

void IMU::read_accel_gyro_rps(float &_accel_x, float &_accel_y, float &_accel_z, float &_gyro_rps_x, float &_gyro_rps_y, float &_gyro_rps_z)
{
    static int16_t gyro_x, gyro_y, gyro_z;
    static int16_t accel_x, accel_y, accel_z;

    read_accel_gyro(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);


    _accel_x    = (float)accel_x * m_accelRes;
    _accel_y    = (float)accel_y * m_accelRes;
    _accel_z    = (float)accel_z * m_accelRes;
    _gyro_rps_x = (float)gyro_x * m_gyroRes; // Convert from degrees to radians and
    _gyro_rps_y = (float)gyro_y * m_gyroRes; // scale dps simplified to one constant
    _gyro_rps_z = (float)gyro_z * m_gyroRes;
}

void IMU::read_register(const uint16_t _address, const uint8_t _number_bytes, uint8_t *_data)
{
    select_bank(_address >> 7);

    digitalWrite(M_CS_PIN, LOW);
    transfer(_address | ~IMU_REG_BANK_SEL);
    transfer(_data, _number_bytes);
    digitalWrite(M_CS_PIN, HIGH);
}

void IMU::write_register(const uint16_t _address, const uint8_t _data)
{
    select_bank(_address >> 7);

    digitalWrite(M_CS_PIN, LOW);
    transfer(_address & IMU_REG_BANK_SEL); // 127 (0x7F) all but the select REG. Address x7F
    transfer(_data);
    digitalWrite(M_CS_PIN, HIGH);
}

void IMU::transfer(uint8_t _data)
{
    spi_transfer(&_spi, &_data, &_data, 1, 1000, false);
}

void IMU::transfer(void *_data, const uint8_t _number_bytes)
{
    spi_transfer(&_spi, ((uint8_t *)_data), ((uint8_t *)_data), _number_bytes, 1000, false);
}

void IMU::select_bank(const uint8_t _bank)
{
    static uint8_t previous_bank = 10;

    if (_bank != previous_bank)
    {
        digitalWrite(M_CS_PIN, LOW);
        transfer(IMU_REG_BANK_SEL);
        transfer(_bank << 4); // First 4 bits are reserved [5:4]
        digitalWrite(M_CS_PIN, HIGH);

        previous_bank = _bank;
    }
}

void IMU::calibrate(const float _time) // Might rewite this
{
    constexpr float accel_offset_scale = 0.5f;           // 16g / 32g = 0.5
    constexpr uint8_t accel_offset_scale_reciprocal = 2; // Faster to multiply than to divide

    constexpr uint8_t accel_tolerance = 8;
    constexpr uint8_t gyro_tolerance = 1;

    bool accel_calibrated = false;
    bool gyro_calibrated = false;

    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int16_t offset_ax, offset_ay, offset_az;
    int16_t offset_gx, offset_gy, offset_gz;

    get_accel_offsets(offset_ax, offset_ay, offset_az);
    get_gyro_offsets(offset_gx, offset_gy, offset_gz);

    offset_ax *= accel_offset_scale_reciprocal; // Scale to 8g
    offset_ay *= accel_offset_scale_reciprocal; // Gyro doesnt need scaling as it is configured for 1000dps
    offset_az *= accel_offset_scale_reciprocal;

    while (!gyro_calibrated || !accel_calibrated)
    {
        mean_accel_gyro(_time, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);

        if (!accel_calibrated)
        {
            accel_calibrated = (abs(mean_ax) < accel_tolerance) && (abs(mean_ay) < accel_tolerance) && (abs(mean_az) < accel_tolerance);

            offset_ax -= mean_ax;
            offset_ay -= mean_ay;
            offset_az -= mean_az;

            set_accel_offsets(offset_ax * accel_offset_scale, offset_ay * accel_offset_scale, offset_az * accel_offset_scale); // Convert back to 32g and set
        }

        if (!gyro_calibrated)
        {
            gyro_calibrated = (abs(mean_gx) < gyro_tolerance) && (abs(mean_gy) < gyro_tolerance) && (abs(mean_gz) < gyro_tolerance);

            offset_gx -= mean_gx;
            offset_gy -= mean_gy;
            offset_gz -= mean_gz;

            set_gyro_offsets(offset_gx, offset_gy, offset_gz);
        }
    }
}

void IMU::mean_accel_gyro(const float _calibration_time, int16_t &_mean_accel_x, int16_t &_mean_accel_y, int16_t &_mean_accel_z, int16_t &_mean_gyro_x, int16_t &_mean_gyro_y, int16_t &_mean_gyro_z)
{
    static int32_t sum_accel_x, sum_accel_y, sum_accel_z, sum_gyro_x, sum_gyro_y, sum_gyro_z;
    static int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    static int32_t iterations;

    sum_accel_x = sum_accel_y = sum_accel_z = sum_gyro_x = sum_gyro_y = sum_gyro_z = 0;
    iterations = 0;

    uint32_t start_time = micros();
    const uint32_t calibration_time_us = _calibration_time * 1e6; // Seconds to micros

    while ((micros() - start_time) < calibration_time_us)
    {
        read_accel_gyro(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

        sum_accel_x += accel_x;
        sum_accel_y += accel_y;
        sum_accel_z += accel_z;
        sum_gyro_x += gyro_x;
        sum_gyro_y += gyro_y;
        sum_gyro_z += gyro_z;

        iterations++;
    }

    _mean_accel_x = sum_accel_x / iterations;
    _mean_accel_y = sum_accel_y / iterations;
    _mean_accel_z = (sum_accel_z / iterations) - 2048; // gravity offset 0x100
    _mean_gyro_x = sum_gyro_x / iterations;
    _mean_gyro_y = sum_gyro_y / iterations;
    _mean_gyro_z = sum_gyro_z / iterations;
}

void IMU::read_accel_gyro(int16_t &_accel_x, int16_t &_accel_y, int16_t &_accel_z, int16_t &_gyro_x, int16_t &_gyro_y, int16_t &_gyro_z)
{
    static uint8_t data[12]; // 12 addresses, 2 per Gryo/Accel axis [MSB, LSB]

    read_register(IMU_REG_ACCEL_XOUT_H_SH, 12, data); // Read from 0x2E to 0x39(0x2E + 11)

    _accel_x = data[0] << 8 | data[1]; // Convert 2 Bytes to 16 bit signed int [MSB << 8 | LSB]
    _accel_y = data[2] << 8 | data[3];
    _accel_z = data[4] << 8 | data[5];
    _gyro_x = data[6] << 8 | data[7];
    _gyro_y = data[8] << 8 | data[9];
    _gyro_z = data[10] << 8 | data[11];
}

void IMU::get_accel_offsets(int16_t &_accel_offset_x, int16_t &_accel_offset_y, int16_t &_accel_offset_z)
{
    uint8_t data[8]; // 8 addresses, 3 per Accel axis [MSB, LSB,  blank] (9th doesnt need to be read)

    read_register(IMU_REG_XA_OFFSET_H, 8, data); // Read from 0x14 to 0x1B(0x14 + 7)

    _accel_offset_x = data[0] << 7 | data[1] >> 1; // Convert 2 Bytes to 16 bit signed int [MSB << 7 | LSB >> 1]
    _accel_offset_y = data[3] << 7 | data[4] >> 1; // Offset low byte is 7 bits not 8. With the last bit being reserved
    _accel_offset_z = data[6] << 7 | data[7] >> 1;
}

void IMU::set_accel_offsets(int16_t _accel_offset_x, int16_t _accel_offset_y, int16_t _accel_offset_z)
{
    write_register(IMU_REG_XA_OFFSET_H, _accel_offset_x >> 7);
    write_register(IMU_REG_XA_OFFSET_L, _accel_offset_x << 1); // Can write 0 to the reserved bits as it just gets overwritten anyway
    write_register(IMU_REG_YA_OFFSET_H, _accel_offset_y >> 7);
    write_register(IMU_REG_YA_OFFSET_L, _accel_offset_y << 1);
    write_register(IMU_REG_ZA_OFFSET_H, _accel_offset_z >> 7);
    write_register(IMU_REG_ZA_OFFSET_L, _accel_offset_z << 1);
}

void IMU::get_gyro_offsets(int16_t &_gyro_offset_x, int16_t &_gyro_offset_y, int16_t &_gyro_offset_z)
{
    uint8_t data[6];

    read_register(IMU_REG_XG_OFFS_USRH, 6, data);

    _gyro_offset_x = data[0] << 8 | data[1];
    _gyro_offset_y = data[2] << 8 | data[3];
    _gyro_offset_z = data[4] << 8 | data[5];
}

void IMU::set_gyro_offsets(int16_t _gyro_offset_x, int16_t _gyro_offset_y, int16_t _gyro_offset_z)
{
    write_register(IMU_REG_XG_OFFS_USRH, _gyro_offset_x >> 8);
    write_register(IMU_REG_XG_OFFS_USRL, _gyro_offset_x);
    write_register(IMU_REG_YG_OFFS_USRH, _gyro_offset_y >> 8);
    write_register(IMU_REG_YG_OFFS_USRL, _gyro_offset_y);
    write_register(IMU_REG_ZG_OFFS_USRH, _gyro_offset_z >> 8);
    write_register(IMU_REG_ZG_OFFS_USRL, _gyro_offset_z);
}