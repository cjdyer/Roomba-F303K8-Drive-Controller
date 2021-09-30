#include "custom_imu.h"
#include "alt_IMU_REGS.h"



ICM::ICM() : m_port(SPI), spi_settings(SPISettings(7000000, MSBFIRST, SPI_MODE3))
{
    pinMode(M_CS_PIN, OUTPUT);
    digitalWrite(M_CS_PIN, HIGH);
}

void ICM::init()
{
    uint8_t data[1];
    write_register(ICM_REG_PWR_MGMT_1, ICM_BIT_CLK_PLL | ICM_BIT_TEMP_DIS); // Enable clock, disable sleep and disable temp sensor
    delay(30); // Accelerometer start time
    write_register(ICM_REG_USER_CTRL, ICM_BIT_I2C_IF_DIS); // Enable SPI - Disable I2C
    write_register(ICM_REG_ACCEL_CONFIG, ICM_ACCEL_FULLSCALE_16G | ICM_ACCEL_ENABLE_DLPF | ICM_ACCEL_BW_6HZ); // Configure Accel
    write_register(ICM_REG_GYRO_CONFIG_1, ICM_GYRO_BW_12100HZ | ICM_GYRO_FULLSCALE_1000DPS); // Configure Gyro

    write_mag_register(AK09916_REG_CONTROL_2, AK09916_MODE_100HZ);
    read_mag_register(AK09916_REG_STATUS_2, 1, data);

    // high time = more risk of over calibration  
    // low time = more risk of random correlation and worse drift
    calibrate(0.5); // Open to change in this time  
    calibrate_mag(5.0);
}

void ICM::read_accel_gyro_rps(int16_t &_accel_x, int16_t &_accel_y, int16_t &_accel_z, float &_gyro_rps_x, float &_gyro_rps_y, float &_gyro_rps_z) 
{
    static int16_t _gyro_x, _gyro_y, _gyro_z;
    
    read_accel_gyro(_accel_x, _accel_y, _accel_z, _gyro_x, _gyro_y, _gyro_z);

    _gyro_rps_x = (float) _gyro_x * m_gyroRes_rad;
    _gyro_rps_y = (float) _gyro_y * m_gyroRes_rad;
    _gyro_rps_z = (float) _gyro_z * m_gyroRes_rad;
}

bool ICM::read_mag_ut(float &mag_x_ut, float &mag_y_ut, float &mag_z_ut) {
    static int16_t mx, my, mz;
    static bool new_mag;
    
    new_mag = read_mag(mx, my, mz);
    
    /* Multiply the values with the resolution to transform them into uT */
    mag_x_ut = (float) mx * m_magRes;
    mag_y_ut = (float) my * m_magRes;
    mag_z_ut = (float) mz * m_magRes;

    return new_mag;
}

void ICM::write_register(const uint16_t _address, const uint8_t _data) 
{
    select_bank(_address >> 7);
    
    m_port.beginTransaction(spi_settings);
    
    digitalWrite(M_CS_PIN, LOW);
    m_port.transfer(_address & ICM_REG_BANK_SEL); // 127 (0x7F) all but the select REG. Address x7F
    m_port.transfer(_data);
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();
}

void ICM::read_register(const uint16_t _address, const uint8_t _number_bytes, uint8_t *_data) 
{
    select_bank(_address >> 7);
    
    m_port.beginTransaction(spi_settings);
    
    digitalWrite(M_CS_PIN, LOW);
    m_port.transfer(_address | ~ICM_REG_BANK_SEL);
    m_port.transfer(_data, _number_bytes);
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();
}

void ICM::select_bank(const uint8_t _bank) 
{
    static uint8_t previous_bank = 10;

    if (_bank != previous_bank) {
        m_port.beginTransaction(spi_settings);
        
        digitalWrite(M_CS_PIN, LOW);
        m_port.transfer(ICM_REG_BANK_SEL);
        m_port.transfer(_bank << 4); // First 4 bits are reserved [5:4]
        digitalWrite(M_CS_PIN, HIGH);
        
        m_port.endTransaction();
        
        previous_bank = _bank;
    }
}

void ICM::calibrate(const float _time) // Might rewite this
{
    constexpr float accel_offset_scale = 0.5f; // 16g / 32g = 0.25
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
            accel_calibrated = (abs(mean_ax) < accel_tolerance) && (abs(mean_ay) < accel_tolerance) && (abs(mean_az - m_g) < accel_tolerance);

            offset_ax -= mean_ax;
            offset_ay -= mean_ay;
            offset_az -= mean_az - m_g;

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

bool ICM::calibrate_mag(float time_s, int32_t mag_minimumRange, float &offset_mx, float &offset_my, float &offset_mz, float &scale_mx, float &scale_my, float &scale_mz) {
    int16_t min_mx, max_mx, min_my, max_my, min_mz, max_mz;
    int32_t sum_mx, sum_my, sum_mz;
    int32_t dif_mx, dif_my, dif_mz, dif_m;

    m_offset_mx = 0; m_offset_my = 0; m_offset_mz = 0;
    m_scale_mx = 1; m_scale_my = 1; m_scale_mz = 1;
    
    min_max_mag(time_s, mag_minimumRange, min_mx, max_mx, min_my, max_my, min_mz, max_mz);
    
    sum_mx = (int32_t) max_mx + min_mx;
    sum_my = (int32_t) max_my + min_my;
    sum_mz = (int32_t) max_mz + min_mz;
    
    dif_mx = (int32_t) max_mx - min_mx;
    dif_my = (int32_t) max_my - min_my;
    dif_mz = (int32_t) max_mz - min_mz;
    
    offset_mx = -0.5f * sum_mx;
    offset_my = -0.5f * sum_my;
    offset_mz = -0.5f * sum_mz;
    
    dif_m = (dif_mx + dif_my + dif_mz) / 3;
    
    scale_mx = (float) dif_m / dif_mx;
    scale_my = (float) dif_m / dif_my;
    scale_mz = (float) dif_m / dif_mz;
    
    /* Apply calibration result. */
    m_offset_mx = offset_mx; m_offset_my = offset_my; m_offset_mz = offset_mz;
    m_scale_mx = scale_mx; m_scale_my = scale_my; m_scale_mz = scale_mz;
    
    return true;
}   

void ICM::min_max_mag(float time_s, int32_t mag_minimumRange, int16_t &min_mx, int16_t &max_mx, int16_t &min_my, int16_t &max_my, int16_t &min_mz, int16_t &max_mz) {
    int16_t mx, my, mz;
    
    min_mx = 32767; min_my = 32767; min_mz = 32767;
    max_mx = -32768; max_my = -32768; max_mz = -32768;
    
    bool new_mag;
    
    const uint32_t time = time_s * 1e6;
    
    bool miniumRange_mx = false;
    bool miniumRange_my = false;
    bool miniumRange_mz = false;
    
    while (!miniumRange_mx || !miniumRange_my || !miniumRange_mz) 
    {
        uint32_t t_start = micros();
        while ((micros() - t_start) < time)  
        {
            new_mag = read_mag(mx, my, mz);
            
            if (new_mag) {
                if (mx < min_mx)
                    min_mx = mx;      
                else if (mx > max_mx)
                    max_mx = mx;
                
                if (my < min_my)
                    min_my = my;
                else if (my > max_my)
                    max_my = my;
                
                if (mz < min_mz) 
                    min_mz = mz;
                else if (mz > max_mz)
                    max_mz = mz;
                
                miniumRange_mx = (max_mx - min_mx) > mag_minimumRange;
                miniumRange_my = (max_my - min_my) > mag_minimumRange;
                miniumRange_mz = (max_mz - min_mz) > mag_minimumRange;
            }
        }
    }
}

void ICM::mean_accel_gyro(const float _calibration_time, int16_t &_mean_accel_x, int16_t &_mean_accel_y, int16_t &_mean_accel_z, int16_t &_mean_gyro_x, int16_t &_mean_gyro_y, int16_t &_mean_gyro_z) 
{
    static int32_t sum_accel_x, sum_accel_y, sum_accel_z, sum_gyro_x, sum_gyro_y, sum_gyro_z;
    static int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    static int32_t iterations;
    
    sum_accel_x = sum_accel_y = sum_accel_z = sum_gyro_x = sum_gyro_y = sum_gyro_z = 0;
    iterations = 0;

    uint32_t start_time = micros();
    const uint32_t calibration_time_us = _calibration_time * 1e6; // Seconds to micros
    
    while ((micros() - start_time) < calibration_time_us) {
        read_accel_gyro(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        
        sum_accel_x += accel_x; sum_accel_y += accel_y; sum_accel_z += accel_z;
        sum_gyro_x  +=  gyro_x; sum_gyro_y  +=  gyro_y; sum_gyro_z  +=  gyro_z;

        iterations++;
    }

    _mean_accel_x = sum_accel_x / iterations;
    _mean_accel_y = sum_accel_y / iterations;
    _mean_accel_z = sum_accel_z / iterations;
    _mean_gyro_x  = sum_gyro_x  / iterations;
    _mean_gyro_y  = sum_gyro_y  / iterations;
    _mean_gyro_z  = sum_gyro_z  / iterations;
}

void ICM::read_accel_gyro(int16_t &_accel_x, int16_t &_accel_y, int16_t &_accel_z, int16_t &_gyro_x, int16_t &_gyro_y, int16_t &_gyro_z) 
{
    static uint8_t data[12]; // 12 addresses, 2 per Gryo/Accel axis [MSB, LSB] 
    
    read_register(ICM_REG_ACCEL_XOUT_H_SH, 12, data); // Read from 0x2E to 0x39(0x2E + 11)
    
    _accel_x = data[0]  << 8 | data[1]; // Convert 2 Bytes to 16 bit signed int [MSB << 8 | LSB]
    _accel_y = data[2]  << 8 | data[3];
    _accel_z = data[4]  << 8 | data[5];
    _gyro_x =  data[6]  << 8 | data[7];
    _gyro_y =  data[8]  << 8 | data[9];
    _gyro_z =  data[10] << 8 | data[11];
}

bool ICM::read_mag(int16_t &mx, int16_t &my, int16_t &mz) {
    static uint8_t status = 0;
    static uint32_t t_start;
    static uint8_t data[6];
    
    switch (status) {
        case 0:     /* Request AK09916C status_1 */
            /* Set ICM20948 SLV0_REG to AK09916C status_1 address */
            write_register(ICM_REG_I2C_SLV0_REG, AK09916_REG_STATUS_1);
            /* Request AK09916C status_1 */
            write_register(ICM_REG_I2C_SLV0_CTRL, ICM_BIT_I2C_SLV_EN | 1);
        
            t_start = micros();
            status = 1;
            break;
        case 1:     /* Read AK09916C status_1 from ICM20948 to check if data is ready */
            /* Wait for ICM20948 registers to fill with AK09916C status_1 data */
            if ((micros() - t_start) > 1000) {
                /* Read AK09916C status_1 from ICM20948 EXT_SLV_SENS_DATA registers */
                read_register(ICM_REG_EXT_SLV_SENS_DATA_00, 1, data);
                
                /* Check AK09916C status_1 for data ready */
                if ((data[0] & AK09916_BIT_DRDY) == AK09916_BIT_DRDY) {
                    t_start = micros();
                    status = 2;
                }
            }
            break;
        case 2:     /* Request AK09916C measurement data */
            /* Set ICM20948 SLV0_REG to AK09916C measurement data address */
            write_register(ICM_REG_I2C_SLV0_REG, AK09916_REG_HXL);
            /* Request AK09916C measurement data */
            write_register(ICM_REG_I2C_SLV0_CTRL, ICM_BIT_I2C_SLV_EN | 6);
            
            t_start = micros();
            status = 3;
            break;
        case 3:     /* Read AK09916C measurement data from ICM20948 */
            /* Wait for ICM20948 registers to fill with AK09916C measurement data */
            if ((micros() - t_start) > 1000) {
                /* Read AK09916C measurement data from ICM20948 EXT_SLV_SENS_DATA registers */
                read_register(ICM_REG_EXT_SLV_SENS_DATA_00, 6, data);
                
                /* Convert the LSB and MSB into a signed 16-bit value */
                mx = ((int16_t) data[1] << 8) | data[0];
                my = ((int16_t) data[3] << 8) | data[2];
                mz = ((int16_t) data[5] << 8) | data[4];
                
                /* Transform magnetometer values to match the coordinate system of accelerometer and gyroscope */
                my = -my;
                mz = -mz;                
                
                /* Apply hard and soft iron distortion correction */
                mx = ((mx + m_offset_mx) * m_scale_mx) + 0.5;
                my = ((my + m_offset_my) * m_scale_my) + 0.5;
                mz = ((mz + m_offset_mz) * m_scale_mz) + 0.5;
                
                status = 4;
                return true;
            }
            break;
        case 4:    /* Request AK09916C status_2 to indicate that data is read and allow AK09916C to update the measurement data */
            /* Set ICM20948 SLV0_REG to AK09916C status_2 address */
            write_register(ICM_REG_I2C_SLV0_REG, AK09916_REG_STATUS_2);
            /* Request AK09916C status_1 */
            write_register(ICM_REG_I2C_SLV0_CTRL, ICM_BIT_I2C_SLV_EN | 1);
            
            t_start = micros();
            status = 5;
            break;
        case 5:     /* Wait for AK09916C status_2 request from ICM20948 */
            if ((micros() - t_start) > 1000) {
                status = 0;
            }
            break;           
        default:
            status = 0;
            break;
    }
    
    return false;
}


void ICM::set_mag_transfer(bool read) 
{
    static const uint8_t MAG_BIT_READ   = AK09916_BIT_I2C_SLV_ADDR | ICM_BIT_I2C_SLV_READ;
    static const uint8_t MAG_BIT_WRITE  = AK09916_BIT_I2C_SLV_ADDR;

    static bool read_old = !read;

    if (read != read_old) {   
        if (read) {
            write_register(ICM_REG_I2C_SLV0_ADDR, MAG_BIT_READ);
        }
        else {
            write_register(ICM_REG_I2C_SLV0_ADDR, MAG_BIT_WRITE);
        }
        read_old = read;
    }
}

void ICM::read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data) 
{
    set_mag_transfer(true);

    write_register(ICM_REG_I2C_SLV0_REG, addr);
    write_register(ICM_REG_I2C_SLV0_CTRL, ICM_BIT_I2C_SLV_EN | numBytes);
    delay(10);
    read_register(ICM_REG_EXT_SLV_SENS_DATA_00, numBytes, data); 
}

void ICM::write_mag_register(uint8_t addr, uint8_t data) 
{
    set_mag_transfer(false);

    write_register(ICM_REG_I2C_SLV0_REG, addr);
    write_register(ICM_REG_I2C_SLV0_DO, data);
    write_register(ICM_REG_I2C_SLV0_CTRL, ICM_BIT_I2C_SLV_EN | 0x01);
    delay(10);

}
void ICM::get_accel_offsets(int16_t &_accel_offset_x, int16_t &_accel_offset_y, int16_t &_accel_offset_z) 
{
    uint8_t data[8]; // 8 addresses, 3 per Accel axis [MSB, LSB,  blank] (9th doesnt need to be read)
    
    read_register(ICM_REG_XA_OFFSET_H, 8, data); // Read from 0x14 to 0x1B(0x14 + 7)
   
    _accel_offset_x = data[0] << 7 | data[1] >> 1; // Convert 2 Bytes to 16 bit signed int [MSB << 7 | LSB >> 1]
    _accel_offset_y = data[3] << 7 | data[4] >> 1; // Offset low byte is 7 bits not 8. With the last bit being reserved
    _accel_offset_z = data[6] << 7 | data[7] >> 1;
}

void ICM::set_accel_offsets(int16_t _accel_offset_x, int16_t _accel_offset_y, int16_t _accel_offset_z) 
{
    write_register(ICM_REG_XA_OFFSET_H, _accel_offset_x >> 7);
    write_register(ICM_REG_XA_OFFSET_L, _accel_offset_x << 1); // Can write 0 to the reserved bits as it just gets overwritten anyway
    write_register(ICM_REG_YA_OFFSET_H, _accel_offset_y >> 7);
    write_register(ICM_REG_YA_OFFSET_L, _accel_offset_y << 1); 
    write_register(ICM_REG_ZA_OFFSET_H, _accel_offset_z >> 7);
    write_register(ICM_REG_ZA_OFFSET_L, _accel_offset_z << 1);
}

void ICM::get_gyro_offsets(int16_t &_gyro_offset_x, int16_t &_gyro_offset_y, int16_t &_gyro_offset_z) 
{
    uint8_t data[6];
    
    read_register(ICM_REG_XG_OFFS_USRH, 6, data);
        
    _gyro_offset_x = data[0] << 8 | data[1];
    _gyro_offset_y = data[2] << 8 | data[3];
    _gyro_offset_z = data[4] << 8 | data[5];
}

void ICM::set_gyro_offsets(int16_t _gyro_offset_x, int16_t _gyro_offset_y, int16_t _gyro_offset_z) 
{
    write_register(ICM_REG_XG_OFFS_USRH, _gyro_offset_x >> 8);
    write_register(ICM_REG_XG_OFFS_USRL, _gyro_offset_x);
    write_register(ICM_REG_YG_OFFS_USRH, _gyro_offset_y >> 8);
    write_register(ICM_REG_YG_OFFS_USRL, _gyro_offset_y);
    write_register(ICM_REG_ZG_OFFS_USRH, _gyro_offset_z >> 8);
    write_register(ICM_REG_ZG_OFFS_USRL, _gyro_offset_z);
}