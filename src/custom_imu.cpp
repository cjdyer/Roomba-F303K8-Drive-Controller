#include "custom_imu.h"

const float RAD2DEG = (float) 4068 / 71;

ICM::ICM() : M_CS_PIN(10), m_port(SPI), M_CLOCK(7e6), M_BIT_ORDER(MSBFIRST), M_DATA_MODE(SPI_MODE3) 
{
    pinMode(M_CS_PIN, OUTPUT);
    digitalWrite(M_CS_PIN, HIGH);
}

void ICM::init()
{
    write_register(ICM_REG_PWR_MGMT_1, ICM_BIT_CLK_PLL);
    delay(10);
    
    write_register(ICM_REG_USER_CTRL, ICM_BIT_I2C_IF_DIS | ICM_BIT_I2C_MST_EN);
    write_register(ICM_REG_I2C_MST_CTRL, ICM_I2C_MST_CTRL_CLK_400KHZ);

    set_accel_bandwidth(ICM_ACCEL_BW_6HZ);
    set_accel_fullscale(ICM_ACCEL_FULLSCALE_8G);

    set_gyro_bandwidth(ICM_GYRO_BW_12100HZ);
    set_gyro_fullscale(ICM_GYRO_FULLSCALE_1000DPS);
    
    calibrate_gyro(5.0);
    calibrate_accel(5.0);
}

void ICM::read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps) {
    static int16_t gx, gy, gz;
    
    read_accel_gyro(ax, ay, az, gx, gy, gz);

    gx_rps = (float) gx * m_gyroRes_rad;
    gy_rps = (float) gy * m_gyroRes_rad;
    gz_rps = (float) gz * m_gyroRes_rad;
}

void ICM::write_register(uint16_t addr, uint8_t data) 
{
    static uint8_t regAddr;
    static uint8_t bank;
    
    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);
    
    m_port.beginTransaction(SPISettings(M_CLOCK, (BitOrder)M_BIT_ORDER, M_DATA_MODE));
    
    digitalWrite(M_CS_PIN, LOW);
    m_port.transfer(regAddr);
    m_port.transfer(data);
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();
}

void ICM::read_register(uint16_t addr, uint8_t numBytes, uint8_t *data) {
    static uint8_t regAddr;
    static uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);
    
    m_port.beginTransaction(SPISettings(M_CLOCK, (BitOrder)M_BIT_ORDER, M_DATA_MODE));
    
    digitalWrite(M_CS_PIN, LOW);
    m_port.transfer(regAddr | 0x80);
    m_port.transfer(data, numBytes);
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();
}

void ICM::select_bank(uint8_t bank) {
    static uint8_t previous_bank = 255;
    
    if (bank != previous_bank) {
        m_port.beginTransaction(SPISettings(M_CLOCK, (BitOrder)M_BIT_ORDER, M_DATA_MODE));
        
        digitalWrite(M_CS_PIN, LOW);
        m_port.transfer(ICM_REG_BANK_SEL);
        m_port.transfer(bank << 4);
        digitalWrite(M_CS_PIN, HIGH);
        
        m_port.endTransaction();
        
        previous_bank = bank;
    }
}

void ICM::set_accel_bandwidth(uint8_t accelBw) {
    uint8_t reg;

    read_register(ICM_REG_ACCEL_CONFIG, 1, &reg);
    reg = (ICM_ACCEL_BW_6HZ) | (reg & ~(ICM_MASK_ACCEL_BW));
    write_register(ICM_REG_ACCEL_CONFIG, reg);
}

void ICM::set_gyro_bandwidth(uint8_t gyroBw) {
    uint8_t reg;

    read_register(ICM_REG_GYRO_CONFIG_1, 1, &reg);
    reg = (gyroBw & ICM_MASK_GYRO_BW) | (reg & ~(ICM_MASK_GYRO_BW));
    write_register(ICM_REG_GYRO_CONFIG_1, reg);
}

void ICM::set_accel_fullscale(uint8_t accelFs) {
    uint8_t reg;

    /* Calculate and save accel resolution */
    switch ( accelFs ) {
        case ICM_ACCEL_FULLSCALE_2G:
            m_accelRes = 2.0f / 32768.0f;
            break;
        case ICM_ACCEL_FULLSCALE_4G:
            m_accelRes = 4.0f / 32768.0f;
            break;
        case ICM_ACCEL_FULLSCALE_8G:
            m_accelRes = 8.0f / 32768.0f;
            break;
        case ICM_ACCEL_FULLSCALE_16G:
            m_accelRes = 16.0f / 32768.0f;
            break;
    }
        
    accelFs &= ICM_MASK_ACCEL_FULLSCALE;
    read_register(ICM_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM_MASK_ACCEL_FULLSCALE);
    reg |= accelFs;
    write_register(ICM_REG_ACCEL_CONFIG, reg);
    
    m_g = (int16_t) (1 / m_accelRes + 0.5);
}

void ICM::set_gyro_fullscale(uint8_t gyroFs) {
    uint8_t reg;
    
    switch ( gyroFs ) {
        case ICM_GYRO_FULLSCALE_250DPS:
            m_gyroRes = 250.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM_GYRO_FULLSCALE_500DPS:
            m_gyroRes = 500.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM_GYRO_FULLSCALE_1000DPS:
            m_gyroRes = 1000.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM_GYRO_FULLSCALE_2000DPS:
            m_gyroRes = 2000.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
    }
        
    gyroFs &= ICM_MASK_GYRO_FULLSCALE;
    read_register(ICM_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM_MASK_GYRO_FULLSCALE);
    reg |= gyroFs;
    write_register(ICM_REG_GYRO_CONFIG_1, reg);
}

void ICM::calibrate_gyro(float time_s) 
{
    /* Scale factor to convert gyroscope values into 1000dps full scale */
    float gyro_offset_scale = (m_gyroRes * 32768.0f) / 1000;
    
    /* Gyroscope tolerance in current full scale format */
    int32_t gyro_tolerance = gyro_offset_scale + 0.5;
    
    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int32_t offset_gx, offset_gy, offset_gz;
    
    get_gyro_offsets(offset_gx_1000dps, offset_gy_1000dps, offset_gz_1000dps);
    
    /* Convert offsets to the current gyroscope full scale setting */
    offset_gx = offset_gx_1000dps / gyro_offset_scale + 0.5;
    offset_gy = offset_gy_1000dps / gyro_offset_scale + 0.5;
    offset_gz = offset_gz_1000dps / gyro_offset_scale + 0.5;
    
    static uint16_t step = 0;
    while (1) {
        mean_accel_gyro(time_s, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);
        
        if ((abs(mean_gx) < gyro_tolerance) &&
        (abs(mean_gy) < gyro_tolerance) &&
        (abs(mean_gz) < gyro_tolerance)) {
            break;
        }
        
        offset_gx -= mean_gx;
        offset_gy -= mean_gy;
        offset_gz -= mean_gz;
        
        /* Before writing the offsets to the registers, they need need to be converted to 1000dps gyroscope full scale */
        offset_gx_1000dps = offset_gx * gyro_offset_scale + 0.5;
        offset_gy_1000dps = offset_gy * gyro_offset_scale + 0.5;
        offset_gz_1000dps = offset_gz * gyro_offset_scale + 0.5;
        
        set_gyro_offsets(offset_gx_1000dps, offset_gy_1000dps, offset_gz_1000dps);
        
        step++;
    }
}

void ICM::calibrate_accel(float time_s) {
    /* Scale factor to convert accelerometer values into 32g full scale */
    float accel_offset_scale = (m_accelRes * 32768.0f) / 32;
    /* Accelerometer tolerance in current full scale format */
    int32_t accel_tolerance = 16 * accel_offset_scale + 0.5;
    
    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int32_t offset_ax, offset_ay, offset_az;
    
    get_accel_offsets(offset_ax_32g, offset_ay_32g, offset_az_32g);
    
    /* Convert offsets to the current accelerometer full scale settings */
    offset_ax = offset_ax_32g / accel_offset_scale + 0.5;
    offset_ay = offset_ay_32g / accel_offset_scale + 0.5;
    offset_az = offset_az_32g / accel_offset_scale + 0.5;
      
    static uint16_t step = 0;
    while (1) {
        mean_accel_gyro(time_s, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);
        
        if ((abs(mean_ax) < accel_tolerance) &&
        (abs(mean_ay) < accel_tolerance) &&
        (abs(mean_az - m_g) < accel_tolerance)) {
            break;
        }
        
        offset_ax -= mean_ax;
        offset_ay -= mean_ay;
        offset_az -= mean_az - m_g;
        
        /* Before writing the offsets to the registers, they need need to be converted to 32g accelerometer full scale */
        offset_ax_32g = offset_ax * accel_offset_scale + 0.5;
        offset_ay_32g = offset_ay * accel_offset_scale + 0.5;
        offset_az_32g = offset_az * accel_offset_scale + 0.5;
        
        set_accel_offsets(offset_ax_32g, offset_ay_32g, offset_az_32g);
        
        step++;
    }
}

void ICM::get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz) {
    static uint8_t data[6];
    
    /* Read x raw data registers into a data array */
    read_register(ICM_REG_XG_OFFS_USRH, 2, &data[0]);
    /* Read y raw data registers into a data array */
    read_register(ICM_REG_YG_OFFS_USRH, 2, &data[2]);
    /* Read z raw data registers into a data array */
    read_register(ICM_REG_ZG_OFFS_USRH, 2, &data[4]);
        
    /* Convert the MSB and LSB into a signed 16-bit value */
    offset_gx = ((uint16_t) data[0] << 8) | data[1];
    offset_gy = ((uint16_t) data[2] << 8) | data[3];
    offset_gz = ((uint16_t) data[4] << 8) | data[5];
}

void ICM::set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz) {
    /* Write x offset to registers */
    write_register(ICM_REG_XG_OFFS_USRH, (uint8_t) (offset_gx >> 8));
    write_register(ICM_REG_XG_OFFS_USRL, (uint8_t) (offset_gx & 0xFF));
    /* Write y offset to registers */
    write_register(ICM_REG_YG_OFFS_USRH, (uint8_t) (offset_gy >> 8));
    write_register(ICM_REG_YG_OFFS_USRL, (uint8_t) (offset_gy & 0xFF));
    /* Write z offset to registers */
    write_register(ICM_REG_ZG_OFFS_USRH, (uint8_t) (offset_gz >> 8));
    write_register(ICM_REG_ZG_OFFS_USRL, (uint8_t) (offset_gz & 0xFF));
}

void ICM::mean_accel_gyro(float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz) {
    int16_t gx, gy, gz, ax, ay, az;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;
    
    int32_t num = 0;
    uint32_t t_start = micros();
    const uint32_t time = (time_s * 1e6) + 0.5;
    
    while ((micros() - t_start) < time) {
        /* read imu measurements */
        read_accel_gyro(ax, ay, az, gx, gy, gz);
        
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        ++num;
    }

    mean_ax = sum_ax / num;
    mean_ay = sum_ay / num;
    mean_az = sum_az / num;
    mean_gx = sum_gx / num;
    mean_gy = sum_gy / num;
    mean_gz = sum_gz / num;
}

void ICM::read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    static uint8_t data[12];
    
    /* Read accelerometer and gyroscope raw data into a data array */
    read_register(ICM_REG_ACCEL_XOUT_H_SH, 12, data);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    ax = ((int16_t) data[0] << 8) | data[1];
    ay = ((int16_t) data[2] << 8) | data[3];
    az = ((int16_t) data[4] << 8) | data[5];
    gx = ((int16_t) data[6] << 8) | data[7];
    gy = ((int16_t) data[8] << 8) | data[9];
    gz = ((int16_t) data[10] << 8) | data[11];
}

void ICM::get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az) {
    static uint8_t data[6];
    
    /* Read x offset registers into a data array */
    read_register(ICM_REG_XA_OFFSET_H, 2, &data[0]);
    /* Read y offset registers into a data array */
    read_register(ICM_REG_YA_OFFSET_H, 2, &data[2]);
    /* Read z offset registers into a data array */
    read_register(ICM_REG_ZA_OFFSET_H, 2, &data[4]);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    offset_ax = ((uint16_t) data[0] << 7) | (data[1] >> 1);
    offset_ay = ((uint16_t) data[2] << 7) | (data[3] >> 1);
    offset_az = ((uint16_t) data[4] << 7) | (data[5] >> 1);
}

void ICM::set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az) {
    static uint8_t data[3];

    /* Read x LSB offset register into a data array */
    read_register(ICM_REG_XA_OFFSET_L, 1, &data[0]);
    /* Read y LSB offset register into a data array */
    read_register(ICM_REG_YA_OFFSET_L, 1, &data[1]);
    /* Read z LSB offset register into a data array */
    read_register(ICM_REG_ZA_OFFSET_L, 1, &data[2]);
    
    /* Write x offset to registers */
    write_register(ICM_REG_XA_OFFSET_H, (uint8_t) ((offset_ax >> 7) & 0xFF));
    write_register(ICM_REG_XA_OFFSET_L, (uint8_t) (((offset_ax << 1) & 0xFF) | (data[0] & 0x01)));
    /* Write y offset to registers */
    write_register(ICM_REG_YA_OFFSET_H, (uint8_t) ((offset_ay >> 7) & 0xFF));
    write_register(ICM_REG_YA_OFFSET_L, (uint8_t) (((offset_ay << 1) & 0xFF) | (data[1] & 0x01)));
    /* Write z offset to registers */
    write_register(ICM_REG_ZA_OFFSET_H, (uint8_t) ((offset_az >> 7) & 0xFF));
    write_register(ICM_REG_ZA_OFFSET_L, (uint8_t) (((offset_az << 1) & 0xFF) | (data[2] & 0x01)));
}