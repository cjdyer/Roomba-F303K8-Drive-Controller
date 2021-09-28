#include "custom_imu.h"

const float RAD2DEG = (float) 4068 / 71;

ICM::ICM() : M_CS_PIN(10), m_port(SPI), M_CLOCK(7e6), M_BIT_ORDER(MSBFIRST), M_DATA_MODE(SPI_MODE3) 
{
    pinMode(M_CS_PIN, OUTPUT);
    digitalWrite(M_CS_PIN, HIGH);
}

bool ICM::init()
{
    uint8_t data[1];
    
    /* Reset ICM20948 */
    //reset();
    
    /* Auto select best available clock source PLL if ready, else use internal oscillator */
    write_register(ICM20948_REG_PWR_MGMT_1, ICM20948_BIT_CLK_PLL);
     
    /* PLL startup time - no spec in data sheet */
    delay(10);
    
    /* Reset I2C Slave module and use SPI */
    /* Enable I2C Master I/F module */
    write_register(ICM20948_REG_USER_CTRL, ICM20948_BIT_I2C_IF_DIS | ICM20948_BIT_I2C_MST_EN);
    
    /* Set I2C Master clock frequency */
    write_register(ICM20948_REG_I2C_MST_CTRL, ICM20948_I2C_MST_CTRL_CLK_400KHZ);
    
    /* Read ICM20948 "Who am I" register */
    read_register(ICM20948_REG_WHO_AM_I, 1, data);
    
    /* Check if "Who am I" register was successfully read */
    if (data[0] != ICM20948_DEVICE_ID) {
        return false;
    }

    /* Configure accelerometer */
    set_accel_bandwidth(ICM20948_ACCEL_BW_6HZ);
    set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_8G);
    //set_accel_sample_rate_div(...);    /* the accelerometer sample rate is 4500 Hz for ICM20948_ACCEL_BW_1210HZ */
    
    /* Configure gyroscope */
    set_gyro_bandwidth(ICM20948_GYRO_BW_12100HZ);
    set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_1000DPS);
    //set_gyro_sample_rate_div(...);    /* the gyroscope sample rate is 9000 Hz for ICM20948_GYRO_BW_12100HZ */
    
    /* Apply calibration data */
    calibrate_gyro(5.0);
    calibrate_accel(5.0);
    Serial.println("CaliM succ");
    
    return true;
}

void ICM::read_accel_gyro_rps(int16_t &ax, int16_t &ay, int16_t &az, float &gx_rps, float &gy_rps, float &gz_rps) {
    static int16_t gx, gy, gz;
    
    read_accel_gyro(ax, ay, az, gx, gy, gz);
    
    /* Multiply the gyroscope values with their resolution to transform them into rad/s */
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
    
    /* Enable chip select */
    digitalWrite(M_CS_PIN, LOW);
    
    /* Transfer address without read-bit set */
    m_port.transfer(regAddr);
    /* Send data byte */
    m_port.transfer(data);
    
    /* Disable chip select */
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();

    return;
}

void ICM::read_register(uint16_t addr, uint8_t numBytes, uint8_t *data) {
    static uint8_t regAddr;
    static uint8_t bank;

    regAddr = (uint8_t) (addr & 0x7F);
    bank = (uint8_t) (addr >> 7);

    select_bank(bank);
    
    m_port.beginTransaction(SPISettings(M_CLOCK, (BitOrder)M_BIT_ORDER, M_DATA_MODE));
    
    /* Enable chip select */
    digitalWrite(M_CS_PIN, LOW);
    
    /* Transfer address with read-bit set */
    m_port.transfer(regAddr | 0x80);
    /* Receive data array */
    m_port.transfer(data, numBytes);

    /* Disable chip select */
    digitalWrite(M_CS_PIN, HIGH);
    
    m_port.endTransaction();

    return;
}

void ICM::select_bank(uint8_t bank) {
    static uint8_t bank_old = 255;
    
    /* Select bank if it has changed */
    if (bank != bank_old) {
        m_port.beginTransaction(SPISettings(M_CLOCK, (BitOrder)M_BIT_ORDER, M_DATA_MODE));
        
        /* Enable chip select */
        digitalWrite(M_CS_PIN, LOW);
        
        /* Transfer address without read-bit set */
        m_port.transfer(ICM20948_REG_BANK_SEL);
        /* Send data byte */
        m_port.transfer(bank << 4);
        
        /* Disable chip select */
        digitalWrite(M_CS_PIN, HIGH);
        
        m_port.endTransaction();
        
        bank_old = bank;
    }
    return;
}

uint32_t ICM::set_accel_bandwidth(uint8_t accelBw) {
    uint8_t reg;

    /* Read GYRO_CONFIG_1 register */
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20948_MASK_ACCEL_BW);

    /* Write new bandwidth value to gyro config register */
    reg |= (accelBw & ICM20948_MASK_ACCEL_BW);
    write_register(ICM20948_REG_ACCEL_CONFIG, reg);

    return OK;
}

uint32_t ICM::set_gyro_bandwidth(uint8_t gyroBw) {
    uint8_t reg;

    /* Read GYRO_CONFIG_1 register */
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20948_MASK_GYRO_BW);

    /* Write new bandwidth value to gyro config register */
    reg |= (gyroBw & ICM20948_MASK_GYRO_BW);
    write_register(ICM20948_REG_GYRO_CONFIG_1, reg);

    return OK;
}

uint32_t ICM::set_accel_fullscale(uint8_t accelFs) {
    uint8_t reg;

    /* Calculate and save accel resolution */
    switch ( accelFs ) {
        case ICM20948_ACCEL_FULLSCALE_2G:
            m_accelRes = 2.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_4G:
            m_accelRes = 4.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_8G:
            m_accelRes = 8.0f / 32768.0f;
            break;
        case ICM20948_ACCEL_FULLSCALE_16G:
            m_accelRes = 16.0f / 32768.0f;
            break;
        default:
            return ERROR;
    }
        
    accelFs &= ICM20948_MASK_ACCEL_FULLSCALE;
    read_register(ICM20948_REG_ACCEL_CONFIG, 1, &reg);
    reg &= ~(ICM20948_MASK_ACCEL_FULLSCALE);
    reg |= accelFs;
    write_register(ICM20948_REG_ACCEL_CONFIG, reg);
    
    /*  Acceleration of gravity in LSB  */
    m_g = (int16_t) (1 / m_accelRes + 0.5);

    return OK;
}

uint32_t ICM::set_gyro_fullscale(uint8_t gyroFs) {
    uint8_t reg;
    
    /* Calculate and save gyro resolution */
    switch ( gyroFs ) {
        case ICM20948_GYRO_FULLSCALE_250DPS:
            m_gyroRes = 250.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM20948_GYRO_FULLSCALE_500DPS:
            m_gyroRes = 500.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM20948_GYRO_FULLSCALE_1000DPS:
            m_gyroRes = 1000.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        case ICM20948_GYRO_FULLSCALE_2000DPS:
            m_gyroRes = 2000.0f / 32768.0f;
            m_gyroRes_rad = m_gyroRes / RAD2DEG;
            break;
        default:
            return ERROR;
    }
        
    gyroFs &= ICM20948_MASK_GYRO_FULLSCALE;
    read_register(ICM20948_REG_GYRO_CONFIG_1, 1, &reg);
    reg &= ~(ICM20948_MASK_GYRO_FULLSCALE);
    reg |= gyroFs;
    write_register(ICM20948_REG_GYRO_CONFIG_1, reg);

    return OK;
}

void ICM::set_mag_transfer(bool read) {
    static const uint8_t MAG_BIT_READ   = AK09916_BIT_I2C_SLV_ADDR | ICM20948_BIT_I2C_SLV_READ;
    static const uint8_t MAG_BIT_WRITE  = AK09916_BIT_I2C_SLV_ADDR;
    
    static bool read_old = !read;
    
    /* Set transfer mode if it has changed */
    if (read != read_old) {   
        if (read) {
            write_register(ICM20948_REG_I2C_SLV0_ADDR, MAG_BIT_READ);
        }
        else {
            write_register(ICM20948_REG_I2C_SLV0_ADDR, MAG_BIT_WRITE);
        }
        read_old = read;
    }
    return;
}

void ICM::read_mag_register(uint8_t addr, uint8_t numBytes, uint8_t *data) {
    /* Set transfer mode to read */
    set_mag_transfer(true);
    
    /* Set SLV0_REG to magnetometer register address */
    write_register(ICM20948_REG_I2C_SLV0_REG, addr);
    
    /* Request bytes */
    write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | numBytes);
    
    /* Wait some time for registers to fill */
    delay(10);
    
    /* Read bytes from the ICM20948 EXT_SLV_SENS_DATA registers */
    read_register(ICM20948_REG_EXT_SLV_SENS_DATA_00, numBytes, data); 
    
    return;
}

void ICM::write_mag_register(uint8_t addr, uint8_t data) {
    /* Set transfer mode to write */
    set_mag_transfer(false);
    
    /* Set SLV0_REG to magnetometer register address */
    write_register(ICM20948_REG_I2C_SLV0_REG, addr);
    
    /* Store data to write inside SLV0_DO */
    write_register(ICM20948_REG_I2C_SLV0_DO, data);
    
    /* Send one byte */
    write_register(ICM20948_REG_I2C_SLV0_CTRL, ICM20948_BIT_I2C_SLV_EN | 0x01);
    
    /* Wait some time for registers to fill */
    delay(10);
    
    return;
}

uint32_t ICM::set_mag_mode(uint8_t magMode){
    switch ( magMode ) {
        case AK09916_BIT_MODE_POWER_DOWN:
            break;
        case AK09916_MODE_SINGLE:
            break;
        case AK09916_MODE_10HZ:
            break;
        case AK09916_MODE_20HZ:
            break;
        case AK09916_MODE_50HZ:
            break;
        case AK09916_MODE_100HZ:
            break;
        case AK09916_MODE_ST:
            break;
        default:
            return ERROR;
    }
    
    write_mag_register(AK09916_REG_CONTROL_2, magMode);
    
    return OK;
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

uint32_t ICM::get_gyro_offsets(int16_t &offset_gx, int16_t &offset_gy, int16_t &offset_gz) {
    static uint8_t data[6];
    
    /* Read x raw data registers into a data array */
    read_register(ICM20948_REG_XG_OFFS_USRH, 2, &data[0]);
    /* Read y raw data registers into a data array */
    read_register(ICM20948_REG_YG_OFFS_USRH, 2, &data[2]);
    /* Read z raw data registers into a data array */
    read_register(ICM20948_REG_ZG_OFFS_USRH, 2, &data[4]);
        
    /* Convert the MSB and LSB into a signed 16-bit value */
    offset_gx = ((uint16_t) data[0] << 8) | data[1];
    offset_gy = ((uint16_t) data[2] << 8) | data[3];
    offset_gz = ((uint16_t) data[4] << 8) | data[5];
    
    return OK;
}

uint32_t ICM::set_gyro_offsets(int16_t offset_gx, int16_t offset_gy, int16_t offset_gz) {
    /* Write x offset to registers */
    write_register(ICM20948_REG_XG_OFFS_USRH, (uint8_t) (offset_gx >> 8));
    write_register(ICM20948_REG_XG_OFFS_USRL, (uint8_t) (offset_gx & 0xFF));
    /* Write y offset to registers */
    write_register(ICM20948_REG_YG_OFFS_USRH, (uint8_t) (offset_gy >> 8));
    write_register(ICM20948_REG_YG_OFFS_USRL, (uint8_t) (offset_gy & 0xFF));
    /* Write z offset to registers */
    write_register(ICM20948_REG_ZG_OFFS_USRH, (uint8_t) (offset_gz >> 8));
    write_register(ICM20948_REG_ZG_OFFS_USRL, (uint8_t) (offset_gz & 0xFF));
    
    return OK;
}

uint32_t ICM::mean_accel_gyro(float time_s, int16_t &mean_ax, int16_t &mean_ay, int16_t &mean_az, int16_t &mean_gx, int16_t &mean_gy, int16_t &mean_gz) {
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
    
    return OK;
}

bool ICM::read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    static uint8_t data[12];
    
    /* Read accelerometer and gyroscope raw data into a data array */
    read_register(ICM20948_REG_ACCEL_XOUT_H_SH, 12, data);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    ax = ((int16_t) data[0] << 8) | data[1];
    ay = ((int16_t) data[2] << 8) | data[3];
    az = ((int16_t) data[4] << 8) | data[5];
    gx = ((int16_t) data[6] << 8) | data[7];
    gy = ((int16_t) data[8] << 8) | data[9];
    gz = ((int16_t) data[10] << 8) | data[11];
    
    return true;
}

uint32_t ICM::get_accel_offsets(int16_t &offset_ax, int16_t &offset_ay, int16_t &offset_az) {
    static uint8_t data[6];
    
    /* Read x offset registers into a data array */
    read_register(ICM20948_REG_XA_OFFSET_H, 2, &data[0]);
    /* Read y offset registers into a data array */
    read_register(ICM20948_REG_YA_OFFSET_H, 2, &data[2]);
    /* Read z offset registers into a data array */
    read_register(ICM20948_REG_ZA_OFFSET_H, 2, &data[4]);
    
    /* Convert the MSB and LSB into a signed 16-bit value */
    offset_ax = ((uint16_t) data[0] << 7) | (data[1] >> 1);
    offset_ay = ((uint16_t) data[2] << 7) | (data[3] >> 1);
    offset_az = ((uint16_t) data[4] << 7) | (data[5] >> 1);
    
    return OK;
}

uint32_t ICM::set_accel_offsets(int16_t offset_ax, int16_t offset_ay, int16_t offset_az) {
    static uint8_t data[3];
    
    /* Bit 0 of the LSB offset register must be preserved, since it is used for temperature compensation calculations (? the data sheet is not clear). */
    
    /* Read x LSB offset register into a data array */
    read_register(ICM20948_REG_XA_OFFSET_L, 1, &data[0]);
    /* Read y LSB offset register into a data array */
    read_register(ICM20948_REG_YA_OFFSET_L, 1, &data[1]);
    /* Read z LSB offset register into a data array */
    read_register(ICM20948_REG_ZA_OFFSET_L, 1, &data[2]);
    
    /* Write x offset to registers */
    write_register(ICM20948_REG_XA_OFFSET_H, (uint8_t) ((offset_ax >> 7) & 0xFF));
    write_register(ICM20948_REG_XA_OFFSET_L, (uint8_t) (((offset_ax << 1) & 0xFF) | (data[0] & 0x01)));
    /* Write y offset to registers */
    write_register(ICM20948_REG_YA_OFFSET_H, (uint8_t) ((offset_ay >> 7) & 0xFF));
    write_register(ICM20948_REG_YA_OFFSET_L, (uint8_t) (((offset_ay << 1) & 0xFF) | (data[1] & 0x01)));
    /* Write z offset to registers */
    write_register(ICM20948_REG_ZA_OFFSET_H, (uint8_t) ((offset_az >> 7) & 0xFF));
    write_register(ICM20948_REG_ZA_OFFSET_L, (uint8_t) (((offset_az << 1) & 0xFF) | (data[2] & 0x01)));
    
    return OK;
}