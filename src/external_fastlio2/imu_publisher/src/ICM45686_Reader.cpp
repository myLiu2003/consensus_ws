#include "ICM45686_Reader.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cmath>

ICM45686_Reader::ICM45686_Reader(uint8_t device_address, const std::string& i2c_bus)
    : device_addr_(device_address), i2c_bus_(i2c_bus), i2c_file_(-1),
    accel_sensitivity_(8192.0f), gyro_sensitivity_(65.5f) {
}
//2g 500dps
ICM45686_Reader::~ICM45686_Reader() {
    if (i2c_file_ >= 0) {
        close(i2c_file_);
    }
}

bool ICM45686_Reader::initialize() {
    // 打开I2C总线
    i2c_file_ = open(i2c_bus_.c_str(), O_RDWR);
    if (i2c_file_ < 0) {
        std::cerr << "无法打开I2C总线: " << i2c_bus_ << std::endl;
        return false;
    }

    // 设置I2C设备地址
    if (ioctl(i2c_file_, I2C_SLAVE, device_addr_) < 0) {
        std::cerr << "无法设置I2C设备地址: 0x" << std::hex << static_cast<int>(device_addr_) << std::endl;
        close(i2c_file_);
        i2c_file_ = -1;
        return false;
    }

    // 检查设备ID
    if (!check_device_id()) {
        std::cerr << "设备ID验证失败" << std::endl;
        return false;
    }

    // 软复位
    if (!write_register(static_cast<uint8_t>(BANK_0::REG_MISC2), 0x01)) {
        std::cerr << "软复位失败" << std::endl;
        return false;
    }
    usleep(10000); // 等待复位完成

    // 配置传感器 
    // 加速度：低噪音 量程为4g 400Hz
    // 陀螺仪：低噪音 量程为500dps 400Hz
    if (!set_accel_mode(Mode::LowNoise, AccelScale::Scale4g, ODR::Rate400Hz)) {
        std::cerr << "配置加速度计失败" << std::endl;
        return false;
    }

    if (!set_gyro_mode(Mode::LowNoise, GyroScale::Scale500dps, ODR::Rate400Hz)) {
        std::cerr << "配置陀螺仪失败" << std::endl;
        return false;
    }

    std::cout << "ICM45686初始化成功" << std::endl;
    return true;
}

bool ICM45686_Reader::set_accel_mode(Mode mode, AccelScale scale, ODR odr) {
    // 读取并设置PWR_MGMT0
    uint8_t pwr_mgmt0;
    if (!read_register(static_cast<uint8_t>(BANK_0::PWR_MGMT0), pwr_mgmt0)) {
        return false;
    }
    pwr_mgmt0 = (pwr_mgmt0 & 0xFC) | (static_cast<uint8_t>(mode) & 0x03);
    // 设置加速度计模式
    if (!write_register(static_cast<uint8_t>(BANK_0::PWR_MGMT0), pwr_mgmt0)) {
        return false;
    }

    // 设置加速度计量程和ODR
    uint8_t acc_config;
    if (!read_register(static_cast<uint8_t>(BANK_0::ACCEL_CONFIG0), acc_config)) {
        return false;
    }

    // 更新灵敏度
    switch (scale) {
    case AccelScale::Scale32g: accel_sensitivity_ = 1024.0f; break;
    case AccelScale::Scale16g: accel_sensitivity_ = 2048.0f; break;
    case AccelScale::Scale8g: accel_sensitivity_ = 4096.0f; break;
    case AccelScale::Scale4g: accel_sensitivity_ = 8192.0f; break;
    case AccelScale::Scale2g: accel_sensitivity_ = 16384.0f; break;
    }

    // 配置寄存器：保留最高位，设置FS_SEL和ODR
    acc_config = ((static_cast<uint8_t>(scale) & 0x07) << 4) |
        (static_cast<uint8_t>(odr) & 0x0F);
    return write_register(static_cast<uint8_t>(BANK_0::ACCEL_CONFIG0), acc_config);
}

bool ICM45686_Reader::set_gyro_mode(Mode mode, GyroScale scale, ODR odr) {
    // 读取并设置PWR_MGMT0
    uint8_t pwr_mgmt0;
    if (!read_register(static_cast<uint8_t>(BANK_0::PWR_MGMT0), pwr_mgmt0)) {
        return false;
    }
    pwr_mgmt0 = (pwr_mgmt0 & 0xF3) | ((static_cast<uint8_t>(mode) & 0x03) << 2); // 设置陀螺仪模式
    if (!write_register(static_cast<uint8_t>(BANK_0::PWR_MGMT0), pwr_mgmt0)) {
        return false;
    }

    // 设置陀螺仪量程和ODR
    uint8_t gyro_config;
    if (!read_register(static_cast<uint8_t>(BANK_0::GYRO_CONFIG0), gyro_config)) {
        return false;
    }

    // 更新灵敏度
    // 根据量程计算灵敏度（LSB/°/s）
    switch (scale) {
    case GyroScale::Scale4000dps: gyro_sensitivity_ = 8.2f; break;   // 4000dps: 8.2 LSB/°/s
    case GyroScale::Scale2000dps: gyro_sensitivity_ = 16.4f; break;  // 2000dps: 16.4 LSB/°/s
    case GyroScale::Scale1000dps: gyro_sensitivity_ = 32.8f; break;  // 1000dps: 32.8 LSB/°/s
    case GyroScale::Scale500dps: gyro_sensitivity_ = 65.5f; break;  // 500dps: 65.5 LSB/°/s
    case GyroScale::Scale250dps: gyro_sensitivity_ = 131.0f; break; // 250dps: 131 LSB/°/s
    case GyroScale::Scale125dps: gyro_sensitivity_ = 262.0f; break; // 125dps: 262 LSB/°/s
    case GyroScale::Scale62dps: gyro_sensitivity_ = 524.0f; break;   // 62.5dps: 524 LSB/°/s
    case GyroScale::Scale31dps: gyro_sensitivity_ = 1048.0f; break;  // 31.25dps: 1048 LSB/°/s
    case GyroScale::Scale15dps: gyro_sensitivity_ = 2096.0f; break;  // 15.625dps: 2096 LSB/°/s
    case GyroScale::Scale6dps: gyro_sensitivity_ = 4192.0f; break;   // 7.8125dps: 4192 LSB/°/s
    }

    // 配置寄存器：设置FS_SEL和ODR
    gyro_config = ((static_cast<uint8_t>(scale) & 0x0F) << 4) | (static_cast<uint8_t>(odr) & 0x0F);
    return write_register(static_cast<uint8_t>(BANK_0::GYRO_CONFIG0), gyro_config);
}

bool ICM45686_Reader::check_device_id() {
    uint8_t device_id;
    if (!read_register(static_cast<uint8_t>(BANK_0::WHO_AM_I), device_id)) {
        return false;
    }

    std::cout << "设备ID: 0x" << std::hex << static_cast<int>(device_id) << std::dec << std::endl;

    if ((device_id == 0xE9 || device_id == 0xE5)) {
        std::cout << "设备ID验证成功" << std::endl;
        return true;
    }
    else {
        std::cerr << "设备ID验证失败，期望: 0xE9或0xE5，实际: 0x"
            << std::hex << static_cast<int>(device_id) << std::dec << std::endl;
        return false;
    }
}

bool ICM45686_Reader::read_accelerometer(float& x, float& y, float& z) {
    uint8_t buffer[6];
    if (!read_registers(static_cast<uint8_t>(BANK_0::ACCEL_DATA_X1_UI), buffer, 6)) {
        return false;
    }

    // 使用小端序读取
    int16_t raw_x = combine_bytes(buffer[0], buffer[1]); // 低字节在前
    int16_t raw_y = combine_bytes(buffer[2], buffer[3]);
    int16_t raw_z = combine_bytes(buffer[4], buffer[5]);

    // g/s2
    x = static_cast<float>(raw_x) / accel_sensitivity_ ;
    y = static_cast<float>(raw_y) / accel_sensitivity_ ;
    z = static_cast<float>(raw_z) / accel_sensitivity_;
    return true;
}

bool ICM45686_Reader::read_gyroscope(float& x, float& y, float& z) {
    uint8_t buffer[6];
    if (!read_registers(static_cast<uint8_t>(BANK_0::GYRO_DATA_X1_UI), buffer, 6)) {
        return false;
    }

    // 使用小端序读取
    int16_t raw_x = combine_bytes(buffer[0], buffer[1]); 
    int16_t raw_y = combine_bytes(buffer[2], buffer[3]);
    int16_t raw_z = combine_bytes(buffer[4], buffer[5]);

    
    x = static_cast<float>(raw_x) / gyro_sensitivity_ ;
    y = static_cast<float>(raw_y) / gyro_sensitivity_ ;
    z = static_cast<float>(raw_z) / gyro_sensitivity_ ;

    return true;
}

bool ICM45686_Reader::read_temperature(float& temperature) {
    uint8_t buffer[2];
    if (!read_registers(static_cast<uint8_t>(BANK_0::TEMP_DATA1_UI), buffer, 2)) {
        return false;
    }

    // 使用小端序读取
    int16_t raw_temp = combine_bytes(buffer[0], buffer[1]);

    // 温度转换
    temperature = static_cast<float>(raw_temp) / 128.0f + 25.0f;

    return true;
}

bool ICM45686_Reader::read_accelerometer_raw(int16_t& x, int16_t& y, int16_t& z) {
    uint8_t buffer[6];
    if (!read_registers(static_cast<uint8_t>(BANK_0::ACCEL_DATA_X1_UI), buffer, 6)) {
        return false;
    }
    
    x = combine_bytes(buffer[0], buffer[1]);
    y = combine_bytes(buffer[2], buffer[3]);
    z = combine_bytes(buffer[4], buffer[5]);
    
    return true;
}

bool ICM45686_Reader::read_gyroscope_raw(int16_t& x, int16_t& y, int16_t& z) {
    uint8_t buffer[6];
    if (!read_registers(static_cast<uint8_t>(BANK_0::GYRO_DATA_X1_UI), buffer, 6)) {
        return false;
    }
    
    x = combine_bytes(buffer[0], buffer[1]);
    y = combine_bytes(buffer[2], buffer[3]);
    z = combine_bytes(buffer[4], buffer[5]);
    
    return true;
}

bool ICM45686_Reader::read_temperature_raw(int16_t& temp) {
    uint8_t buffer[2];
    if (!read_registers(static_cast<uint8_t>(BANK_0::TEMP_DATA1_UI), buffer, 2)) {
        return false;
    }
    
    temp = combine_bytes(buffer[0], buffer[1]);
    
    return true;
}

// 私有辅助方法
bool ICM45686_Reader::write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    if (write(i2c_file_, buffer, 2) != 2) {
        std::cerr << "写入寄存器失败: 0x" << std::hex << static_cast<int>(reg) << std::endl;
        return false;
    }
    return true;
}

bool ICM45686_Reader::read_register(uint8_t reg, uint8_t& value) {
    if (write(i2c_file_, &reg, 1) != 1) {
        std::cerr << "写入寄存器地址失败: 0x" << std::hex << static_cast<int>(reg) << std::endl;
        return false;
    }

    if (read(i2c_file_, &value, 1) != 1) {
        std::cerr << "读取寄存器失败: 0x" << std::hex << static_cast<int>(reg) << std::endl;
        return false;
    }

    return true;
}

bool ICM45686_Reader::read_registers(uint8_t start_reg, uint8_t* buffer, uint8_t length) {
    if (write(i2c_file_, &start_reg, 1) != 1) {
        std::cerr << "写入起始寄存器失败: 0x" << std::hex << static_cast<int>(start_reg) << std::endl;
        return false;
    }

    if (read(i2c_file_, buffer, length) != length) {
        std::cerr << "读取多个寄存器失败，起始地址: 0x" << std::hex << static_cast<int>(start_reg) << std::endl;
        return false;
    }

    return true;
}

int16_t ICM45686_Reader::combine_bytes(uint8_t low_byte, uint8_t high_byte) {
    return static_cast<int16_t>((high_byte << 8) | low_byte);
}

int16_t ICM45686_Reader::to_signed_int(uint16_t value) {
    if (value & 0x8000) {
        return static_cast<int16_t>(value - 0x10000);
    }
    else {
        return static_cast<int16_t>(value);
    }
}
