#ifndef ICM45686_READER_H
#define ICM45686_READER_H

#include <cstdint>
#include <string>

class ICM45686_Reader {
public:
    // 模式定义
    enum class Mode : uint8_t {
        Off = 0x00,
        Standby = 0x01,
        LowPower = 0x02,
        LowNoise = 0x03
    };

    enum class AccelScale : uint8_t {
        Scale32g = 0x0,
        Scale16g = 0x1,
        Scale8g = 0x2,
        Scale4g = 0x3,
        Scale2g = 0x4
    };

    enum class GyroScale : uint8_t {
        Scale4000dps = 0x0,
        Scale2000dps = 0x1,
        Scale1000dps = 0x2,
        Scale500dps = 0x3,
        Scale250dps = 0x4,
        Scale125dps = 0x5,
        Scale62dps = 0x6,
        Scale31dps = 0x7,
        Scale15dps = 0x8,
        Scale6dps = 0x9
    };

    enum class ODR : uint8_t {
        Rate6400Hz = 3,
        Rate3200Hz = 4,
        Rate1600Hz = 5,
        Rate800Hz = 6,
        Rate400Hz = 7,
        Rate200Hz = 8,
        Rate100Hz = 9,
        Rate50Hz = 10,
        Rate25Hz = 11,
        Rate12Hz = 12,
        Rate6Hz = 13,
        Rate3Hz = 14,
        Rate1Hz = 15
    };

    // 寄存器地址定义
    enum class BANK_0 : uint8_t {
        ACCEL_DATA_X1_UI = 0x00,
        ACCEL_DATA_X0_UI = 0x01,
        ACCEL_DATA_Y1_UI = 0x02,
        ACCEL_DATA_Y0_UI = 0x03,
        ACCEL_DATA_Z1_UI = 0x04,
        ACCEL_DATA_Z0_UI = 0x05,
        GYRO_DATA_X1_UI = 0x06,
        GYRO_DATA_X0_UI = 0x07,
        GYRO_DATA_Y1_UI = 0x08,
        GYRO_DATA_Y0_UI = 0x09,
        GYRO_DATA_Z1_UI = 0x0A,
        GYRO_DATA_Z0_UI = 0x0B,
        TEMP_DATA1_UI = 0x0C,
        TEMP_DATA0_UI = 0x0D,
        PWR_MGMT0 = 0x10,
        ACCEL_CONFIG0 = 0x1B,
        GYRO_CONFIG0 = 0x1C,
        WHO_AM_I = 0x72,
        REG_MISC2 = 0x7F
    };

    ICM45686_Reader(uint8_t device_address = 0x69, const std::string& i2c_bus = "/dev/i2c-8");
    ~ICM45686_Reader();

    bool initialize();
    bool set_accel_mode(Mode mode, AccelScale scale, ODR odr);
    bool set_gyro_mode(Mode mode, GyroScale scale, ODR odr);
    bool read_accelerometer(float& x, float& y, float& z);
    bool read_gyroscope(float& x, float& y, float& z);
    bool read_temperature(float& temperature);
    bool check_device_id();
    bool read_accelerometer_raw(int16_t& x, int16_t& y, int16_t& z);
    bool read_gyroscope_raw(int16_t& x, int16_t& y, int16_t& z);
    bool read_temperature_raw(int16_t& temp);

private:
    uint8_t device_addr_;
    std::string i2c_bus_;
    int i2c_file_;
    float accel_sensitivity_;
    float gyro_sensitivity_;

    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool read_registers(uint8_t start_reg, uint8_t* buffer, uint8_t length);
    int16_t combine_bytes(uint8_t low_byte, uint8_t high_byte); // 修正：改为小端序
    int16_t to_signed_int(uint16_t value);
};

#endif // ICM45686_READER_H

