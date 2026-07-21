#include "ICM45686_Reader.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace {
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

class ImuPublisherNode : public rclcpp::Node {
public:
  ImuPublisherNode() : Node("imu_publisher") {
    i2c_bus_ = declare_parameter<std::string>("i2c_bus", "/dev/i2c-8");
    i2c_addr_ = static_cast<uint8_t>(declare_parameter<int>("i2c_addr", 0x69));
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    publish_topic_ = declare_parameter<std::string>("publish_topic", "imu/data_raw");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 300.0);
    accel_covariance_ = declare_parameter<double>("accel_covariance", 0.04);
    gyro_covariance_ = declare_parameter<double>("gyro_covariance", 0.02);

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }

    sensor_ = std::make_unique<ICM45686_Reader>(i2c_addr_, i2c_bus_);
    RCLCPP_INFO(get_logger(), "Initializing ICM45686 on %s addr 0x%02x",
                i2c_bus_.c_str(), static_cast<unsigned int>(i2c_addr_));
    if (!sensor_->initialize()) {
      throw std::runtime_error("Failed to initialize ICM45686");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        publish_topic_, rclcpp::SensorDataQoS());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ImuPublisherNode::publishImu, this));

    RCLCPP_INFO(get_logger(), "Publishing IMU on '%s' at %.1f Hz, frame_id='%s'",
                publish_topic_.c_str(), publish_rate_hz_, frame_id_.c_str());
  }

private:
  void publishImu() {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    if (!sensor_->read_accelerometer(accel_x, accel_y, accel_z) ||
        !sensor_->read_gyroscope(gyro_x, gyro_y, gyro_z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to read IMU data");
      return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // ICM45686 driver reports acceleration in g and angular velocity in deg/s.
    msg.linear_acceleration.x = static_cast<double>(accel_x) * kGravity;
    msg.linear_acceleration.y = static_cast<double>(accel_y) * kGravity;
    msg.linear_acceleration.z = static_cast<double>(accel_z) * kGravity;

    msg.angular_velocity.x = static_cast<double>(gyro_x) * kDegToRad;
    msg.angular_velocity.y = static_cast<double>(gyro_y) * kDegToRad;
    msg.angular_velocity.z = static_cast<double>(gyro_z) * kDegToRad;

    msg.orientation_covariance[0] = -1.0;  // No fused orientation estimate.
    msg.linear_acceleration_covariance[0] = accel_covariance_;
    msg.linear_acceleration_covariance[4] = accel_covariance_;
    msg.linear_acceleration_covariance[8] = accel_covariance_;
    msg.angular_velocity_covariance[0] = gyro_covariance_;
    msg.angular_velocity_covariance[4] = gyro_covariance_;
    msg.angular_velocity_covariance[8] = gyro_covariance_;

    publisher_->publish(msg);
  }

  std::string i2c_bus_;
  uint8_t i2c_addr_;
  std::string frame_id_;
  std::string publish_topic_;
  double publish_rate_hz_;
  double accel_covariance_;
  double gyro_covariance_;

  std::unique_ptr<ICM45686_Reader> sensor_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ImuPublisherNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_publisher"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
