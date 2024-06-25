#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>


using namespace std::chrono_literals;

auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

class IMU : public rclcpp::Node
{
  public:
    IMU()
    : Node("imu"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("car_bot/imu", 10);
      timer_ = this->create_wall_timer(66ms, std::bind(&IMU::timer_callback, this));
      gyro_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "camera/gyro/sample", sensor_qos, std::bind(&IMU::gyro_callback, this, std::placeholders::_1));
      accel_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "camera/accel/sample", sensor_qos, std::bind(&IMU::accel_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback()
    {
      imu_message.header.stamp = this->get_clock()->now();
      imu_message.angular_velocity.x = gyro_message.angular_velocity.x;
      imu_message.angular_velocity.y = gyro_message.angular_velocity.z;
      imu_message.angular_velocity.z = gyro_message.angular_velocity.y;
      imu_message.linear_acceleration.x = accel_message.linear_acceleration.x;
      imu_message.linear_acceleration.y = accel_message.linear_acceleration.y;
      imu_message.linear_acceleration.z = accel_message.linear_acceleration.z;

      publisher_->publish(imu_message);

      // auto t = this->get_clock().now;
      //RCLCPP_INFO(this->get_logger(), t);
      //RCLCPP_INFO(this->get_logger(), "Time: '%s'", std::to_string(this->get_clock()).c_str());

//      RCLCPP_INFO(this->get_logger(), "Gyro X: '%s'", std::to_string(imu_message.angular_velocity.x).c_str());
//      RCLCPP_INFO(this->get_logger(), "Gyro Y: '%s'", std::to_string(imu_message.angular_velocity.y).c_str());
//      RCLCPP_INFO(this->get_logger(), "Gyro Z: '%s'", std::to_string(imu_message.angular_velocity.z).c_str());
//      RCLCPP_INFO(this->get_logger(), "---");
//      RCLCPP_INFO(this->get_logger(), "Accel X: '%s'", std::to_string(imu_message.linear_acceleration.x).c_str());
//      RCLCPP_INFO(this->get_logger(), "Accel Y: '%s'", std::to_string(imu_message.linear_acceleration.y).c_str());
//      RCLCPP_INFO(this->get_logger(), "Accel Z: '%s'", std::to_string(imu_message.linear_acceleration.z).c_str());
//      RCLCPP_INFO(this->get_logger(), "___________________");
    }

    void gyro_callback(const sensor_msgs::msg::Imu & msg)
    {
        gyro_message = msg;
    }

    void accel_callback(const sensor_msgs::msg::Imu & msg)
    {
        accel_message = msg;
    }

    sensor_msgs::msg::Imu gyro_message;
    sensor_msgs::msg::Imu accel_message;
    sensor_msgs::msg::Imu imu_message;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_subscription;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU>());
  rclcpp::shutdown();
  return 0;
}