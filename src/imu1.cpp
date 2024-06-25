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
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "oak/imu/data", sensor_qos, std::bind(&IMU::subscription_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback()
    {
      imu_message.header.stamp = this->get_clock()->now();
      imu_message.angular_velocity.x = sub_message.angular_velocity.x;
      imu_message.angular_velocity.y = sub_message.angular_velocity.y;
      imu_message.angular_velocity.z = sub_message.angular_velocity.z;
      imu_message.linear_acceleration.x = sub_message.linear_acceleration.x;
      imu_message.linear_acceleration.y = sub_message.linear_acceleration.y;
      imu_message.linear_acceleration.z = sub_message.linear_acceleration.z;

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

    void subscription_callback(const sensor_msgs::msg::Imu & msg)
    {
        sub_message = msg;
    }

    sensor_msgs::msg::Imu sub_message;
    sensor_msgs::msg::Imu imu_message;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU>());
  rclcpp::shutdown();
  return 0;
}