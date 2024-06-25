#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;


class Odometry1 : public rclcpp::Node
{
  public:
    Odometry1()
    : Node("odometry"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("topic", 10);
      timer_ = this->create_wall_timer(66ms, std::bind(&Odometry1::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = nav_msgs::msg::Odometry();

      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry1>());
  rclcpp::shutdown();
  return 0;
}