#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("ros2_listener")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10, std::bind(&Listener::callback, this, std::placeholders::_1));
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "linearx: %f angularz: %f", msg->linear.x, msg->angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // serial::Serial my_serial("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000));
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}