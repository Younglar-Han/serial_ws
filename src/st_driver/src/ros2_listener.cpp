#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("ros2_listener")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Listener::callback, this, std::placeholders::_1));
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "linearx: %f angularz: %f", msg->linear.x, msg->angular.z);
    serial::Serial my_serial("/dev/tty_SERIAL0", 9600, serial::Timeout::simpleTimeout(1000));
    if (!my_serial.isOpen())
    {
      my_serial.open();
    }
    float linearx = msg->linear.x;
    float angularz = msg->angular.z;
    uint8_t header[] = {0xA5};
    uint8_t linearxTemp[4];
    uint8_t *linearxPtr = (uint8_t *)&linearx; // 指针
    uint8_t angularzTemp[4];
    uint8_t *angularzPtr = (uint8_t *)&angularz; // 指针
    uint8_t checkCodeTemp = 0;
    uint8_t checkCode[1];
    uint8_t tailer[] = {0x5A};

    for (int i = 0; i < 4; i++)
    {
      linearxTemp[i] = (linearxPtr[i] & 0xFF);
      angularzTemp[i] = (angularzPtr[i] & 0xFF);
      checkCodeTemp += linearxTemp[i] + angularzTemp[i];
    }
    checkCode[0] = checkCodeTemp;

    my_serial.write(header, 1);
    my_serial.write(linearxTemp, 4);
    my_serial.write(angularzTemp, 4);
    my_serial.write(checkCode, 1);
    my_serial.write(tailer, 1);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}