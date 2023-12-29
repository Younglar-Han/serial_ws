#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
class Ros2_Serial : public rclcpp::Node
{
public:
  Ros2_Serial() : Node("ros2_serial")
  {
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Ros2_Serial::callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&Ros2_Serial::timer_callback, this));
    tf_boadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    my_serial.setPort("/dev/tty_SERIAL0");
    my_serial.setBaudrate(9600);
    my_serial.setTimeout(100, 100, 0, 100, 0);
    if (!my_serial.isOpen())
    {
      my_serial.open();
    }
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
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

  void timer_callback()
  { 
    std::string data = my_serial.read(11);
    if (data == "")
    {
      return;
    }
    uint8_t header = data[0];
    char linearxTemp[4] = {data[1], data[2], data[3], data[4]};
    float linearx = *(float *)linearxTemp;
    char angularzTemp[4] = {data[5], data[6], data[7], data[8]};
    float angularz = *(float *)angularzTemp;
    // uint8_t checkCode = data[9];
    uint8_t tailer = data[10];
    if (header != 0xA5 || tailer != 0x5A)
    {
      return;
    }
    vx = linearx;
    vth = angularz;
    my_serial.flush();

    current_time = rclcpp::Clock().now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
 
    x += delta_x;
    y += delta_y;
    th += delta_th;

    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = current_time;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";

    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.w = cos(th / 2);
    message.pose.pose.orientation.x = 0.0;
    message.pose.pose.orientation.y = 0.0;
    message.pose.pose.orientation.z = sin(th / 2);
    
    message.twist.twist.linear.x = vx;
    message.twist.twist.linear.y = vy;
    message.twist.twist.angular.z = vth;

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.w = cos(th / 2);
    odom_trans.transform.rotation.x = 0.0;
    odom_trans.transform.rotation.y = 0.0;
    odom_trans.transform.rotation.z = sin(th / 2);

    geometry_msgs::msg::TransformStamped footprint_trans;
    footprint_trans.header.stamp = current_time;
    footprint_trans.header.frame_id = "base_link";
    footprint_trans.child_frame_id = "base_footprint";

    tf_boadcaster_->sendTransform(odom_trans);
    tf_boadcaster_->sendTransform(footprint_trans);
    odom_pub_->publish(message);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_boadcaster_;
  serial::Serial my_serial;
  rclcpp::Time last_time;
  rclcpp::Time current_time;
  double x = 0;
  double y = 0;
  double th = 0;
  double vx = 0;
  double vy = 0;
  double vth = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2_Serial>());
  rclcpp::shutdown();
  return 0;
}