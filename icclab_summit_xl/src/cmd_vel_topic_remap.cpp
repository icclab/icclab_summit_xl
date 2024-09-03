#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class CmdVelTopicRemap : public rclcpp::Node
{
public:
  CmdVelTopicRemap() 
  : Node("cmd_vel_topic_remap")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/summit/cmd_vel", 10, std::bind(&CmdVelTopicRemap::cmd_vel_topic_remap_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/summit/robotnik_base_control/cmd_vel_unstamped", 10);
  }

private:

  void cmd_vel_topic_remap_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
  {
    geometry_msgs::msg::Twist cmd_vel_pub;

    cmd_vel_pub.linear.x = cmd_vel->linear.x;
    cmd_vel_pub.linear.y = cmd_vel->linear.y;
    cmd_vel_pub.linear.z = cmd_vel->linear.z;

    cmd_vel_pub.angular.x = cmd_vel->angular.x;
    cmd_vel_pub.angular.y = cmd_vel->angular.y;
    cmd_vel_pub.angular.z = cmd_vel->angular.z;

    publisher_->publish(cmd_vel_pub);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelTopicRemap>());
  rclcpp::shutdown();
  return 0;
}