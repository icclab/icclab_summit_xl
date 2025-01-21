#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OdomToBaseLinkPublisher : public rclcpp::Node
{
public:
  OdomToBaseLinkPublisher() 
  : Node("odom_to_base_link_publisher")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/summit/robotnik_base_control/odom", 10, std::bind(&OdomToBaseLinkPublisher::odom_subscriber_callback, this, _1));
    
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/summit/odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:

  void odom_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
  {
    nav_msgs::msg::Odometry odom_pub;
    
    odom_pub.header = odometry->header;
    odom_pub.child_frame_id = odometry->child_frame_id;

    odom_pub.pose.pose.position.x = odometry->pose.pose.position.x;
    odom_pub.pose.pose.position.y = odometry->pose.pose.position.y;
    odom_pub.pose.pose.position.z = odometry->pose.pose.position.z;

    odom_pub.pose.pose.orientation.x = odometry->pose.pose.orientation.x;
    odom_pub.pose.pose.orientation.y = odometry->pose.pose.orientation.y;
    odom_pub.pose.pose.orientation.z = odometry->pose.pose.orientation.z;
    odom_pub.pose.pose.orientation.w = odometry->pose.pose.orientation.w;

    odom_pub.pose.covariance = odometry->pose.covariance;

    odom_pub.twist.twist.linear.x = odometry->twist.twist.linear.x;
    odom_pub.twist.twist.linear.y = odometry->twist.twist.linear.y;
    odom_pub.twist.twist.linear.z = odometry->twist.twist.linear.z;

    odom_pub.twist.twist.angular.x = odometry->twist.twist.angular.x;
    odom_pub.twist.twist.angular.y = odometry->twist.twist.angular.y;
    odom_pub.twist.twist.angular.z = odometry->twist.twist.angular.z;

     odom_pub.twist.covariance = odometry->twist.covariance;    

    publisher_->publish(odom_pub);

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header = odometry->header;
    transformStamped.child_frame_id = odometry->child_frame_id;
    
    transformStamped.transform.translation.x = odometry->pose.pose.position.x;
    transformStamped.transform.translation.y = odometry->pose.pose.position.y;
    transformStamped.transform.translation.z = odometry->pose.pose.position.z;
    
    transformStamped.transform.rotation.x = odometry->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odometry->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odometry->pose.pose.orientation.z;     
    transformStamped.transform.rotation.w = odometry->pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToBaseLinkPublisher>());
  rclcpp::shutdown();
  return 0;
}