#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int32.hpp>
//#include "src/joy_pub_sub/msg/Screw.msg"

class MinimalSubscriber : public rclcpp::Node{
private:

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_twist_r_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_twist_l_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr viz_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  void _topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
public:
  MinimalSubscriber(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  MinimalSubscriber(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );

};
