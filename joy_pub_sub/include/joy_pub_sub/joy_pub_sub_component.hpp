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
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr screw_publisher_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr screw_test_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr screw_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr screw_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr screw_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr screw_rl_publisher_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ballast_up_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ballast_dawn_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_twist_publisher_;

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
