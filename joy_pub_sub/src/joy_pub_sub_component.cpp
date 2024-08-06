#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include "../include/joy_pub_sub/joy_pub_sub_component.hpp"

using namespace std::chrono_literals;
int convertAnalogStick(float input, float a, float b);

geometry_msgs::msg::Twist diff_twist_msg;

void MinimalSubscriber::_topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg){

    if(msg->axes[4] > 0){//left
    }

    else if(msg->axes[4] < 0){//right

    }

    else if(msg->axes[5] > 0){//forward

    }

    else if(msg->axes[5] < 0){//back

    }

    else if(msg->buttons[2] == 1){//CW 2s approximately 45 degree

    }


    else if(msg->buttons[0] == 1){//CCW strong 1s approximately 45 degree

    }

    else if(msg->buttons[3] == 1){

    }

    else if(msg->buttons[1] == 1){

    }

    else{

      diff_twist_msg.linear.x = 0;
      diff_twist_msg.linear.y = 0;
      diff_twist_msg.linear.z = 0;
      diff_twist_msg.angular.x = 0;
      diff_twist_msg.angular.y = 0;
      diff_twist_msg.angular.z = 0;

    }

      printf("[cmd_vel] x: %f, y: %f, z: %f\n", diff_twist_msg.linear.x, diff_twist_msg.linear.y, diff_twist_msg.angular.z);

}

MinimalSubscriber::MinimalSubscriber(
  const rclcpp::NodeOptions& options
): MinimalSubscriber("",options){}

MinimalSubscriber::MinimalSubscriber(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("minimal_subscriber_test", name_space, options){

    diff_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_0",rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
    300ms,//Twistのパブリッシュ周波数
    [this](){
        diff_twist_publisher_ ->publish(diff_twist_msg);
    }
    );

  _subscription = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy",
    rclcpp::QoS(10),
    std::bind(&MinimalSubscriber::_topic_callback, this, std::placeholders::_1)
  );

//val Init

  diff_twist_msg.linear.x = 0;
  diff_twist_msg.linear.y = 0;
  diff_twist_msg.linear.z = 0;
  diff_twist_msg.angular.x = 0;
  diff_twist_msg.angular.y = 0;
  diff_twist_msg.angular.z = 0;

}

