#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include "../include/joy_pub_sub/joy_pub_sub_component.hpp"

#define GEAR 20
#define PERIMETER 0.628//周長[m]
#define WHEEL_BASE 0.517//車輪間距離[m]

using namespace std::chrono_literals;

float rpm_to_rad(float rpm);
float rad_to_ms(float rad);
float ms_calc(float rpm);

geometry_msgs::msg::Twist diff_twist_r_msg;
geometry_msgs::msg::Twist diff_twist_l_msg;

geometry_msgs::msg::Twist viz_msg;

void MinimalSubscriber::_topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg){

    if(msg->axes[7] > 0){//forward
      diff_twist_r_msg.linear.x =  300.0;
      diff_twist_l_msg.linear.x = -300.0;

      viz_msg.linear.x = 3;
      viz_msg.linear.y = 3;
      viz_msg.linear.z = 0;
      viz_msg.angular.x = 0;
      viz_msg.angular.y = 0;
      viz_msg.angular.z = 0;

      float i = 0;
      i = ms_calc(300);
      printf("FORMARD:%.2f[m/s]\n",i);

    }

    else if(msg->axes[7] < 0){//back
      diff_twist_r_msg.linear.x = -300;
      diff_twist_l_msg.linear.x =  300;

      viz_msg.linear.x = -1.5;
      viz_msg.linear.y = -1.5;
      viz_msg.linear.z = 0;
      viz_msg.angular.x = 0;
      viz_msg.angular.y = 0;
      viz_msg.angular.z = 0;

      float i = 0;
      i = ms_calc(300);
      printf("BACK:%.2f[m/s]\n",i);
    }

    else if(msg->axes[6] > 0){//left
      // diff_twist_r_msg.linear.x = 300.0;
      // diff_twist_l_msg.linear.x = 150.0;

      // viz_msg.linear.x = 1;
      // viz_msg.linear.y = 0.5;
      // viz_msg.linear.z = 0;
      // viz_msg.angular.x = 0;
      // viz_msg.angular.y = 0;
      // viz_msg.angular.z = -0.5;
    }

    else if(msg->axes[6] < 0){//right
      // diff_twist_r_msg.linear.x = 150.0;
      // diff_twist_l_msg.linear.x = 300.0;

      // viz_msg.linear.x = 0.5;
      // viz_msg.linear.y = 1.0;
      // viz_msg.linear.z = 0;
      // viz_msg.angular.x = 0;
      // viz_msg.angular.y = 0;
      // viz_msg.angular.z = 0.5;
    }

    else if(msg->buttons[1] == 1){//CW
      diff_twist_r_msg.linear.x = -150.0;
      diff_twist_l_msg.linear.x = -150.0;

      viz_msg.linear.x = 0;
      viz_msg.linear.y = 0;
      viz_msg.linear.z = 0;
      viz_msg.angular.x = 0;
      viz_msg.angular.y = 0;
      viz_msg.angular.z = 30;

      float i = 0, v = 0, w = 0;
      i = rpm_to_rad(150);
      v = rad_to_ms(i);
      w = 2 * v / WHEEL_BASE;
      printf("CW:%.2f[rad/s]\n",w);
    }

    else if(msg->buttons[3] == 1){//CCW
      diff_twist_r_msg.linear.x =  150.0;
      diff_twist_l_msg.linear.x =  150.0;

      viz_msg.linear.x = 0;
      viz_msg.linear.y = 0;
      viz_msg.linear.z = 0;
      viz_msg.angular.x = 0;
      viz_msg.angular.y = 0;
      viz_msg.angular.z = -30;

      float i = 0, v = 0, w = 0;
      i = rpm_to_rad(150);
      v = rad_to_ms(i);
      w = 2 * v / WHEEL_BASE;
      printf("CCW:%.2f[rad/s]\n",w);
    }

    else if(msg->buttons[3] == 1){

    }

    else if(msg->buttons[1] == 1){

    }

    else{

      diff_twist_r_msg.linear.x = 0;
      diff_twist_l_msg.linear.x = 0;

      viz_msg.linear.x = 0;
      viz_msg.linear.y = 0;
      viz_msg.linear.z = 0;
      viz_msg.angular.x = 0;
      viz_msg.angular.y = 0;
      viz_msg.angular.z = 0;

    }

      //printf("[cmd_vel] m/s motor0: %f[m/s], motor1: %f[m/s]\n", diff_twist_r_msg.linear.x, diff_twist_l_msg.linear.x);

}

MinimalSubscriber::MinimalSubscriber(
  const rclcpp::NodeOptions& options
): MinimalSubscriber("",options){}

MinimalSubscriber::MinimalSubscriber(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("minimal_subscriber_test", name_space, options){

    diff_twist_r_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_0", 10);
    diff_twist_l_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_1", 10);

    viz_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
    30ms,//Twistのパブリッシュ周波数
    [this](){
        diff_twist_r_publisher_->publish(diff_twist_r_msg);
        diff_twist_l_publisher_->publish(diff_twist_l_msg);

        viz_publisher_->publish(viz_msg);
    }
    );

  _subscription = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy",
    rclcpp::QoS(10),
    std::bind(&MinimalSubscriber::_topic_callback, this, std::placeholders::_1)
  );

//val Init
  diff_twist_r_msg.linear.x = 0;
  diff_twist_r_msg.linear.y = 0;
  diff_twist_r_msg.linear.z = 0;
  diff_twist_r_msg.angular.x = 0;
  diff_twist_r_msg.angular.y = 0;
  diff_twist_r_msg.angular.z = 0;

  diff_twist_l_msg.linear.x = 0;
  diff_twist_l_msg.linear.y = 0;
  diff_twist_l_msg.linear.z = 0;
  diff_twist_l_msg.angular.x = 0;
  diff_twist_l_msg.angular.y = 0;
  diff_twist_l_msg.angular.z = 0;

  viz_msg.linear.x = 0;
  viz_msg.linear.y = 0;
  viz_msg.linear.z = 0;
  viz_msg.angular.x = 0;
  viz_msg.angular.y = 0;
  viz_msg.angular.z = 0;


}

float rpm_to_rad(float rpm){
    return rpm * ((2 * 3.14) / 60);
  }

float rad_to_ms(float rad){
    return 0.1 * rad;
  }

float ms_calc(float rpm){
  return ((rpm / GEAR) * PERIMETER) / 60;
}
