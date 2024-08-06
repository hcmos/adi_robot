#include <rclcpp/rclcpp.hpp>
#include "../include/joy_pub_sub/joy_pub_sub_component.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
