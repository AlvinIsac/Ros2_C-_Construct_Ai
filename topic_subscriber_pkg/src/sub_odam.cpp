#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include <chrono>

class OdamSubscriber : public rclcpp::Node
{
public:
  OdamSubscriber(): Node("odam_subscriber")
  {
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odam", 10, std::bind(&OdamSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", message->pose.pose.position.x);
  }
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdamSubscriber>());
  rclcpp::shutdown();
  return 0; 
}