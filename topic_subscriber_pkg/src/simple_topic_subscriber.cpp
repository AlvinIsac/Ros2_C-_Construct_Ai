#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber(): Node("simple_subscriber")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>("counter", 10, std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr message)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", message->data);
  }
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0; 
}