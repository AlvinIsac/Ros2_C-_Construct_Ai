#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/age.hpp"
#include <chrono>

using namespace std::chrono_literals;

class AgePublisher : public rclcpp::Node
{
public:
  AgePublisher(): Node("simple_publisher")
  {
    publisher_ = this->create_publisher<custom_interfaces::msg::Age>("age", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&AgePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = custom_interfaces::msg::Age();
    message.years = 1999;
    message.months = 9;
    message.days = 9;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgePublisher>());
  rclcpp::shutdown();
  return 0; 
}