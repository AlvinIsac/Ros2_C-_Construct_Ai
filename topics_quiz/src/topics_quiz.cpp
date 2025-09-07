#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TopicQuiz : public rclcpp::Node
{
public:
  TopicQuiz(): Node("move_robot")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subcriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&TopicQuiz::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto cmd_msg = geometry_msgs::msg::Twist();

    float front = msg->ranges[msg->ranges.size() / 2];  // Front
    float left = msg->ranges[msg->ranges.size() * 3 / 4];  // Left
    float right = msg->ranges[msg->ranges.size() / 4];  // Right

    RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f", front, left, right);

    // Obstacle Avoidance Logic
    const float SAFE_DISTANCE = 1.0;

    if (front > SAFE_DISTANCE)
    {
        // Move forward
        cmd_msg.linear.x = 0.5;
        cmd_msg.angular.z = 0.0;
    }
    else if (front <= SAFE_DISTANCE)
    {
        // Turn left
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.5;
    }
    else if (right <= SAFE_DISTANCE)
    {
        // Turn left
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.5;
    }
    else if (left <= SAFE_DISTANCE)
    {
        // Turn right
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = -0.5;
    }

    // Publish the command
    publisher_->publish(cmd_msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subcriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicQuiz>());
  rclcpp::shutdown();
  return 0;
}