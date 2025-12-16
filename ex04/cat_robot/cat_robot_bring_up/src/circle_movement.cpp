#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


#define PI 3.1415
using namespace std::chrono_literals;

class CirclePublisher : public rclcpp::Node
{
public:
    CirclePublisher() : Node("circle_publisher_node") {
        speed = this->declare_parameter<double>("speed", 1.0);
        rotation = this->declare_parameter<double>("rotation", PI / 2.0);
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer = this->create_wall_timer(1s, std::bind(&CirclePublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Initialized CirclePublisher");
    }

    void timer_callback() {
        geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
        msg.linear.x = speed;
        msg.angular.z = rotation;
        publisher->publish(msg);
    }

private:
    double speed, rotation;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CirclePublisher>());
  rclcpp::shutdown();
  return 0;
}
