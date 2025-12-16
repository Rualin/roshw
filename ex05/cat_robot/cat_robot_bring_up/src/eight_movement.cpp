#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define PI 3.1415
using namespace std::chrono_literals;

class FigureEightPublisher : public rclcpp::Node
{
public:
    FigureEightPublisher() : Node("figure_eight_publisher_node")
    {
        speed = this->declare_parameter<double>("speed", 0.5);
        rotation = this->declare_parameter<double>("rotation", PI / 2.0);
        switch_time = this->declare_parameter<int>("switch_time", 6);

        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer = this->create_wall_timer(
            100ms, std::bind(&FigureEightPublisher::timer_callback, this));

        start_time = this->now();

        RCLCPP_INFO(this->get_logger(), "Initialized FigureEightPublisher");
    }

private:
    void timer_callback()
    {
        auto now = this->now();
        double elapsed = (now - start_time).seconds();

        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed;

        // Alternate rotation direction every switch_time seconds
        if (static_cast<int>(elapsed / switch_time) % 2 == 0)
            msg.angular.z = rotation;     // left circle
        else
            msg.angular.z = -rotation;    // right circle

        publisher->publish(msg);
    }

    double speed;
    double rotation;
    int switch_time;

    rclcpp::Time start_time;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FigureEightPublisher>());
    rclcpp::shutdown();
    return 0;
}
