#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class TextToCmdVel : public rclcpp::Node
{
public:
    TextToCmdVel() : Node("text_to_cmd_vel")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        auto topic_callback =
        [this](std_msgs::msg::String::UniquePtr msg) -> void {
            std::string string_msg = msg->data;
            auto twist_msg = geometry_msgs::msg::Twist();
            if (string_msg == std::string("turn_right")) {
                twist_msg.angular.z = -1.5;
            }
            else if (string_msg == std::string("turn_left")) {
                twist_msg.angular.z = 1.5;
            }
            else if (string_msg == std::string("move_forward")) {
                twist_msg.linear.x = 1.0;
            }
            else if (string_msg == std::string("move_backward")) {
                twist_msg.linear.x = -1.0;
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", string_msg.c_str());
            }
            this->publisher_->publish(twist_msg);
        };

        subscription_ = this->create_subscription<std_msgs::msg::String>("cmd_text", 10, topic_callback);
        RCLCPP_INFO(this->get_logger(), "Node TextToCmdVel created.\nReady to listen string messages...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TextToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
