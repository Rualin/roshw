#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "service_full_name/srv/summ_full_name.hpp"

#include <memory>

void concatenate(const std::shared_ptr<service_full_name::srv::SummFullName::Request> request,
            std::shared_ptr<service_full_name::srv::SummFullName::Response> response)
{
    response->full_name = request->last_name + request->first_name + request->patronymic;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nlast_name: %s;\tfirst_name: %s;\tpatronymic: %s",
                    request->last_name.c_str(), request->first_name.c_str(), request->patronymic.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->full_name.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_name");

    rclcpp::Service<service_full_name::srv::SummFullName>::SharedPtr service =
    node->create_service<service_full_name::srv::SummFullName>("summ_full_name", &concatenate);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to concatenate name parts.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
