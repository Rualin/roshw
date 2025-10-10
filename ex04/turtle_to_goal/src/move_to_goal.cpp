#include <cmath>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"


#define PI 3.14159265

class MoveToGoal: public rclcpp::Node
{
public:
    MoveToGoal(float x, float y, float theta)
    : Node("move_to_goal"), px(x), py(y), ptheta(theta) {
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&MoveToGoal::turtle_pose_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Node MoveToGoal initialized with x = %f, y = %f, theta = %f", x, y, theta);
    }

    float calc_distance(float x1, float y1, float x2, float y2) {
        return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    float calc_relative_angle(float x1, float y1, float x2, float y2) {
        if (x1 == x2) { // cos == 0
            return (y2 > y1) ? (PI / 2.0) : (-PI / 2.0);
        }
        float tang = (y2 - y1) / (x2 - x1);
        float arctang = atan(tang);
        if (tang > 0) { // I or III coordinate quarter
            return (x2 > x1) ? arctang : arctang - PI;
        }
        else if (tang == 0) { // sin == 0
            return (x2 > x1) ? 0.0 : (PI - 0.000001);
        }
        else { // II or IV coordinate quarter
            return (x2 > x1) ? arctang : arctang + PI;
        }
    }

    float normalize_angle(float angle) {
        return angle - (2.0 * PI * std::floor((angle + PI) / (2.0 * PI)));
    }

    void turtle_pose_callback(turtlesim::msg::Pose::UniquePtr msg) {
        float cx = msg->x;
        float cy = msg->y;
        float ctheta = msg->theta;
        if (cx == px && cy == py) {
            return;
        }
        float relative_angle = this->calc_relative_angle(cx, cy, px, py);
        std::chrono::nanoseconds ns = std::chrono::nanoseconds(1100000000); // 1.1 second to wait until moving comleted
        geometry_msgs::msg::Twist vel_msg = geometry_msgs::msg::Twist();
        vel_msg.angular.z = relative_angle - ctheta;
        this->vel_publisher->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "Rotating turtle to goal...");
        rclcpp::sleep_for(ns);
        RCLCPP_INFO(this->get_logger(), "Rotated. Current theta = %f", relative_angle);
        vel_msg = geometry_msgs::msg::Twist();
        float dist = calc_distance(cx, cy, px, py);
        vel_msg.linear.x = dist;
        this->vel_publisher->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "Moving turtle to goal...");
        rclcpp::sleep_for(ns);
        RCLCPP_INFO(this->get_logger(), "Moved. Current: x = %f, y = %f", cx + cosf(relative_angle) * dist, cy + sinf(relative_angle) * dist);
        vel_msg = geometry_msgs::msg::Twist();
        vel_msg.angular.z = this->normalize_angle(ptheta - relative_angle);
        this->vel_publisher->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "Rotating turtle to final theta...");
        rclcpp::sleep_for(ns);
        RCLCPP_INFO(this->get_logger(), "Rotated. Delta theta = %f", this->normalize_angle(ptheta - relative_angle));
        completion_promise.set_value();
    }

    std::shared_future<void> get_completion_future() {
        return completion_promise.get_future().share();
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    float px;
    float py;
    float ptheta;
    std::promise<void> completion_promise;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    float x = std::atof(argv[1]);
    float y = std::atof(argv[2]);
    float theta = std::atof(argv[3]);
    std::shared_ptr<MoveToGoal> node = std::make_shared<MoveToGoal>(x, y, theta);
    auto completion_future = node->get_completion_future();

    if (rclcpp::spin_until_future_complete(node, completion_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node MoveToGoal has succesfully worked");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to complete node MoveToGoal succesfully");
    }
    rclcpp::shutdown();
    return 0;
}
