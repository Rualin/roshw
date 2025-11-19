#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
    DynamicFrameBroadcaster() : Node("dynamic_frame_tf2_broadcaster") {
        radius = this->declare_parameter<double>("radius", 5.0);
        slow_coef = radius < 0.5 ? 1.0 : radius * 2.0;
        int dir = this->declare_parameter<int>("direction_of_rotation", 1);
        if ((dir != 1) && (dir != -1)) {
            RCLCPP_ERROR(
                this->get_logger(), "Incorrect direction_of_rotation argument (%d)! Must be 1 or -1",
                dir);
            return;
        }
        direction = (double)dir;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(100ms, std::bind(&DynamicFrameBroadcaster::broadcast_timer_callback, this));
        RCLCPP_INFO(
            this->get_logger(),
            "Initialized DynamicFrameBroadcaster with parameters:\nRadius: %f\nSlow coef: %f\nDirection: %f",
            radius, slow_coef, direction);
    }

private:
    void broadcast_timer_callback()
    {
        rclcpp::Time now = this->get_clock()->now();
        double x = direction * (std::fmod(now.seconds(), 2 * slow_coef) / slow_coef) * PI;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "turtle1";
        t.child_frame_id = "carrot1";
        t.transform.translation.x = radius * cos(x);
        t.transform.translation.y = radius * sin(x);
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double radius, direction, slow_coef;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
