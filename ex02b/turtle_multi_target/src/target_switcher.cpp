#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "turtle_multi_target_intefaces/srv/target_switch.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
// #include "std_msgs/msg/uint32.hpp"

#define PI 3.141592653589793238463

class TargetSwitcher: public rclcpp::Node
{
public:
    TargetSwitcher(): Node("target_switcher"), spawning_service_ready(false), turtles_spawned(false) {
        // Read run parameters
        first_name = this->declare_parameter<std::string>("first_turtle_name", "turtle1");
        second_name = this->declare_parameter<std::string>("second_turtle_name", "turtle2");
        third_name = this->declare_parameter<std::string>("third_turtle_name", "turtle3");
        first_carrot_name = this->declare_parameter<std::string>("first_carrot_name", "carrot1");
        second_carrot_name = this->declare_parameter<std::string>("second_carrot_name", "carrot2");
        static_name = this->declare_parameter<std::string>("static_target_name", "static_target");
        radius = this->declare_parameter<double>("radius", 5.0);
        slow_coef = radius < 0.5 ? 1.0 : radius * 2.0;
        int dir = this->declare_parameter<int>("direction_of_rotation", 1);
        if ((dir != 1) && (dir != -1)) {
            RCLCPP_ERROR(
                this->get_logger(), "Incorrect direction_of_rotation argument (%d)! Must be 1 or -1",
                dir);
            rclcpp::shutdown();
        }
        direction = (double)dir;
        switch_threshold = this->declare_parameter<double>("switch_threshold", 1.0);

        // // Targets init
        // targets = {first_carrot_name, second_carrot_name, static_name};
        // current_target_id = 0;

        // Current target name publisher and target switch service
        // target_id_pub = this->create_publisher<std_msgs::msg::UInt32>("/current_target_id", 1);
        // target_switch_service = this->create_service<turtle_multi_target_intefaces::srv::TargetSwitch>("/target_switch",
        //     std::bind(&TargetSwitcher::switch_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create tf2 broadcaster and static broadcaster
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Service to spawn turtles and spawning timer
        spawner = this->create_client<turtlesim::srv::Spawn>("spawn");
        spawn_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TargetSwitcher::spawn_timer_callback, this));

        // Turtles' poses to tf2 broadcasters
        std::string topic_name = std::string("/") + first_name + std::string("/pose");
        first_turtle_pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {this->turtle_broadcast_handler(msg, this->first_name);});

        topic_name = std::string("/") + second_name + std::string("/pose");
        second_turtle_pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {this->turtle_broadcast_handler(msg, this->second_name);});

        topic_name = std::string("/") + third_name + std::string("/pose");
        third_turtle_pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {this->turtle_broadcast_handler(msg, this->third_name);});

        // Timer for carrot rotating
        auto carrot_timer_callback = [this]() {
            this->carrot_broadcast_callback(first_name, first_carrot_name);
            this->carrot_broadcast_callback(third_name, second_carrot_name);
        };
        carrot_timer = this->create_wall_timer(std::chrono::milliseconds(100), carrot_timer_callback);

        // Static target broadcast
        geometry_msgs::msg::TransformStamped t = this->make_transform(
            this->get_clock()->now(), std::string("world"), static_name,
            8.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        tf_static_broadcaster->sendTransform(t);

        RCLCPP_INFO(this->get_logger(), "TargetSwitcher node initialized");
    }

private:
    geometry_msgs::msg::TransformStamped make_transform(
        rclcpp::Time stamp, std::string parent, std::string child, double x, double y, double z,
        double rx, double ry, double rz, double rw) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = stamp;
        t.header.frame_id = parent.c_str();
        t.child_frame_id = child.c_str();
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        t.transform.rotation.x = rx;
        t.transform.rotation.y = ry;
        t.transform.rotation.z = rz;
        t.transform.rotation.w = rw;
        return t;
    }

    void turtle_broadcast_handler(const std::shared_ptr<turtlesim::msg::Pose> msg, const std::string &child_name) {
        if (!turtles_spawned) {
            return;
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        geometry_msgs::msg::TransformStamped t = this->make_transform(
            this->get_clock()->now(), std::string("world"), child_name,
            msg->x, msg->y, 0.0, q.x(), q.y(), q.z(), q.w());

        tf_broadcaster->sendTransform(t);
    }

    void carrot_broadcast_callback(std::string parent_name, std::string child_name) {
        if (!turtles_spawned) {
            return;
        }

        rclcpp::Time now = this->get_clock()->now();
        double x = direction * (std::fmod(now.seconds(), 2.0 * slow_coef) / slow_coef) * PI;

        geometry_msgs::msg::TransformStamped t = this->make_transform(
            now, parent_name, child_name, radius* cos(x), radius * sin(x), 0.0, 0.0, 0.0, 0.0, 1.0);

        tf_broadcaster->sendTransform(t);
    }

    bool send_spawn_request(double x, double y, double theta, std::string name) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = name;
        // Call request
        bool status = false;
        auto response_received_callback = [this, &name, &status](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), name.c_str()) == 0) {
                status = true;
            }
            else {
                RCLCPP_ERROR(
                    this->get_logger(), 
                    "Service callback for %s result mismatch (result == %s)", 
                    name.c_str(), result->name.c_str());
            }
        };
        auto result = spawner->async_send_request(request, response_received_callback);
        return status;
    }

    void spawn_timer_callback() {
        if (spawning_service_ready) {
            if (turtles_spawned) {
                spawn_timer->cancel();
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Successfully spawned");
                turtles_spawned = true;
            }
        }
        else {
            if (spawner->service_is_ready()) {
                bool status = true;
                status &= this->send_spawn_request(5.0, 5.0, 0.0, second_name);
                status &= this->send_spawn_request(12.0, 12.0, 0.0, third_name);
                spawning_service_ready = true;
                if (status) {
                    spawning_service_ready = true;
                    RCLCPP_INFO(this->get_logger(), "Correct responses were received from the spawning service");
                }
                else {
                    RCLCPP_ERROR(this->get_logger(), "Not all responses received from the spawning service was correct!");
                    // throw std::runtime_error("Not all responses received from the spawning service was correct!");
                }
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }
    }

    // void switch_callback(const std::shared_ptr<turtle_multi_target_intefaces::srv::TargetSwitch::Request> request,
    //       std::shared_ptr<turtle_multi_target_intefaces::srv::TargetSwitch::Response> response) {
    // RCLCPP_INFO(this->get_logger(), "Got request to switch target");
    // current_target_id = (current_target_id + 1) % 3;
    // std_msgs::msg::UInt32 msg = std_msgs::msg::UInt32()
    // msg.data = current_target_id
    // target_id_pub->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Target switched! Current target id is: %ld", current_target_id);
    // response->new_target_id = current_target_id;
    // RCLCPP_INFO(this->get_logger(), "Request processed. Send back response: %ld", response->new_target_id);
    // }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner;
    rclcpp::TimerBase::SharedPtr spawn_timer, carrot_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr first_turtle_pose_sub, second_turtle_pose_sub, third_turtle_pose_sub;
    // rclcpp::Publisher<std_msgs::msg::UInt32> target_id_pub;
    // rclcpp::Service<turtle_multi_target_intefaces::srv::TargetSwitch>::SharedPtr target_switch_service;
    std::string first_name, second_name, third_name, first_carrot_name, second_carrot_name, static_name;
    bool spawning_service_ready, turtles_spawned;
    double radius, direction, slow_coef, switch_threshold;
    // size_t current_target_id;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TargetSwitcher>());
    }
    catch (const std::runtime_error &ex) {
        std::cout << ex.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
