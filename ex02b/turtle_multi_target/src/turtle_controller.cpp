#include <cmath>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtle_multi_target_intefaces/msg/target_info.hpp"
// #include "turtle_multi_target_intefaces/srv/target_switch.hpp"
// #include "std_msgs/msg/uint32.hpp"


bool wait_for_turtle(const std::string & name, std::shared_ptr<rclcpp::Node> node)
{
    auto logger = rclcpp::get_logger("logger");

    std::string topic = "/" + name + "/pose";
    std::cout << "Waiting for topic: " << topic << std::endl;

    // Wait until the topic appears in the ROS graph
    while (rclcpp::ok()) {
        auto topics = node->get_topic_names_and_types();
        if (topics.find(topic) != topics.end()) {
            std::cout << "Turtle " << name << " detected!" << std::endl;
            return true;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}


class TurtleController: public rclcpp::Node
{
public:
    std::string first_name, second_name, third_name, first_carrot_name, second_carrot_name, static_name;

    TurtleController(): Node("turtle_controller"), cx(5.0), cy(5.0), ctheta(0.0) {
        // Read run parameters
        first_name = this->declare_parameter<std::string>("first_turtle_name", "turtle1");
        second_name = this->declare_parameter<std::string>("second_turtle_name", "turtle2");
        third_name = this->declare_parameter<std::string>("third_turtle_name", "turtle3");
        first_carrot_name = this->declare_parameter<std::string>("first_carrot_name", "carrot1");
        second_carrot_name = this->declare_parameter<std::string>("second_carrot_name", "carrot2");
        static_name = this->declare_parameter<std::string>("static_target_name", "static_target");

        // Get switch_threshold from target_switcher node
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "target_switcher");
        if (!param_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Parameter Service not available after waiting");
            rclcpp::shutdown();
            return;
        }
        try {
            switch_threshold = param_client->get_parameter<double>(std::string("switch_threshold"));
            RCLCPP_INFO(this->get_logger(), 
                "Successfully got parameter from target_switcher: %f", switch_threshold);
        } 
        catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get parameter: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Targets init
        targets = {first_carrot_name, second_carrot_name, static_name};
        current_target_id = 0;

        // Tf2 buffer and listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Turtle2 velocity publisher
        std::string topic_name = std::string("/") + second_name + std::string("/cmd_vel");
        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(topic_name.c_str(), 1);

        // Current turget publisher and timer for target info publishing
        target_pub = this->create_publisher<turtle_multi_target_intefaces::msg::TargetInfo>("/current_target", 5);
        target_info_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TurtleController::publish_current_target, this));

        topic_name = std::string("/") + second_name + std::string("/pose");
        turtle_pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10, std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));

        // Timer for turtle2 directing
        turtle_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TurtleController::turtle_timer_callback, this));

        // Timer to check current target
        target_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleController::target_coords_update, this));

        // Thread to read from keyboard
        is_running = true;
        keyboard_thread = std::thread(&TurtleController::keyboard_loop, this);

        RCLCPP_INFO(this->get_logger(), "TurtleController node initialized");
    }

    ~TurtleController() {
        is_running = false;
        if (keyboard_thread.joinable()) {
            keyboard_thread.join();
        }
        // restore_terminal();
        RCLCPP_INFO(this->get_logger(), "TurtleController node destroyed");
    }

private:
    void pose_callback(const std::shared_ptr<turtlesim::msg::Pose> msg) {
        cx = msg->x;
        cy = msg->y;
        ctheta = msg->theta;
    }

    void setup_terminal() {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        raw_termios_ = orig_termios_;
        raw_termios_.c_lflag &= ~(ICANON | ECHO); // Raw mode: no buffering, no echo
        tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios_);
    }

    void restore_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    void handle_ctrl_c()
    {
        is_running = false;
        rclcpp::shutdown();
    }

    // target_frame is turtle, source_frame is target
    geometry_msgs::msg::TransformStamped get_transform(std::string source_frame, std::string target_frame) {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
        }
        return t;
    }

    void turtle_timer_callback() {
            geometry_msgs::msg::Twist msg;

            double trans_x = target_x - cx;
            double trans_y = target_y - cy;
            msg.angular.z = atan2(trans_y, trans_x);
            double dist = sqrt(pow(trans_x, 2) + pow(trans_y, 2));
            msg.linear.x = dist > 4.0 ? 0.8 * dist : 1.2 * dist;

            vel_pub->publish(msg);
    }

    // void send_switch_request() {
    //     std::lock_guard<std::mutex> lock(mut);
    //     auto request = std::make_shared<turtle_multi_target_intefaces::srv::TargetSwitch>();
    //     auto response_received_callback = [this](rclcpp::Client<turtle_multi_target_intefaces::srv::TargetSwitch>::SharedFuture future) {
    //         auto result = future.get();
    //         current_target_id = result->new_target_id;
    //     };
    //     auto result = spawner->async_send_request(request, response_received_callback);
    // }

    void switch_target() {
        current_target_id = (current_target_id + 1) % 3;
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Target switched. Current target id is: %d", (int)current_target_id);
    }

    void publish_current_target() {
        turtle_multi_target_intefaces::msg::TargetInfo msg = turtle_multi_target_intefaces::msg::TargetInfo();
        msg.target_name = targets[current_target_id];
        msg.target_x = target_x;
        msg.target_y = target_y;
        msg.distance_to_target = target_dist;
        target_pub->publish(msg);
    }

    void target_coords_update() {
        geometry_msgs::msg::TransformStamped t = this->get_transform(targets[current_target_id], second_name);
        target_x = cx + t.transform.translation.x;
        target_y = cy + t.transform.translation.y;
        target_dist = sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
        if (target_dist < switch_threshold) {
            {
                std::lock_guard<std::mutex> lock(mut);
                switch_target();
            }
            publish_current_target();
        }
    }

    // void target_id_callback(const std::shared_ptr<std_msgs::msg::UInt32> msg) {
    //     current_target_id = msg->data;
    //     geometry_msgs::msg::TransformStamped t = this->get_transform(targets[current_target_id], second_name);
    //     target_x = cx + t.transform.translation.x;
    //     target_y = cy + t.transform.translation.y;
    //     target_dist = sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
    //     if (target_dist < switch_threshold) {
    //         this->send_switch_request();
    //     }
    // }

    void keyboard_loop() {
        setup_terminal();
        char c;
        ssize_t n;

        while (is_running) {
            n = read(STDIN_FILENO, &c, 1);

            if (n > 0) {
                if (c == 3) { // ASCII code 3 = Ctrl+C
                    handle_ctrl_c();
                    break;
                }
                if (c == 'n' || c == 'N') {
                    std::lock_guard<std::mutex> lock(mut);
                    switch_target();
                    // this->send_switch_request();
                }
            }
            usleep(1000);
        }
        restore_terminal();
    }

    std::mutex mut;
    rclcpp::TimerBase::SharedPtr turtle_timer, target_timer, target_info_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Publisher<turtle_multi_target_intefaces::msg::TargetInfo>::SharedPtr target_pub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub;
    // rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr target_id_sub;
    // rclcpp::Client<turtle_multi_target_intefaces::srv::TargetSwitch>::SharedPtr switcher;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    double cx, cy, ctheta, target_x, target_y, target_dist;
    double switch_threshold;
    std::vector<std::string> targets;
    std::atomic<size_t> current_target_id;
    // For keyboard handling
    std::atomic<bool> is_running;
    std::thread keyboard_thread;
    struct termios orig_termios_;
    struct termios raw_termios_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> dummy_node = std::make_shared<rclcpp::Node>("dummy_node");
    std::string name = dummy_node->declare_parameter<std::string>("second_turtle_name", "turtle2");
    try {
        if (!wait_for_turtle(name, dummy_node)) {
            std::cout << "Couldn't wait for the turtle " << name << std::endl;
            return 1;
        }
        else {
            std::cout << "Succesfully waited for the turtle " << name << std::endl;
        }
    }
    catch (const std::exception &ex) {
        std::cout << "There is error while waiting for turtle " << name << ": " << ex.what() << std::endl;
    }

    std::shared_ptr<TurtleController> node = std::make_shared<TurtleController>();
    try {
        rclcpp::spin(node);
    }
    catch (const std::runtime_error &ex) {
        std::cout << ex.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}