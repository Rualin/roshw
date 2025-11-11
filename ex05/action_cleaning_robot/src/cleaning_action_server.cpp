#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "cleaning_robot_interfaces/action/cleaning_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_cleaning_robot/visibility_control.h"


#define PI 3.14159265

namespace action_cleaning_robot
{
class CleaningActionServer : public rclcpp::Node
{
public:
    using CleaningTask = cleaning_robot_interfaces::action::CleaningTask;
    using GoalHandleCleaningTask = rclcpp_action::ServerGoalHandle<CleaningTask>;

    ACTION_CLEANING_ROBOT_PUBLIC
    explicit CleaningActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("cleaning_action_server", options), is_executing(false) {
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&CleaningActionServer::turtle_pose_callback, this, std::placeholders::_1)
        );

        auto handle_accepted = [this](const std::shared_ptr<GoalHandleCleaningTask> goal_handle) {
            is_executing = true;
            auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
            std::thread{execute_in_thread}.detach();
        };

        this->action_server_ = rclcpp_action::create_server<CleaningTask>(
            this,
            "cleaning_robot",
            std::bind(&CleaningActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CleaningActionServer::handle_cancel, this, std::placeholders::_1),
            handle_accepted);
        
        RCLCPP_INFO(this->get_logger(), "Server created. Ready to accomplish actions...");
    }


private:
    bool is_executing;
    rclcpp_action::Server<CleaningTask>::SharedPtr action_server_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    float cx;
    float cy;
    float ctheta;

    float _calc_distance(float x1, float y1, float x2, float y2) {
        return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    float _calc_relative_angle(float x1, float y1, float x2, float y2) {
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

    bool _is_far(float a, float b, float eps) {
        return (abs(a - b) > eps);
    }

    void turtle_pose_callback(turtlesim::msg::Pose::UniquePtr msg) {
        cx = msg->x;
        cy = msg->y;
        ctheta = msg->theta;
    }

    void move_to_goal(float px, float py, float ptheta) {
        geometry_msgs::msg::Twist vel_msg = geometry_msgs::msg::Twist();
        geometry_msgs::msg::Twist stop_vel_msg = geometry_msgs::msg::Twist();
        rclcpp::Rate rate(0.7);
        float relative_angle, dist;

        // if (!(this->_is_far(cx, px, 0.1) || this->_is_far(cy, py, 0.1))) {
            if (this->_is_far(ctheta, ptheta, 0.01)) {
                vel_msg.angular.z = (ptheta - ctheta);
                this->vel_publisher->publish(vel_msg);
                // RCLCPP_INFO(this->get_logger(), "Rotating turtle to final theta...");
                rate.sleep();
                RCLCPP_INFO(this->get_logger(), "\tRotated. Current theta = %f", ctheta);
            }
            else {
                RCLCPP_WARN(this->get_logger(), "All coordinates are equal to goal!");
            }
            return;
        // }

        relative_angle = this->_calc_relative_angle(cx, cy, px, py);

        if (this->_is_far(ctheta, relative_angle, 0.01)) {
            vel_msg.angular.z = (relative_angle - ctheta);
            this->vel_publisher->publish(vel_msg);
            // RCLCPP_INFO(this->get_logger(), "Rotating turtle to goal...");
            rate.sleep();
            this->vel_publisher->publish(stop_vel_msg);
            RCLCPP_INFO(this->get_logger(), "\tRotated. Current theta = %f", ctheta);
        }

        // if (this->_is_far(cx, px, 0.1) && this->_is_far(cy, py, 0.1)) {
            vel_msg = geometry_msgs::msg::Twist();
            dist = _calc_distance(cx, cy, px, py);
            vel_msg.linear.x = dist;
            this->vel_publisher->publish(vel_msg);
            // RCLCPP_INFO(this->get_logger(), "Moving turtle to goal...");
            rate.sleep();
            this->vel_publisher->publish(stop_vel_msg);
            RCLCPP_INFO(this->get_logger(), "\tMoved. Current: x = %f, y = %f", cx, cy);
        // }

        if (this->_is_far(ctheta, ptheta, 0.01)) {
            vel_msg = geometry_msgs::msg::Twist();
            vel_msg.angular.z = (ptheta - relative_angle);
            this->vel_publisher->publish(vel_msg);
            // RCLCPP_INFO(this->get_logger(), "Rotating turtle to final theta...");
            rate.sleep();
            this->vel_publisher->publish(stop_vel_msg);
            RCLCPP_INFO(this->get_logger(), "\tRotated. Delta theta = %f", ptheta - relative_angle);
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CleaningTask::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->task_type.c_str());
        (void)uuid;
        if (is_executing) {
            RCLCPP_ERROR(this->get_logger(), "Reject goal for a reason: another goal is executing");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if (goal->task_type == std::string("clean_square")) {
            if (goal->area_size <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Reject goal for a reason: incorrect area_size");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        else if (goal->task_type == std::string("clean_circle")) {
            RCLCPP_ERROR(this->get_logger(), "Reject goal for a reason: goal \'clean_circle\' is not supported");
            return rclcpp_action::GoalResponse::REJECT;
        }
        else if (goal->task_type == std::string("return_home")) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Reject goal for a reason: unknown goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCleaningTask> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        if (is_executing) {
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        RCLCPP_ERROR(this->get_logger(), "Reject cancel for a reason: there is no executing task");
        return rclcpp_action::CancelResponse::REJECT;
    }

    void _update_feedback(
        CleaningTask::Feedback::SharedPtr& feedback, int progr, int cnt_p, double curr_x, double curr_y) {
        feedback->progress_percent = progr;
        feedback->current_cleaned_points = cnt_p;
        feedback->current_x = curr_x;
        feedback->current_y = curr_y;
    }

    void execute(const std::shared_ptr<GoalHandleCleaningTask> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CleaningTask::Feedback>();
        auto result = std::make_shared<CleaningTask::Result>();
        float total_dist = 0, lx, ly;
        if (goal->task_type == std::string("return_home")) {
            RCLCPP_INFO(this->get_logger(), "Start returning home");
            float px = goal->target_x;
            float py = goal->target_y;
            float gx, gy;
            float dist = _calc_distance(cx, cy, px, py);
            float relative_angle = this->_calc_relative_angle(cx, cy, px, py);
            this->move_to_goal(cx, cy, relative_angle);
            this->_update_feedback(feedback, 20, 0, cx, cy);
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Stage 1: x = %f y = %f t = %f", cx, cy, ctheta);
            for (int i = 1; i < 5; i++) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->cleaned_points = 0;
                    result->total_distance = (double)total_dist;
                    goal_handle->canceled(result);
                    is_executing = false;
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                lx = cx;
                ly = cy;
                gx = cx + (dist / 4.0) * cosf(relative_angle);
                gy = cy + (dist / 4.0) * sinf(relative_angle);
                this->move_to_goal(gx, gy, relative_angle);
                this->_update_feedback(feedback, 20 + 20 * i, 0, cx, cy);
                goal_handle->publish_feedback(feedback);
                total_dist += this->_calc_distance(lx, ly, cx, cy);
                loop_rate.sleep();
                RCLCPP_INFO(this->get_logger(), "Stage %d: x = %f y = %f t = %f", i + 1, cx, cy, ctheta);
            }
            if (rclcpp::ok()) {
                if (!(this->_is_far(cx, px, 0.1) || this->_is_far(cy, py, 0.1))) {
                    result->success = true;
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                }
                else {
                    result->success = false;
                    RCLCPP_INFO(this->get_logger(), "Goal failed");
                }
                result->cleaned_points = 0;
                result->total_distance = (double)total_dist;
                goal_handle->succeed(result);
            }
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Start cleaning square");
            float area_size = (float)(goal->area_size);
            float lux = cx, luy = cy; // left up
            float rbx = lux + area_size; // right bottom
            float delta = 0.5;
            int n_lines = ceilf(area_size / delta);
            int curr_line = 0;
            lx = cx;
            ly = cy;
            this->move_to_goal(rbx, luy, -PI / 2.0);
            total_dist += this->_calc_distance(lx, ly, cx, cy);
            curr_line++;
            this->_update_feedback(
                feedback, curr_line * 100 / n_lines,
                (int)((float)curr_line * (float)n_lines * delta),
                cx, cy
            );
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
            bool is_inv;
            for(; curr_line < n_lines; curr_line++) {
                is_inv = curr_line % 2;
                lx = cx;
                ly = cy;
                this->move_to_goal(
                    cx, cy + delta,
                    is_inv ? PI - 0.000001 : 0
                );
                total_dist += this->_calc_distance(lx, ly, cx, cy);
                lx = cx;
                ly = cy;
                this->move_to_goal(is_inv ? lux : rbx, cy, -PI / 2.0);
                total_dist += this->_calc_distance(lx, ly, cx, cy);
                this->_update_feedback(
                    feedback, curr_line * 100 / n_lines,
                    (int)((float)curr_line * (float)n_lines * delta),
                    cx, cy
                );
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }
            if (rclcpp::ok()) {
                result->success = true;
                result->cleaned_points = (int)((float)curr_line * (float)n_lines * delta);
                result->total_distance = (double)total_dist;
                goal_handle->succeed(result);
            }
        }
        is_executing = false;
    }
}; // class CleaningActionServer

} // namespace action_cleaning_robot

RCLCPP_COMPONENTS_REGISTER_NODE(action_cleaning_robot::CleaningActionServer)
