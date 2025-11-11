#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "cleaning_robot_interfaces/action/cleaning_task.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_cleaning_robot/visibility_control.h"


namespace action_cleaning_robot
{
class CleaningActionClient : public rclcpp::Node
{
public:
    using CleaningTask = cleaning_robot_interfaces::action::CleaningTask;
    using GoalHandleCleaningTask = rclcpp_action::ClientGoalHandle<CleaningTask>;

    explicit CleaningActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("cleaning_action_client", options) {
        this->client_ptr_ = rclcpp_action::create_client<CleaningTask>(
            this,
            "cleaning_robot");

        auto timer_callback_lambda = [this](){ return this->send_goal(); };
        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        timer_callback_lambda);
    }

    void send_goal() {
        this->timer_->cancel();
        rclcpp::Rate loop_rate(1);

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto send_goal_options = this->define_goal_option();
        
        auto goal_msg = CleaningTask::Goal();
        this->update_goal(goal_msg, "return_home", 0, 5.0, 5.5);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        // for (int i = 0; i < 15; i++) {
        //     loop_rate.sleep();
        // }

        this->update_goal(goal_msg, "clean_circle", 5, 5.0, 5.5);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        // for (int i = 0; i < 15; i++) {
        //     loop_rate.sleep();
        // }

        // this->update_goal(goal_msg, "clean_square", 5, 5.0, 5.5);
        // this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
private:
    rclcpp_action::Client<CleaningTask>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Client<CleaningTask>::SendGoalOptions define_goal_option() {
        auto send_goal_options = rclcpp_action::Client<CleaningTask>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleCleaningTask::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.feedback_callback = [this](
        GoalHandleCleaningTask::SharedPtr,
        const std::shared_ptr<const CleaningTask::Feedback> feedback) {
            std::stringstream ss;
            ss << "Processing [" << feedback->progress_percent << "%]:\n";
            ss << "\tCleaned points: " << feedback->current_cleaned_points << "\n";
            ss << "\tCurrent position: x = " << feedback->current_x << ", y = " << feedback->current_y << "\n";
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        };

        send_goal_options.result_callback = [this](const GoalHandleCleaningTask::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
            std::stringstream ss;
            ss << "Result received:\n";
            ss << "\tStatus: " << (result.result->success ? "Success\n" : "Fail\n");
            ss << "\tTotal cleaned points: " << result.result->cleaned_points << "\n";
            ss << "\tTotal distance: " << result.result->total_distance << "\n";
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        };
        return send_goal_options;
    }

    void update_goal(CleaningTask::Goal& goal_msg, std::string task_type, double area_size, double x, double y) {
        goal_msg.task_type = task_type;
        goal_msg.area_size = area_size;
        goal_msg.target_x = x;
        goal_msg.target_y = y;
    }
}; // class CleaningActionClient

} // namespace action_cleaning_robot

RCLCPP_COMPONENTS_REGISTER_NODE(action_cleaning_robot::CleaningActionClient)
