#include <functional>
#include <future>
#include <memory>
#include <string>

#include "cleaning_robot_interfaces/action/cleaning_task.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace cleaning_action
{

class CleaningActionClient : public rclcpp::Node
{
public:
  using CleaningTask = cleaning_robot_interfaces::action::CleaningTask;
  using GoalHandleCleaningTask = rclcpp_action::ClientGoalHandle<CleaningTask>;

  explicit CleaningActionClient(const rclcpp::NodeOptions & options): Node("cleaning_action_client", options){
    this->client_ptr_ = rclcpp_action::create_client<CleaningTask>(this, "cleaning_task");
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CleaningActionClient::cleaning_request, this));
  }

  void cleaning_request(){
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = CleaningTask::Goal();
    goal_msg.task_type = "clean_circle";
    goal_msg.area_size = 3;

    RCLCPP_INFO(this->get_logger(), "Sending cleaning request");

    auto send_goal_options = rclcpp_action::Client<CleaningTask>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&CleaningActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&CleaningActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&CleaningActionClient::cleaning_result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void returning_home_request(){
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = CleaningTask::Goal();
    goal_msg.task_type = "return_home";
    goal_msg.target_x = 5;
    goal_msg.target_y = 5;

    RCLCPP_INFO(this->get_logger(), "Sending return_home request");

    auto send_goal_options = rclcpp_action::Client<CleaningTask>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&CleaningActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&CleaningActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&CleaningActionClient::returning_home_result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<CleaningTask>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleCleaningTask::SharedPtr & goal_handle){
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleCleaningTask::SharedPtr,
                         const std::shared_ptr<const CleaningTask::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Progress: %d%%, Cleaned points: %d, Pos: (%.3f, %.3f)",
                feedback->progress_percent, feedback->current_cleaned_points,
                feedback->current_x, feedback->current_y);
  }

  void cleaning_result_callback(const GoalHandleCleaningTask::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Cleaning was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Cleaning was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code during cleaning");
        return;
    }
    
    if (result.result->success) {
        RCLCPP_INFO(this->get_logger(), "Cleaning was completed successfully. Cleaned points: %d, Total distance: %.3f",
                    result.result->cleaned_points, result.result->total_distance);
        returning_home_request();
    } else {
        RCLCPP_INFO(this->get_logger(), "Cleaning was failed. Cleaned points: %d, Total distance: %.3f",
                    result.result->cleaned_points, result.result->total_distance);
    }
  }

  void returning_home_result_callback(const GoalHandleCleaningTask::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Returning home was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Returning home was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code during returning home");
        return;
    }
    
    if (result.result->success) {
        RCLCPP_INFO(this->get_logger(), "Returning home was completed successfully. Total distance: %.3f",
                    result.result->total_distance);
    } else {
        RCLCPP_INFO(this->get_logger(), "Returning home was failed. Total distance: %.3f",
                    result.result->total_distance);
    }
    rclcpp::shutdown();
  }
};  // class CleaningActionClient

}  // namespace cleaning_action

RCLCPP_COMPONENTS_REGISTER_NODE(cleaning_action::CleaningActionClient)