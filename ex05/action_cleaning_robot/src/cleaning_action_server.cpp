#include <functional>
#include <memory>
#include <thread>
#include <math.h>

#include "cleaning_robot_interfaces/action/cleaning_task.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace cleaning_action
{

class CleaningActionServer : public rclcpp::Node
{
public:
  using CleaningTask = cleaning_robot_interfaces::action::CleaningTask;
  using GoalHandleCleaningTask = rclcpp_action::ServerGoalHandle<CleaningTask>;

  explicit CleaningActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("cleaning_action_server", options){
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<CleaningTask>(
      this, 
      "cleaning_task",
      std::bind(&CleaningActionServer::handle_goal, this, _1, _2),
      std::bind(&CleaningActionServer::handle_cancel, this, _1),
      std::bind(&CleaningActionServer::handle_accepted, this, _1));

    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
    this->pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&CleaningActionServer::pose_update, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "CleaningActionServer is ready for executing requests");
  }
private:
  rclcpp_action::Server<CleaningTask>::SharedPtr action_server_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  turtlesim::msg::Pose curr_p;
  double default_anglular_vel = 2 * 3.14;


  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
                                          std::shared_ptr<const CleaningTask::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request with task_type %s", goal->task_type.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCleaningTask> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCleaningTask> goal_handle){
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CleaningActionServer::execute, this, _1), goal_handle}.detach();
  }

  void pose_update(const turtlesim::msg::Pose & p){
    this->curr_p = p;
  }

  void wait_for_turtle_stop(rclcpp::Rate & waiting_rate){
    while (this->curr_p.linear_velocity != 0 || this->curr_p.angular_velocity != 0) {
      waiting_rate.sleep();
    }
  }

  double round_n(double x, int n){
    return round(x * pow(10, n)) / pow(10, n);
  }

  int clean_circle(double r, 
                   std::shared_ptr<CleaningTask::Feedback> feedback,
                   std::shared_ptr<CleaningTask::Result> result,
                   const std::shared_ptr<GoalHandleCleaningTask> goal_handle){
    rclcpp::Rate wait_pose_update_rate(1);
    wait_for_turtle_stop(wait_pose_update_rate);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = r;
    twist.linear.y = 0;
    twist.angular.z = 0;
    cmd_vel_publisher_->publish(twist);
    wait_pose_update_rate.sleep();
    wait_for_turtle_stop(wait_pose_update_rate);

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 1.57;
    cmd_vel_publisher_->publish(twist);
    wait_pose_update_rate.sleep();
    wait_for_turtle_stop(wait_pose_update_rate);
    
    twist.linear.y = 0;
    twist.angular.z = default_anglular_vel;
    double cleaned_points = 0;
    double cleaned_radius;
    double linear_vel = r * default_anglular_vel;
    while (((linear_vel / default_anglular_vel) > 0.1) && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->cleaned_points = cleaned_points;
        result->total_distance = cleaned_points + r;
        return -1;
      }

      twist.linear.x = linear_vel;
      cmd_vel_publisher_->publish(twist);
      wait_pose_update_rate.sleep();
      cleaned_radius = r - (linear_vel / default_anglular_vel);

      while (this->curr_p.linear_velocity != 0 || this->curr_p.angular_velocity != 0) {
        feedback->current_x = curr_p.x;
        feedback->current_y = curr_p.y;
        cleaned_points += r * default_anglular_vel;
        feedback->progress_percent = (cleaned_radius / r) * 100;
        feedback->current_cleaned_points = cleaned_points;
        goal_handle->publish_feedback(feedback);
        wait_pose_update_rate.sleep();
      }
      
      linear_vel -= default_anglular_vel * 0.05;
    }

    result->success = true;
    result->cleaned_points = cleaned_points;
    result->total_distance = cleaned_points + r;
    return 0;
  }

  double vec_len(double x, double y){
    return sqrt(x*x + y*y);
  }

  double angle_between_vectors(double x1, double y1, double x2, double y2){
    return acos((x1*x2 + y1*y2) / (vec_len(x1, y1) * vec_len(x2, y2)));
  }

  int return_home(double x, double y,
                  std::shared_ptr<CleaningTask::Feedback> feedback,
                  std::shared_ptr<CleaningTask::Result> result,
                  const std::shared_ptr<GoalHandleCleaningTask> goal_handle){
    rclcpp::Rate wait_pose_update_rate(1);
    wait_for_turtle_stop(wait_pose_update_rate);
    
    double goal_vec_x = round_n(x - curr_p.x, 3);
    double goal_vec_y = round_n(y - curr_p.y, 3);
    double goal_dist = vec_len(goal_vec_x, goal_vec_y);
    if (goal_dist <= 0.1) {
      result->success = true;
      result->cleaned_points = 0;
      result->total_distance = 0;
      return 0;
    }

    double turtle_vec_x = cos(curr_p.theta);
    double turtle_vec_y = sin(curr_p.theta);
    double alpha = angle_between_vectors(goal_vec_x, goal_vec_y, turtle_vec_x, turtle_vec_y);
    double direction = turtle_vec_x * goal_vec_y - turtle_vec_y * goal_vec_x;
    if (direction < 0) {
      alpha = -alpha;
    }

    // Поворот черепахи в сторону точки назначения
    geometry_msgs::msg::Twist twist;
    twist.linear.x = twist.linear.y = 0;
    twist.angular.z = alpha;
    cmd_vel_publisher_->publish(twist);
    wait_pose_update_rate.sleep();
    wait_for_turtle_stop(wait_pose_update_rate);
    
    twist.linear.y = 0;
    twist.angular.z = 0;
    double rest_dist = goal_dist;
    rclcpp::Rate check_turtle_pos_rate(3);
    while ((rest_dist > 0.1) && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->cleaned_points = 0;
        result->total_distance = goal_dist - rest_dist;
        return -1;
      }

      twist.linear.x = rest_dist;
      cmd_vel_publisher_->publish(twist);
      check_turtle_pos_rate.sleep();

      while (this->curr_p.linear_velocity != 0 || this->curr_p.angular_velocity != 0) {
        feedback->current_x = curr_p.x;
        feedback->current_y = curr_p.y;
        rest_dist = vec_len(x - curr_p.x, y - curr_p.y);
        feedback->progress_percent = ((goal_dist - rest_dist) / goal_dist) * 100;
        feedback->current_cleaned_points = 0;
        goal_handle->publish_feedback(feedback);
        check_turtle_pos_rate.sleep();
      }
      rest_dist = vec_len(x - curr_p.x, y - curr_p.y);
    }

    result->success = true;
    result->cleaned_points = 0;
    result->total_distance = goal_dist - rest_dist;
    return 0;
  }

  void execute(const std::shared_ptr<GoalHandleCleaningTask> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CleaningTask::Feedback>();
    auto result = std::make_shared<CleaningTask::Result>();

    if (goal->task_type != "clean_circle" && goal->task_type != "return_home"){
      result->success = false;
      result->cleaned_points = 0;
      result->total_distance = 0;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Goal has unexpected task_type: %s", goal->task_type.c_str());
    } else {
      int return_code;
      if (goal->task_type == "clean_circle") {
        return_code = clean_circle(goal->area_size, feedback, result, goal_handle);
      } else {
        return_code = return_home(goal->target_x, goal->target_y, feedback, result, goal_handle);
      }

      switch (return_code){
        case -1:{
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          break;
        }
        case 0:{
          if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          }
          break;
        }
        default: break;
      }
    }
  }
};  // class CleaningActionServer

}  // namespace cleaning_action

RCLCPP_COMPONENTS_REGISTER_NODE(cleaning_action::CleaningActionServer)