#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

class ApproachVelocityController : public rclcpp::Node {
public:
  ApproachVelocityController() : Node("approach_velocity_controller") {
    this->declare_parameter("slow_down_distance", 1.0);
    this->declare_parameter("min_speed", 0.3);
    this->declare_parameter("normal_speed", 1.0);
    this->declare_parameter("robot_frame", std::string("base_footprint"));
    this->declare_parameter("timeout_sec", 1.0);

    slow_down_distance_ = this->get_parameter("slow_down_distance").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    normal_speed_ = this->get_parameter("normal_speed").as_double();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10,
        std::bind(&ApproachVelocityController::planCallback, this, std::placeholders::_1));

    param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
        "/velocity_smoother/set_parameters");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ApproachVelocityController::controlLoop, this));
  }

private:
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) return;
    last_plan_time_ = this->now();
    auto &last_pose = msg->poses.back();
    goal_x_ = last_pose.pose.position.x;
    goal_y_ = last_pose.pose.position.y;
    goal_frame_ = msg->header.frame_id;
  }

  void controlLoop() {
    bool navigating = (this->now() - last_plan_time_).seconds() < timeout_sec_;

    if (!navigating) {
      if (last_set_speed_ != normal_speed_) {
        setMaxVelocity(normal_speed_);
        last_set_speed_ = normal_speed_;
      }
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(goal_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }

    double robot_x = tf.transform.translation.x;
    double robot_y = tf.transform.translation.y;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "dist_to_goal=%.2f, speed=%.2f", std::sqrt((goal_x_-robot_x)*(goal_x_-robot_x)+(goal_y_-robot_y)*(goal_y_-robot_y)), last_set_speed_);
    double dx = goal_x_ - robot_x;
    double dy = goal_y_ - robot_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double target_speed = (dist < slow_down_distance_) ? min_speed_ : normal_speed_;

    if (std::abs(target_speed - last_set_speed_) < 0.05) return;

    setMaxVelocity(target_speed);
    last_set_speed_ = target_speed;
  }

  void setMaxVelocity(double speed) {
    if (!param_client_->wait_for_service(std::chrono::milliseconds(50))) return;

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name = "max_velocity";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    param.value.double_array_value = {speed, speed, 0.6};
    request->parameters.push_back(param);

    param_client_->async_send_request(request);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double goal_x_ = 0.0, goal_y_ = 0.0;
  std::string goal_frame_;
  std::string robot_frame_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  double timeout_sec_ = 1.0;
  double slow_down_distance_ = 1.0;
  double min_speed_ = 0.3;
  double normal_speed_ = 1.0;
  double last_set_speed_ = -1.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachVelocityController>());
  rclcpp::shutdown();
  return 0;
}
