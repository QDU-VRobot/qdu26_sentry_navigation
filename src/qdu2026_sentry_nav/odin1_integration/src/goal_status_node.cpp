#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <std_msgs/msg/int32.hpp>

class GoalStatusNode : public rclcpp::Node {
public:
  GoalStatusNode() : Node("goal_status_node"), last_published_(-1) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>("/goal_arrive", 10);

    sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10,
        std::bind(&GoalStatusNode::statusCallback, this, std::placeholders::_1));
  }

private:
  void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
    if (msg->status_list.empty()) return;

    auto &latest = msg->status_list.back();
    int32_t value;

    switch (latest.status) {
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        value = 1;
        break;
      case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        value = 0;
        break;
      default:
        value = 2;
        break;
    }

    if (value != last_published_) {
      std_msgs::msg::Int32 out;
      out.data = value;
      pub_->publish(out);
      last_published_ = value;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_;
  int32_t last_published_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalStatusNode>());
  rclcpp::shutdown();
  return 0;
}
