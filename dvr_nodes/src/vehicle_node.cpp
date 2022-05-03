#include "dvr_nodes/vehicle_node.hpp"

#include <random>

namespace dvr {

VehicleNode::VehicleNode(const rclcpp::NodeOptions& node_options)
    : Node("vehicle_node", node_options) {
  // Read in parameters
  id_    = this->declare_parameter<int>("uid", -1);
  speed_ = this->declare_parameter<double>("speed", 0.0);
  world_info_.boundary_min.x() =
      this->declare_parameter<double>("world.boundary.min.x");
  world_info_.boundary_min.y() =
      this->declare_parameter<double>("world.boundary.min.y");
  world_info_.boundary_min.z() =
      this->declare_parameter<double>("world.boundary.min.z");
  world_info_.boundary_max.x() =
      this->declare_parameter<double>("world.boundary.max.x");
  world_info_.boundary_max.y() =
      this->declare_parameter<double>("world.boundary.max.y");
  world_info_.boundary_max.z() =
      this->declare_parameter<double>("world.boundary.max.z");

  // Set initial pose
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<double> uniform_distribution{-1.0, 1.0};

  state_.pos = Eigen::Vector3d::NullaryExpr(
      [&]() { return uniform_distribution(generator); });
  state_.pos.z() = 0;
  RCLCPP_INFO(get_logger(), "Vehicle %d state: [%f, %f, %f]", id_,
              state_.pos.x(), state_.pos.y(), state_.pos.z());

  // Set initial velocity
  state_.vel = Eigen::Vector3d::NullaryExpr(
      [&]() { return uniform_distribution(generator); });
  state_.vel.z() = 0;
  state_.vel.normalize();

  // Create timer to update state
  state_update_rate_ = this->declare_parameter<double>("rate", 0.1);
  state_dt_          = 1.0 / state_update_rate_;
  auto viz_period_ns = rclcpp::Rate(state_update_rate_).period();
  timer_control_     = rclcpp::create_timer(this, get_clock(), viz_period_ns,
                                            std::bind(&VehicleNode::OnTimer, this));

  pub_state_ = this->create_publisher<dvr_msgs::msg::VehicleStateStamped>(
      "state", rclcpp::QoS{1});
}

void VehicleNode::OnTimer() {
  PublishCurrentState();
  KinematicForward();
  UpdateGoal();
}

void VehicleNode::PublishCurrentState() {
  dvr_msgs::msg::VehicleStateStamped msg;
  msg.header.stamp    = now();
  msg.header.frame_id = "map";
  msg.uid             = id_;
  msg.pose.position.x = state_.pos.x();
  msg.pose.position.y = state_.pos.y();
  msg.pose.position.z = state_.pos.z();
  pub_state_->publish(msg);
}

void VehicleNode::KinematicForward() {
  Eigen::Vector3d new_pos =
      state_.pos + speed_ * state_.vel.normalized() * state_dt_;

  // bounce back if hits the world boundary
  auto min_comp = new_pos.array() >= world_info_.boundary_min.array();
  auto max_comp = new_pos.array() <= world_info_.boundary_max.array();
  if (!min_comp.all()) {
    if (!min_comp.x()) state_.vel.x() *= -1;
    if (!min_comp.y()) state_.vel.y() *= -1;
    if (!min_comp.z()) state_.vel.z() *= -1;
  } else if (!max_comp.all()) {
    if (!max_comp.x()) state_.vel.x() *= -1;
    if (!max_comp.y()) state_.vel.y() *= -1;
    if (!max_comp.z()) state_.vel.z() *= -1;
  }

  state_.pos += speed_ * state_.vel.normalized() * state_dt_;
}

void VehicleNode::UpdateGoal() {}

}  // namespace dvr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dvr::VehicleNode)
