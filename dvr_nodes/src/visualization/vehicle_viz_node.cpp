#include <dvr_nodes/visualization/vehicle_viz_node.hpp>

namespace dvr {

VehicleVizNode::VehicleVizNode(const rclcpp::NodeOptions& node_options)
    : Node("vehicle_visualization_node", node_options) {
  using std::placeholders::_1;

  num_vehicles_ = this->declare_parameter<int>("num_vehicles", 0);
  vehicle_states_.positions.resize(num_vehicles_);
  for (auto& pos : vehicle_states_.positions) {
    pos = Eigen::Vector3d::Zero();
  }

  state_viz_rate_ = this->declare_parameter<double>("visualization.rate", 0.1);
  state_viz_dt_   = 1.0 / state_viz_rate_;

  auto viz_period_ns = rclcpp::Rate(state_viz_rate_).period();
  timer_control_ =
      rclcpp::create_timer(this, get_clock(), viz_period_ns,
                           std::bind(&VehicleVizNode::OnTimer, this));

  sub_vehicle_state_ =
      this->create_subscription<dvr_msgs::msg::VehicleStateStamped>(
          "state", rclcpp::QoS{1},
          std::bind(&VehicleVizNode::OnVehicleState, this, _1));
  pub_state_viz_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "position_viz", rclcpp::QoS{1});
}

void VehicleVizNode::OnTimer() {
  std_msgs::msg::ColorRGBA blue{};
  blue.r = 0.0;
  blue.g = 0.0;
  blue.b = 0.999;
  blue.a = 0.999;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = "map";
  marker.header.stamp       = now();
  marker.ns                 = "vehicle_state";
  marker.frame_locked       = true;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.color              = blue;
  marker.id                 = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.05;
  marker.scale.y            = 0.05;
  marker.scale.z            = 0.05;

  for (const auto& pos : vehicle_states_.positions) {
    geometry_msgs::msg::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    marker.points.push_back(p);
  }

  pub_state_viz_->publish(marker);
}

void VehicleVizNode::OnVehicleState(
    const dvr_msgs::msg::VehicleStateStamped::ConstSharedPtr state_msg) {
  int vid                            = state_msg->uid;  // vehicle id
  vehicle_states_.positions[vid].x() = state_msg->pose.position.x;
  vehicle_states_.positions[vid].y() = state_msg->pose.position.y;
  vehicle_states_.positions[vid].z() = state_msg->pose.position.z;
}

}  // namespace dvr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dvr::VehicleVizNode)
