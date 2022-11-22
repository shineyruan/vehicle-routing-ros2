#include "dvr_nodes/visualization/world_viz_node.hpp"

namespace dvr {

WorldVizNode::WorldVizNode(const rclcpp::NodeOptions& node_options)
    : Node("world_visualization_node", node_options) {
  world_boundary_.boundary_min.x =
      this->declare_parameter<double>("world.boundary.min.x");
  world_boundary_.boundary_min.y =
      this->declare_parameter<double>("world.boundary.min.y");
  world_boundary_.boundary_min.z =
      this->declare_parameter<double>("world.boundary.min.z");
  world_boundary_.boundary_max.x =
      this->declare_parameter<double>("world.boundary.max.x");
  world_boundary_.boundary_max.y =
      this->declare_parameter<double>("world.boundary.max.y");
  world_boundary_.boundary_max.z =
      this->declare_parameter<double>("world.boundary.max.z");

  pub_world_boundary_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "world_viz", rclcpp::QoS{1}.transient_local());

  VisualizeWorldBoundary();
}

void WorldVizNode::VisualizeWorldBoundary() {
  std_msgs::msg::ColorRGBA black{};
  black.r = 0.01;
  black.g = 0.01;
  black.b = 0.01;
  black.a = 0.999;

  visualization_msgs::msg::Marker boundary_viz;
  boundary_viz.header.frame_id    = "map";
  boundary_viz.header.stamp       = now();
  boundary_viz.ns                 = "world_boundary";
  boundary_viz.frame_locked       = false;
  boundary_viz.action             = visualization_msgs::msg::Marker::ADD;
  boundary_viz.type               = visualization_msgs::msg::Marker::LINE_STRIP;
  boundary_viz.color              = black;
  boundary_viz.id                 = 0;
  boundary_viz.pose.orientation.x = 0.0;
  boundary_viz.pose.orientation.y = 0.0;
  boundary_viz.pose.orientation.z = 0.0;
  boundary_viz.pose.orientation.w = 1.0;
  boundary_viz.scale.x            = 0.01;
  boundary_viz.scale.y            = 0.0;
  boundary_viz.scale.z            = 0.0;

  // lower left corner
  boundary_viz.points.push_back(world_boundary_.boundary_min);

  // lower right corner
  geometry_msgs::msg::Point p;
  p.x = world_boundary_.boundary_max.x;
  p.y = world_boundary_.boundary_min.y;
  p.z = world_boundary_.boundary_min.z;
  boundary_viz.points.push_back(p);

  // upper right corner
  p.x = world_boundary_.boundary_max.x;
  p.y = world_boundary_.boundary_max.y;
  p.z = world_boundary_.boundary_min.z;
  boundary_viz.points.push_back(p);

  // upper left corner
  p.x = world_boundary_.boundary_min.x;
  p.y = world_boundary_.boundary_max.y;
  p.z = world_boundary_.boundary_min.z;
  boundary_viz.points.push_back(p);

  // lower left corner
  p.x = world_boundary_.boundary_min.x;
  p.y = world_boundary_.boundary_min.y;
  p.z = world_boundary_.boundary_min.z;
  boundary_viz.points.push_back(p);

  pub_world_boundary_->publish(boundary_viz);
}

}  // namespace dvr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dvr::WorldVizNode)
