#include "dvr_nodes/vehicle_node.hpp"

namespace dvr {

VehicleNode::VehicleNode(const rclcpp::NodeOptions& node_options)
    : Node("vehicle_node", node_options) {
  id_ = this->declare_parameter<int>("uid", -1);
  RCLCPP_INFO(get_logger(), "Vehicle node %d", id_);
}

}  // namespace dvr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dvr::VehicleNode)
