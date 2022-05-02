#include "dvr_nodes/vehicle_node.hpp"

namespace dvr {

VehicleNode::VehicleNode() : Node("vehicle_node") {
  id_ = this->declare_parameter<int>("uid", -1);
  RCLCPP_INFO(get_logger(), "Vehicle node %d", id_);
}

}  // namespace dvr

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dvr::VehicleNode>());
  rclcpp::shutdown();
  return 0;
}
