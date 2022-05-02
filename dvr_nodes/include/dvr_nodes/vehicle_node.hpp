#ifndef __DVR_NODES_VEHICLE_NODE__HPP__
#define __DVR_NODES_VEHICLE_NODE__HPP__

#include <rclcpp/rclcpp.hpp>

namespace dvr {

class VehicleNode : public rclcpp::Node {
public:
  explicit VehicleNode();

private:
  int id_;
};

}  // namespace dvr

#endif /* __DVR_NODES_VEHICLE_NODE__HPP__ */
