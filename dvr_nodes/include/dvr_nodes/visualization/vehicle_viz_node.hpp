#ifndef __DVR_NODES_VISUALIZATION_VEHICLE_VIZ_NODE__HPP__
#define __DVR_NODES_VISUALIZATION_VEHICLE_VIZ_NODE__HPP__

#include <Eigen/Dense>
#include <dvr_msgs/msg/vehicle_state_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace dvr {

struct VehicleStates {
  std::vector<Eigen::Vector3d> positions;
};

class VehicleVizNode : public rclcpp::Node {
public:
  explicit VehicleVizNode(const rclcpp::NodeOptions& node_options);

private:
  int num_vehicles_;
  VehicleStates vehicle_states_;

  double state_viz_rate_;  // in hz
  double state_viz_dt_;    // = 1/state_viz_rate
  rclcpp::TimerBase::SharedPtr timer_control_;

  rclcpp::Subscription<dvr_msgs::msg::VehicleStateStamped>::SharedPtr
      sub_vehicle_state_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_state_viz_;

  /**
   * @brief Publishes visualization markers upon tick
   *
   */
  void OnTimer();
  /**
   * @brief Subscribes to incoming vehicle states
   *
   * @param state_msg
   */
  void OnVehicleState(
      const dvr_msgs::msg::VehicleStateStamped::ConstSharedPtr state_msg);
};

}  // namespace dvr

#endif /* __DVR_NODES_VISUALIZATION_VEHICLE_VIZ_NODE__HPP__ */
