#ifndef __DVR_NODES_VEHICLE_NODE__HPP__
#define __DVR_NODES_VEHICLE_NODE__HPP__

#include <Eigen/Dense>
#include <chrono>
#include <dvr_msgs/msg/vehicle_state_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dvr {

struct VehicleState {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

struct WorldInfo {
  Eigen::Vector3d boundary_min;
  Eigen::Vector3d boundary_max;
};

class VehicleNode : public rclcpp::Node {
public:
  explicit VehicleNode(const rclcpp::NodeOptions& node_options);

private:
  int id_;

  VehicleState state_;
  double speed_;
  WorldInfo world_info_;

  double state_update_rate_;  // in hz
  double state_dt_;           // = 1 / state_update_rate
  rclcpp::TimerBase::SharedPtr timer_control_;

  rclcpp::Publisher<dvr_msgs::msg::VehicleStateStamped>::SharedPtr pub_state_;

  /**
   * @brief Callback function upon each timer ticks
   *
   */
  void OnTimer();
  /**
   * @brief Publishes current vehicle state
   *
   */
  void PublishCurrentState();
  /**
   * @brief Forward vehicle's state according to kinematic model
   *
   */
  void KinematicForward();
  /**
   * @brief Updates vehicle's goal
   *
   */
  void UpdateGoal();
};

}  // namespace dvr

#endif /* __DVR_NODES_VEHICLE_NODE__HPP__ */
