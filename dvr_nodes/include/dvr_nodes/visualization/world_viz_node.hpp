#ifndef __DVR_NODES_VISUALIZATION_WORLD_VIZ_NODES__HPP__
#define __DVR_NODES_VISUALIZATION_WORLD_VIZ_NODES__HPP__

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace dvr {

class WorldVizNode : public rclcpp::Node {
public:
  explicit WorldVizNode(const rclcpp::NodeOptions& node_options);

private:
  struct WorldBoundary {
    geometry_msgs::msg::Point boundary_min;
    geometry_msgs::msg::Point boundary_max;
  };

  WorldBoundary world_boundary_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pub_world_boundary_;

  void VisualizeWorldBoundary();
};

}  // namespace dvr

#endif /* __DVR_NODES_VISUALIZATION_WORLD_VIZ_NODES__HPP__ */
