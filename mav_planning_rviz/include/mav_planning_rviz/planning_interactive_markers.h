#ifndef MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_
#define MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_

#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mav_planning_rviz {

class PlanningInteractiveMarkers {
 public:
  typedef std::function<void(const geometry_msgs::msg::Pose& pose)> PoseUpdatedFunctionType;

  PlanningInteractiveMarkers(rclcpp::Node::SharedPtr node);

  ~PlanningInteractiveMarkers();

  void setFrameId(const std::string& frame_id);
  // Bind callback for whenever pose updates.

  void setPoseUpdatedCallback(const PoseUpdatedFunctionType& function) { pose_updated_function_ = function; }

  void initialize();

  // Functions to interface with the set_pose marker:
  void enableSetPoseMarker(const geometry_msgs::msg::Pose& pose);
  void disableSetPoseMarker();
  void setPose(const geometry_msgs::msg::Pose& pose);

  // Functions to interact with markers from the marker map (no controls):
  void enableMarker(const std::string& id, const geometry_msgs::msg::Pose& pose);
  void updateMarkerPose(const std::string& id, const geometry_msgs::msg::Pose& pose);
  void disableMarker(const std::string& id);

  void processSetPoseFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

 private:
  // Creates markers without adding them to the marker server.
  void createMarkers();

  // ROS stuff.
  rclcpp::Node::SharedPtr node_;
  interactive_markers::InteractiveMarkerServer marker_server_;

  // Settings.
  std::string frame_id_;

  // State.
  bool initialized_;
  visualization_msgs::msg::InteractiveMarker set_pose_marker_;

  // This is map for waypoint visualization markers:
  std::map<std::string, visualization_msgs::msg::InteractiveMarker> marker_map_;
  // This determines how the markers in the marker map will look:
  visualization_msgs::msg::InteractiveMarker marker_prototype_;

  // State:
  PoseUpdatedFunctionType pose_updated_function_;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_INTERACTIVE_MARKERS_H_
