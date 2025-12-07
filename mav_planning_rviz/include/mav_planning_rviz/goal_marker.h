
#ifndef MAV_PLANNING_RVIZ_GOAL_MARKER_H_
#define MAV_PLANNING_RVIZ_GOAL_MARKER_H_

#include <visualization_msgs/msg/interactive_marker_feedback.h>

#include <Eigen/Dense>
#include <functional>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

class GoalMarker {
 public:
  typedef std::function<void(const geometry_msgs::msg::Pose& pose)> MenuCallbackType;

  GoalMarker(rclcpp::Node::SharedPtr node);
  virtual ~GoalMarker();
  Eigen::Vector3d getGoalPosition();

  // Set callbacks for menu actions
  void setGoalCallback(const MenuCallbackType& callback) { set_goal_callback_ = callback; }
  void setStartCallback(const MenuCallbackType& callback) { set_start_callback_ = callback; }

 private:
  Eigen::Vector3d toEigen(const geometry_msgs::msg::Pose &p);
  void processSetPoseFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void processMenuFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void GridmapCallback(const grid_map_msgs::msg::GridMap &msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;

  interactive_markers::InteractiveMarkerServer marker_server_;
  visualization_msgs::msg::InteractiveMarker set_goal_marker_;

  // Menu
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle set_goal_entry_;
  interactive_markers::MenuHandler::EntryHandle set_start_entry_;
  MenuCallbackType set_goal_callback_;
  MenuCallbackType set_start_callback_;

  Eigen::Vector3d goal_pos_{Eigen::Vector3d::Zero()};
  grid_map::GridMap map_;
  std::mutex goal_mutex_;
};

#endif  // MAV_PLANNING_RVIZ_GOAL_MARKER_H_
