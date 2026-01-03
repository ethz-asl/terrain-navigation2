#include "mav_planning_rviz/goal_marker.h"

#include <functional>
#include <sstream>
#include <iomanip>

using std::placeholders::_1;

GoalMarker::GoalMarker(rclcpp::Node::SharedPtr node) : node_(node), marker_server_("goal", node) {
  set_goal_marker_.header.stamp = node_->now();
  set_goal_marker_.header.frame_id = "map";
  set_goal_marker_.name = "set_pose";
  set_goal_marker_.description = "Drag to move, right-click for menu";
  set_goal_marker_.scale = 100.0;
  set_goal_marker_.controls.clear();

  const double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::msg::InteractiveMarkerControl control;
  set_goal_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  control.always_visible = true;
  set_goal_marker_.controls.push_back(control);

  visualization_msgs::msg::InteractiveMarkerControl altitude_control;
  altitude_control.orientation.w = kSqrt2Over2;
  altitude_control.orientation.x = 0;
  altitude_control.orientation.y = kSqrt2Over2;
  altitude_control.orientation.z = 0;
  altitude_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  altitude_control.name = "move altitude";
  altitude_control.always_visible = true;
  set_goal_marker_.controls.push_back(altitude_control);

  // Add menu control for right-click context menu
  visualization_msgs::msg::InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  menu_control.name = "menu";
  menu_control.always_visible = true;
  // Add a sphere marker to make the menu clickable
  visualization_msgs::msg::Marker menu_marker;
  menu_marker.type = visualization_msgs::msg::Marker::SPHERE;
  menu_marker.scale.x = 50.0;
  menu_marker.scale.y = 50.0;
  menu_marker.scale.z = 50.0;
  menu_marker.color.r = 0.2;
  menu_marker.color.g = 0.6;
  menu_marker.color.b = 1.0;
  menu_marker.color.a = 0.8;
  menu_control.markers.push_back(menu_marker);
  set_goal_marker_.controls.push_back(menu_control);

  // Create menu entries
  set_start_entry_ = menu_handler_.insert("Set as Start", std::bind(&GoalMarker::processMenuFeedback, this, _1));
  set_goal_entry_ = menu_handler_.insert("Set as Goal", std::bind(&GoalMarker::processMenuFeedback, this, _1));
  set_soaring_goal_entry_ = menu_handler_.insert("Soaring Goal", std::bind(&GoalMarker::processMenuFeedback, this, _1));

  marker_server_.insert(set_goal_marker_);
  marker_server_.setCallback(set_goal_marker_.name, std::bind(&GoalMarker::processSetPoseFeedback, this, _1));
  menu_handler_.apply(marker_server_, set_goal_marker_.name);
  marker_server_.applyChanges();

  grid_map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      "/grid_map", 1, std::bind(&GoalMarker::GridmapCallback, this, _1));

  altitude_text_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("goal_altitude_text", 1);
}

GoalMarker::~GoalMarker() = default;

Eigen::Vector3d GoalMarker::getGoalPosition() { return goal_pos_; };

Eigen::Vector3d GoalMarker::toEigen(const geometry_msgs::msg::Pose &p) {
  Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
  return position;
}

void GoalMarker::processMenuFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
    if (feedback->menu_entry_id == set_goal_entry_ && set_goal_callback_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Menu: Set as Goal selected");
      set_goal_callback_(feedback->pose);
    } else if (feedback->menu_entry_id == set_start_entry_ && set_start_callback_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Menu: Set as Start selected");
      set_start_callback_(feedback->pose);
    } else if (feedback->menu_entry_id == set_soaring_goal_entry_ && set_soaring_goal_callback_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Menu: Set as Soaring Goal selected");
      set_soaring_goal_callback_(feedback->pose);
    }
  }
}

void GoalMarker::processSetPoseFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    Eigen::Vector2d marker_position_2d(feedback->pose.position.x, feedback->pose.position.y);
    if (map_.isInside(marker_position_2d)) {
      double elevation = map_.atPosition("elevation", marker_position_2d);
      // Calculate terrain-relative altitude from feedback position
      double relative_altitude = feedback->pose.position.z - elevation;
      // Enforce minimum and maximum relative altitude
      if (relative_altitude < min_relative_altitude_) {
        relative_altitude = min_relative_altitude_;
      } else if (relative_altitude > max_relative_altitude_) {
        relative_altitude = max_relative_altitude_;
      }
      set_goal_marker_.pose = feedback->pose;
      set_goal_marker_.pose.position.z = elevation + relative_altitude;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
      goal_pos_ = toEigen(set_goal_marker_.pose);
      updateAltitudeText(relative_altitude);
    }
  }
  marker_server_.applyChanges();
}

void GoalMarker::GridmapCallback(const grid_map_msgs::msg::GridMap &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  grid_map::GridMapRosConverter::fromMessage(msg, map_);
  Eigen::Vector2d marker_position_2d(set_goal_marker_.pose.position.x, set_goal_marker_.pose.position.y);
  if (map_.isInside(marker_position_2d)) {
    double elevation = map_.atPosition("elevation", marker_position_2d);
    // Enforce minimum and maximum terrain-relative altitude
    double min_altitude = elevation + min_relative_altitude_;
    double max_altitude = elevation + max_relative_altitude_;
    if (set_goal_marker_.pose.position.z < min_altitude) {
      set_goal_marker_.pose.position.z = min_altitude;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
    } else if (set_goal_marker_.pose.position.z > max_altitude) {
      set_goal_marker_.pose.position.z = max_altitude;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
    }
    goal_pos_ = toEigen(set_goal_marker_.pose);
    double relative_altitude = set_goal_marker_.pose.position.z - elevation;
    updateAltitudeText(relative_altitude);
  }
  marker_server_.applyChanges();
}

void GoalMarker::updateAltitudeText(double relative_altitude) {
  visualization_msgs::msg::Marker text_marker;
  text_marker.header.frame_id = "map";
  text_marker.header.stamp = node_->now();
  text_marker.ns = "goal_altitude";
  text_marker.id = 0;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;

  // Position the text above the marker
  text_marker.pose.position.x = set_goal_marker_.pose.position.x + 60.0;
  text_marker.pose.position.y = set_goal_marker_.pose.position.y + 60.0;
  text_marker.pose.position.z = set_goal_marker_.pose.position.z + 60.0;
  text_marker.pose.orientation.w = 1.0;

  text_marker.scale.z = 20.0;  // Text height

  text_marker.color.r = 0.0;
  text_marker.color.g = 0.5;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(0) << relative_altitude << " m AGL";
  text_marker.text = oss.str();

  altitude_text_pub_->publish(text_marker);
}
