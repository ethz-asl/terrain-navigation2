/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,R
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Terrain Planner
 *
 * Terrain Planner
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_navigation_ros/terrain_planner.h"

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <planner_msgs/msg/navigation_status.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "terrain_navigation_ros/geo_conversions.h"
#include "terrain_navigation_ros/visualization.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

TerrainPlanner::TerrainPlanner() : Node("terrain_planner") {
  std::string avalanche_map_path;

  // original parameters: resource locations and goal radius
  resource_path_ = this->declare_parameter("resource_path", "resources");
  map_path_ = this->declare_parameter("terrain_path", resource_path_ + "/davosdorf.tif");
  map_color_path_ = this->declare_parameter("terrain_color_path", resource_path_ + "/davosdorf_color");
  mesh_resource_path_ = this->declare_parameter("meshresource_path", resource_path_ + "/believer.dae");
  avalanche_map_path = this->declare_parameter("avalanche_map_path", resource_path_ + "/avalanche.tif");
  goal_radius_ = this->declare_parameter("minimum_turn_radius", 66.67);

  RCLCPP_INFO_STREAM(this->get_logger(), "resource_path: " << resource_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "map_path_: " << map_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "map_color_path_: " << map_color_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "mesh_resource_path_: " << mesh_resource_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "avalanche_map_path: " << avalanche_map_path);
  RCLCPP_INFO_STREAM(this->get_logger(), "goal_radius_: " << goal_radius_);

  // additional parameters: vehicle guidance
  K_z_ = this->declare_parameter("alt_control_p", 0.5);
  max_climb_rate_control_ = this->declare_parameter("alt_control_max_climb_rate", 3.0);
  cruise_speed_ = this->declare_parameter("cruise_speed", 15.0);

  RCLCPP_INFO_STREAM(this->get_logger(), "alt_control_p: " << K_z_);
  RCLCPP_INFO_STREAM(this->get_logger(), "alt_control_max_climb_rate: " << max_climb_rate_control_);
  RCLCPP_INFO_STREAM(this->get_logger(), "cruise_speed: " << cruise_speed_);

  // px4 namespace parameter
  px4_namespace_ = this->declare_parameter("px4_namespace", "");
  RCLCPP_INFO_STREAM(this->get_logger(), "px4_namespace: " << px4_namespace_);

  // quality of service settings
  rclcpp::QoS latching_qos(1);
  latching_qos.reliable().transient_local();

  auto mavros_position_qos = rclcpp::SensorDataQoS();
  // mavros_position_qos.best_effort();

  vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vehicle_path", 1);

  //! @todo(srmainwaring) add latching
  grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", latching_qos);

  posehistory_pub_ = this->create_publisher<nav_msgs::msg::Path>("geometric_controller/path", 10);
  referencehistory_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference/path", 10);
  position_target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("position_target", 1);
  curvature_target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("curvature_target", 1);
  vehicle_velocity_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_velocity", 1);
  goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 1);
  rallypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rallypoints_marker", 1);
  candidate_goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("candidate_goal_marker", 1);
  candidate_start_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("candidate_start_marker", 1);

  setpoint_triplet_sub_ = this->create_subscription<px4_msgs::msg::PositionSetpointTriplet>(
      px4_namespace_ + "/fmu/out/position_setpoint_triplet", mavros_position_qos,
      std::bind(&TerrainPlanner::setpointTripletCallback, this, _1));

  vehicle_pose_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vehicle_pose_marker", 1);
  planner_status_pub_ = this->create_publisher<planner_msgs::msg::NavigationStatus>("planner_status", 1);
  path_segment_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_segments", 1);
  tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree", 1);

  mavlocalpose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_namespace_ + "/fmu/out/vehicle_local_position", mavros_position_qos,
      std::bind(&TerrainPlanner::mavLocalPoseCallback, this, _1));
  mavattitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      px4_namespace_ + "/fmu/out/vehicle_attitude", mavros_position_qos,
      std::bind(&TerrainPlanner::mavAttitudeCallback, this, _1));
  mavglobalpose_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      px4_namespace_ + "/fmu/out/vehicle_global_position", mavros_position_qos,
      std::bind(&TerrainPlanner::mavGlobalPoseCallback, this, _1));

  setlocation_serviceserver_ = this->create_service<planner_msgs::srv::SetString>(
      "/terrain_planner/set_location", std::bind(&TerrainPlanner::setLocationCallback, this, _1, _2));
  setmaxaltitude_serviceserver_ = this->create_service<planner_msgs::srv::SetString>(
      "/terrain_planner/set_max_altitude", std::bind(&TerrainPlanner::setMaxAltitudeCallback, this, _1, _2));
  setgoal_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/terrain_planner/set_goal", std::bind(&TerrainPlanner::setGoalCallback, this, _1, _2));
  setstart_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/terrain_planner/set_start", std::bind(&TerrainPlanner::setStartCallback, this, _1, _2));
  setcurrentsegment_serviceserver_ = this->create_service<planner_msgs::srv::SetService>(
      "/terrain_planner/set_current_segment", std::bind(&TerrainPlanner::setCurrentSegmentCallback, this, _1, _2));
  setstartloiter_serviceserver_ = this->create_service<planner_msgs::srv::SetService>(
      "/terrain_planner/set_start_loiter", std::bind(&TerrainPlanner::setStartLoiterCallback, this, _1, _2));
  setplannerstate_service_server_ = this->create_service<planner_msgs::srv::SetPlannerState>(
      "/terrain_planner/set_planner_state", std::bind(&TerrainPlanner::setPlannerStateCallback, this, _1, _2));
  setplanning_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/terrain_planner/trigger_planning", std::bind(&TerrainPlanner::setPlanningCallback, this, _1, _2));
  updatepath_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/terrain_planner/set_path", std::bind(&TerrainPlanner::setPathCallback, this, _1, _2));

  // create terrain map
  terrain_map_ = std::make_shared<TerrainMap>();

  // Initialize Dubins state space
  dubins_state_space_ = std::make_shared<ompl::base::OwenStateSpace>(goal_radius_);
  global_planner_ = std::make_shared<TerrainOmplRrt>(ompl::base::StateSpacePtr(dubins_state_space_));
  global_planner_->setMap(terrain_map_);
  global_planner_->setAltitudeLimits(max_elevation_, min_elevation_);

  planner_profiler_ = std::make_shared<Profiler>("planner");

  // Initialise times - required to ensure all time sources are consistent
  plan_time_ = this->get_clock()->now();
  last_triggered_time_ = this->get_clock()->now();

  // Build the topic prefix expected by px4_ros2: strip leading '/', add trailing '/'
  std::string topic_prefix;
  if (!px4_namespace_.empty()) {
    topic_prefix = (px4_namespace_.front() == '/') ? px4_namespace_.substr(1) : px4_namespace_;
    if (topic_prefix.back() != '/') topic_prefix += '/';
  }
  terrain_mode_ = std::make_unique<TerrainNavigationMode>(*this, topic_prefix);
}

void TerrainPlanner::init() {
  if (!terrain_mode_->doRegister()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to register TerrainNavigationMode with PX4");
  }

  auto plannerloop_dt_ = 2s;
  plannerloop_timer_ = this->create_wall_timer(plannerloop_dt_, std::bind(&TerrainPlanner::plannerloopCallback, this));

  auto statusloop_dt_ = 500ms;
  statusloop_timer_ = this->create_wall_timer(statusloop_dt_, std::bind(&TerrainPlanner::statusloopCallback, this));

  auto cmdloop_dt_ = 100ms;
  cmdloop_timer_ = this->create_wall_timer(cmdloop_dt_, std::bind(&TerrainPlanner::cmdloopCallback, this));
}

Eigen::Vector4d TerrainPlanner::rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

void TerrainPlanner::cmdloopCallback() {
  if (!map_initialized_) return;

  if (!reference_primitive_.segments.empty()) {
    Eigen::Vector3d reference_position;
    Eigen::Vector3d reference_tangent;
    double reference_curvature{0.0};
    auto current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
    double path_progress = current_segment.getClosestPoint(vehicle_position_, reference_position, reference_tangent,
                                                           reference_curvature, 1.0E-4);
    // Publish global position setpoints in the global frame
    EPSG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
    // Convert reference position to global
    double latitude;
    double longitude;
    double altitude;
    if (map_coordinate == EPSG::CH1903_LV03) {
      const Eigen::Vector3d lv03_reference_position = reference_position + map_origin;
      GeoConversions::reverse(lv03_reference_position(0), lv03_reference_position(1), lv03_reference_position(2),
                              latitude, longitude, altitude);
    } else {
      // Gdal conversions
      const Eigen::Vector3d geocoordinate_position = reference_position + map_origin;
      auto transformed_position = transformCoordinates(map_coordinate, EPSG::WGS84, geocoordinate_position);
      latitude = transformed_position(0);
      longitude = transformed_position(1);
      altitude = transformed_position(2);
    }
    publishReferenceMarker(position_target_pub_, reference_position, reference_tangent, reference_curvature);
    publishReferenceCurvatureMarker(curvature_target_pub_, reference_position, reference_tangent, reference_curvature);

    // Run additional altitude control
    double altitude_correction = K_z_ * (vehicle_position_(2) - reference_position(2));
    double climb_rate = cruise_speed_ * std::sin(current_segment.flightpath_angle);
    Eigen::Vector3d velocity_reference = reference_tangent;

    velocity_reference(2) =
        std::min(std::max(altitude_correction - climb_rate, -max_climb_rate_control_), max_climb_rate_control_);

    /// Blend curvature with next segment
    double curvature_reference = reference_curvature;
    int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
    bool is_last_segment = bool(current_segment_idx >= static_cast<int>(reference_primitive_.segments.size() - 1));
    if (!is_last_segment) {
      /// Get next segment curvature
      double next_segment_curvature = reference_primitive_.segments[current_segment_idx + 1].curvature;

      /// Blend current curvature with next curvature when close to the end
      double segment_length = current_segment.getLength(1.0);
      double cut_off_distance = 10.0;
      double portion = std::min(
          1.0, std::max((path_progress * segment_length - segment_length + cut_off_distance) / cut_off_distance, 0.0));
      curvature_reference = (1 - portion) * reference_curvature + portion * next_segment_curvature;
    }

    // Publish via the PX4 ROS 2 mode API (ENU velocity → NED tangent/height_rate, curvature sign flip)
    Eigen::Vector3f tangent_ned(static_cast<float>(velocity_reference(1)),   // N ← ENU Y
                                static_cast<float>(velocity_reference(0)),   // E ← ENU X
                                static_cast<float>(-velocity_reference(2))); // D ← ENU -Z
    terrain_mode_->publishSetpoint(latitude, longitude, static_cast<float>(altitude), tangent_ned,
                                   static_cast<float>(-velocity_reference(2)),
                                   static_cast<float>(-curvature_reference));

    if (terrain_mode_->isActive()) {
      publishPositionHistory(referencehistory_pub_, reference_position, referencehistory_vector_);
      tracking_error_ = reference_position - vehicle_position_;
      planner_enabled_ = true;
    } else {
      tracking_error_ = Eigen::Vector3d::Zero();
      planner_enabled_ = false;
    }
  }

  planner_msgs::msg::NavigationStatus msg;
  msg.header.stamp = this->get_clock()->now();
  // msg.planner_time.data = planner_time;
  msg.state = static_cast<uint8_t>(planner_state_);
  msg.tracking_error = toVector3(tracking_error_);
  msg.enabled = planner_enabled_;
  // msg.reference_position = toVector3(reference_position);
  msg.vehicle_position = toVector3(vehicle_position_);
  planner_status_pub_->publish(msg);

  publishVehiclePose(vehicle_pose_pub_, vehicle_position_, vehicle_attitude_, mesh_resource_path_);
  publishVelocityMarker(vehicle_velocity_pub_, vehicle_position_, vehicle_velocity_);
  publishPositionHistory(posehistory_pub_, vehicle_position_, posehistory_vector_);
}

void TerrainPlanner::statusloopCallback() {
  // Check if we want to update the planner state if query state and current state is different
  planner_state_ = finiteStateMachine(planner_state_, query_planner_state_);
  if (query_planner_state_ != planner_state_) {  // Query has been rejected, reset
    query_planner_state_ = planner_state_;
  }
  // printPlannerState(planner_state_);
}

void TerrainPlanner::plannerloopCallback() {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!map_initialized_) {
    //! @todo(srmainwaring) consolidate duplicate code from here and TerrainPlanner::setLocationCallback
    std::cout << "[TerrainPlanner] Local origin received, loading map" << std::endl;
    map_initialized_ = terrain_map_->Load(map_path_, map_color_path_);
    terrain_map_->AddLayerDistanceTransform(min_elevation_, "distance_surface");
    terrain_map_->AddLayerDistanceTransform(max_elevation_, "max_elevation");
    terrain_map_->AddLayerHorizontalDistanceTransform(goal_radius_, "ics_+", "distance_surface");
    terrain_map_->AddLayerHorizontalDistanceTransform(-goal_radius_, "ics_-", "max_elevation");
    terrain_map_->addLayerSafety("safety", "ics_+", "ics_-");

    EPSG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);

    if (map_initialized_) {
      std::cout << "[TerrainPlanner]   - Successfully loaded map: " << map_path_ << std::endl;
      MapPublishOnce(grid_map_pub_, terrain_map_->getGridMap());
      global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
      global_planner_->setupProblem(start_pos_, goal_pos_, start_loiter_radius_);
    } else {
      std::cout << "[TerrainPlanner]   - Failed to load map: " << map_path_ << std::endl;
    }
    return;
  }

  // planner_profiler_->tic();

  switch (planner_mode_) {
    case PLANNER_MODE::GLOBAL: {
      // Solve planning problem with RRT*
      auto time_now = this->get_clock()->now();
      double time_spent_planning_s = (time_now - plan_time_).seconds();

      // RCLCPP_INFO_STREAM(this->get_logger(), "plan_time:           " << plan_time_.seconds());
      // RCLCPP_INFO_STREAM(this->get_logger(), "time_now:            " << time_now.seconds());

      if (time_spent_planning_s < planner_time_budget_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "time_spent_planning: " << time_spent_planning_s);
        RCLCPP_INFO_STREAM(this->get_logger(), "planner_time_budget: " << planner_time_budget_);

        // bool found_solution =
        global_planner_->Solve(1.0, candidate_primitive_);
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      } else {
        publishPathSegments(path_segment_pub_, candidate_primitive_);
      }
      break;
    }
    case PLANNER_MODE::EMERGENCY_ABORT: {
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        PathSegment current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        Eigen::Vector3d start_position = current_segment.states.back().position;
        Eigen::Vector3d start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_start_position_ && !found_solution_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          /// Generate candidate rally points
          const int num_rally_points = 3;
          rally_points.clear();
          for (int i = 0; i < num_rally_points; i++) {
            bool sample_is_valid = false;
            while (!sample_is_valid) {
              Eigen::Vector3d random_sample;
              random_sample(0) = getRandom(-200.0, 200.0);
              random_sample(1) = getRandom(-200.0, 200.0);
              Eigen::Vector3d candidate_loiter_position = start_position + random_sample;
              Eigen::Vector3d new_loiter_position;
              sample_is_valid =
                  validatePosition(terrain_map_->getGridMap(), candidate_loiter_position, new_loiter_position);
              if (sample_is_valid) {
                rally_points.push_back(new_loiter_position);
              }
            }
          }

          global_planner_->setupProblem(start_position, start_velocity, rally_points);
          /// Publish Rally points
          publishRallyPoints(rallypoint_pub_, rally_points, 66.67, Eigen::Vector3d(1.0, 1.0, 0.0));
          previous_start_position_ = start_position;
        }

        Path planner_solution_path;
        bool found_solution = global_planner_->Solve(0.5, planner_solution_path);
        found_solution_ = found_solution;

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_solution_ && problem_updated_) {
          problem_updated_ = false;

          bool update_solution{false};
          update_solution = planner_solution_path.segments.empty() ? false : true;

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            Path updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;
            /// TODO: Figure out which rally point the planner is using
            double min_distance_error = std::numeric_limits<double>::infinity();
            int min_distance_index = -1;
            for (int idx = 0; idx < static_cast<int>(rally_points.size()); idx++) {
              double radial_error =
                  std::abs((end_position - rally_points[idx]).norm() - dubins_state_space_->getMinTurnRadius());
              if (radial_error < min_distance_error) {
                min_distance_index = idx;
                min_distance_error = radial_error;
              }
            }
            Eigen::Vector3d radial_vector = (end_position - rally_points[min_distance_index]);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            // Eigen::Vector3d emergency_rates =
            //     20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            // double horizon = 2 * M_PI / std::abs(emergency_rates(2));
            //  Append a loiter at the end of the planned path
            PathSegment loiter_trajectory;
            generateCircle(end_position, end_velocity, rally_points[min_distance_index], loiter_trajectory);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
            candidate_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
    case PLANNER_MODE::RETURN: {
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        PathSegment current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        Eigen::Vector3d start_position = current_segment.states.back().position;
        Eigen::Vector3d start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_return_start_position_ && !found_return_solution_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          global_planner_->setupProblem(start_position, start_velocity, home_position_, home_position_radius_);
          previous_return_start_position_ = start_position;
        }

        Path planner_solution_path;
        bool found_solution = global_planner_->Solve(0.5, planner_solution_path);
        found_return_solution_ = found_solution;

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_return_solution_ && problem_updated_) {
          problem_updated_ = false;

          bool update_solution{false};
          update_solution = planner_solution_path.segments.empty() ? false : true;

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            Path updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;

            Eigen::Vector3d radial_vector = (end_position - home_position_);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            // Eigen::Vector3d emergency_rates =
            //     20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            // double horizon = 2 * M_PI / std::abs(emergency_rates(2));
            // Append a loiter at the end of the planned path
            PathSegment loiter_trajectory;
            generateCircle(end_position, end_velocity, home_position_, loiter_trajectory);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
            candidate_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
    default:
      break;
  }

  // double planner_time = planner_profiler_->toc();
  publishTrajectory(reference_primitive_.position());
  // publishGoal(goal_pub_, goal_pos_, 66.67, Eigen::Vector3d(0.0, 1.0, 0.0));
}

PLANNER_STATE TerrainPlanner::finiteStateMachine(const PLANNER_STATE current_state, const PLANNER_STATE query_state) {
  PLANNER_STATE next_state;
  next_state = current_state;
  switch (current_state) {
    case PLANNER_STATE::NAVIGATE: {
      // Switch to Hold when segment has been completed
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= static_cast<int>(reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        next_state = PLANNER_STATE::HOLD;
      }

      // Stay in hold mode if the current segment is periodic, Otherwise switch to abort
      if (query_state == PLANNER_STATE::ABORT) {
        if (!reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::ABORT;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        } else {
          /// TODO: Get rid of next segments and only keep current segment
        }
      } else if (query_state == PLANNER_STATE::RETURN) {
        next_state = PLANNER_STATE::RETURN;
        planner_mode_ = PLANNER_MODE::RETURN;
      }
      break;
    }
    case PLANNER_STATE::ROLLOUT: {
      /// TODO: Get rollout primitive from active mapper
      // if (candidate_primitive_.valid()) {
      // Update reference if candidate is valid
      // reference_primitive_ = candidate_primitive_;
      // std::cout << "Candidate primitive is valid" << std::endl;
      // }
      reference_primitive_ = rollout_primitive_;
      /// TODO: Add self termination
      if (query_state == PLANNER_STATE::ABORT) {
        if (reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::HOLD;
          planner_mode_ = PLANNER_MODE::GLOBAL;
        } else {
          next_state = query_state;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        }
      }
      break;
    }
    case PLANNER_STATE::HOLD: {
      switch (query_state) {
        case PLANNER_STATE::NAVIGATE: {
          // Check if the candidate primitive is not empty
          if (!candidate_primitive_.segments.empty()) {
            // Add initial loiter
            Eigen::Vector3d start_position = candidate_primitive_.firstSegment().states.front().position;
            Eigen::Vector3d start_velocity = candidate_primitive_.firstSegment().states.front().velocity;
            PathSegment start_loiter;
            generateCircle(start_position, start_velocity, start_pos_, start_loiter);
            candidate_primitive_.prependSegment(start_loiter);

            // Add terminal loiter
            Eigen::Vector3d end_position = candidate_primitive_.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = candidate_primitive_.lastSegment().states.back().velocity;
            PathSegment terminal_loiter;
            generateCircle(end_position, end_velocity, goal_pos_, terminal_loiter);
            candidate_primitive_.appendSegment(terminal_loiter);

            reference_primitive_ = candidate_primitive_;
            next_state = query_state;
          }
          break;
        }
        case PLANNER_STATE::ROLLOUT: {
          planner_mode_ = PLANNER_MODE::ACTIVE_MAPPING;
          next_state = query_state;
          break;
          default:
            break;
        }
      }
      break;
    }
    case PLANNER_STATE::RETURN: {
      /// TODO: Check if we have a return position defined
      // Switch to Hold when segment has been completed
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= static_cast<int>(reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        next_state = PLANNER_STATE::HOLD;
        found_return_solution_ = false;
        planner_mode_ = PLANNER_MODE::GLOBAL;
      }

      if (query_state == PLANNER_STATE::ABORT) {
        if (!reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::ABORT;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        }
      }
      break;
    }
    case PLANNER_STATE::ABORT: {
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= static_cast<int>(reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        planner_mode_ = PLANNER_MODE::GLOBAL;
        found_solution_ = false;
        next_state = PLANNER_STATE::HOLD;
      }
    }
  }
  return next_state;
}

void TerrainPlanner::publishTrajectory(std::vector<Eigen::Vector3d> trajectory) {
  nav_msgs::msg::Path msg;
  std::vector<geometry_msgs::msg::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : trajectory) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), vector3d2PoseStampedMsg(pos, orientation));
  }
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;
  vehicle_path_pub_->publish(msg);
}

void TerrainPlanner::mavLocalPoseCallback(const px4_msgs::msg::VehicleLocalPosition &msg) {
  // NED->ENU Conversions
  vehicle_velocity_ = Eigen::Vector3d(msg.vy, msg.vx, -msg.vz);

  // if (!local_origin_received_) {
  /// TODO: Keep track of reset counters
  local_origin_latitude_ = msg.ref_lat;
  local_origin_longitude_ = msg.ref_lon;
  local_origin_altitude_ = msg.ref_alt;
  // }
}

void TerrainPlanner::mavAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg) {
  /// TODO: Fix NED->ENU Conversions
  vehicle_attitude_(0) = msg.q[0];
  vehicle_attitude_(1) = msg.q[2];
  vehicle_attitude_(2) = msg.q[1];
  vehicle_attitude_(3) = -msg.q[3];
}

void TerrainPlanner::mavGlobalPoseCallback(const px4_msgs::msg::VehicleGlobalPosition &msg) {
  Eigen::Vector3d wgs84_vehicle_position;
  wgs84_vehicle_position(0) = msg.lat;
  wgs84_vehicle_position(1) = msg.lon;
  wgs84_vehicle_position(2) = msg.alt;

  EPSG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
  if (map_coordinate == EPSG::CH1903_LV03) {
    Eigen::Vector3d transformed_coordinates;
    // LV03 / WGS84 ellipsoid
    GeoConversions::forward(wgs84_vehicle_position(0), wgs84_vehicle_position(1), wgs84_vehicle_position(2),
                            transformed_coordinates.x(), transformed_coordinates.y(), transformed_coordinates.z());
    vehicle_position_ = transformed_coordinates - map_origin;
  } else {
    // Gdal conversions
    Eigen::Vector3d transformed_coordinates = transformCoordinates(EPSG::WGS84, map_coordinate, wgs84_vehicle_position);
    vehicle_position_ = transformed_coordinates - map_origin;
  }
}

void TerrainPlanner::MapPublishOnce(rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub,
                                    const grid_map::GridMap &map) {
  auto message = grid_map::GridMapRosConverter::toMessage(map);
  pub->publish(*message);
}

void TerrainPlanner::publishPositionHistory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                                            const Eigen::Vector3d &position,
                                            std::vector<geometry_msgs::msg::PoseStamped> &history_vector) {
  //! @todo(srmainwaring) provide an editor to allow this to be modified.
  //! @todo(srmainwaring) make member variable and set default in header.
  unsigned int posehistory_window_ = 200;
  Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
  history_vector.insert(history_vector.begin(), vector3d2PoseStampedMsg(position, vehicle_attitude));
  if (history_vector.size() > posehistory_window_) {
    history_vector.pop_back();
  }

  nav_msgs::msg::Path msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.poses = history_vector;

  pub->publish(msg);
}

void TerrainPlanner::publishVelocityMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                           const Eigen::Vector3d &position, const Eigen::Vector3d &velocity) {
  visualization_msgs::msg::Marker marker = vector2ArrowsMsg(position, velocity, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
  pub->publish(marker);
}

void TerrainPlanner::publishReferenceMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                            const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                                            const double /*curvature*/) {
  Eigen::Vector3d scaled_velocity = 20.0 * velocity;
  visualization_msgs::msg::Marker marker =
      vector2ArrowsMsg(position, scaled_velocity, 0, Eigen::Vector3d(0.0, 0.0, 1.0), "reference");

  pub->publish(marker);
}

void TerrainPlanner::publishReferenceCurvatureMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                                     const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                                                     const double curvature) {
  // do not zero the radius completely for zero curvature as this results in
  // a warning when displaying the marker in rviz.
  double radius = std::fabs(curvature) < 1.0E-4 ? 1.0E-6 : 1.0 / std::fabs(curvature);
  double direction = curvature > 0.0 ? 1.0 : -1.0;
  auto direction_vector = Eigen::Vector3d(0.0, 0.0, direction);
  auto projected_velocity = Eigen::Vector3d(velocity(0), velocity(1), 0.0);
  auto unit_tangent_vector = projected_velocity.normalized();
  Eigen::Vector3d curvature_vector = radius * direction_vector.cross(unit_tangent_vector);
  visualization_msgs::msg::Marker marker =
      vector2ArrowsMsg(position, curvature_vector, 0, Eigen::Vector3d(1.0, 0.0, 0.0), "reference_curvature");

  pub->publish(marker);
}

void TerrainPlanner::publishPathSegments(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                                         Path &trajectory) {
  visualization_msgs::msg::MarkerArray msg;

  std::vector<visualization_msgs::msg::Marker> marker;
  visualization_msgs::msg::Marker mark;
  mark.action = visualization_msgs::msg::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub->publish(msg);

  std::vector<visualization_msgs::msg::Marker> segment_markers;
  int i = 0;
  for (auto &segment : trajectory.segments) {
    Eigen::Vector3d color = Eigen::Vector3d(1.0, 0.0, 0.0);
    if (segment.curvature > 0.0) {  // Green is DUBINS_LEFT
      color = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else if (segment.curvature < 0.0) {  // Blue is DUBINS_RIGHT
      color = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
  }
  msg.markers = segment_markers;
  pub->publish(msg);
}

void TerrainPlanner::publishGoal(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                 const Eigen::Vector3d &position, const double radius, Eigen::Vector3d color,
                                 std::string name_space) {
  visualization_msgs::msg::Marker marker;
  marker = getGoalMarker(1, position, radius, color);
  marker.ns = name_space;
  pub->publish(marker);
}

void TerrainPlanner::publishRallyPoints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                                        const std::vector<Eigen::Vector3d> &positions, const double radius,
                                        Eigen::Vector3d color, std::string name_space) {
  visualization_msgs::msg::MarkerArray marker_array;
  std::vector<visualization_msgs::msg::Marker> markers;
  int marker_id = 1;
  for (const auto &position : positions) {
    visualization_msgs::msg::Marker marker;
    marker = getGoalMarker(marker_id, position, radius, color);
    marker.ns = name_space;
    markers.push_back(marker);
    marker_id++;
  }
  marker_array.markers = markers;
  pub->publish(marker_array);
}

visualization_msgs::msg::Marker TerrainPlanner::getGoalMarker(const int id, const Eigen::Vector3d &position,
                                                              const double radius, const Eigen::Vector3d color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  std::vector<geometry_msgs::msg::Point> points;
  /// TODO: Generate circular path
  double delta_theta = 0.05 * 2 * M_PI;
  for (double theta = 0.0; theta < 2 * M_PI + delta_theta; theta += delta_theta) {
    geometry_msgs::msg::Point point;
    point.x = position(0) + radius * std::cos(theta);
    point.y = position(1) + radius * std::sin(theta);
    point.z = position(2);
    points.push_back(point);
  }
  marker.points = points;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

void TerrainPlanner::setpointTripletCallback(const px4_msgs::msg::PositionSetpointTriplet &msg) {
  const double lat = msg.current.lat;
  const double lon = msg.current.lon;
  const double alt = msg.current.alt;
  const double loiter_radius = msg.current.loiter_radius;

  if (std::isfinite(lat) && std::isfinite(lon)) {
    std::cout << "Setpoint Triplet Callback" << std::endl;
    std::cout << "  - lat: " << lat << " lon: " << lon << " alt: " << alt << std::endl;
    std::cout << "  - loiter_radius: " << loiter_radius << std::endl;

    EPSG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);

    if (map_coordinate == EPSG::CH1903_LV03) {
      Eigen::Vector3d lv03_mission_loiter_center;
      GeoConversions::forward(lat, lon, alt, lv03_mission_loiter_center.x(), lv03_mission_loiter_center.y(),
                              lv03_mission_loiter_center.z());
      std::cout << "  - mission_loiter_center_: " << lv03_mission_loiter_center.transpose() << std::endl;
      mission_loiter_center_ = lv03_mission_loiter_center - map_origin;
    } else {
      // Gdal conversions
      Eigen::Vector3d transformed_coordinates =
          transformCoordinates(EPSG::WGS84, map_coordinate, Eigen::Vector3d(lat, lon, alt));
      std::cout << "  - mission_loiter_center_: " << transformed_coordinates.transpose() << std::endl;
      mission_loiter_center_ = transformed_coordinates - map_origin;
    }
    mission_loiter_radius_ = loiter_radius;
  }
}

bool TerrainPlanner::setLocationCallback(const std::shared_ptr<planner_msgs::srv::SetString::Request> req,
                                         std::shared_ptr<planner_msgs::srv::SetString::Response> res) {
  //! @todo(srmainwaring) consolidate duplicate code from here and TerrainPlanner::plannerloopCallback
  std::string set_location = req->string;
  //! @todo(srmainwaring) interface supporting align_location has been removed,
  //!                     decide how to treat (and see below L1112)
  // bool align_location = req->align;
  std::cout << "[TerrainPlanner] Set Location: " << set_location << std::endl;
  // std::cout << "[TerrainPlanner] Set Alignment: " << align_location << std::endl;

  /// TODO: Add location from the new set location service
  map_path_ = resource_path_ + "/" + set_location + ".tif";
  map_color_path_ = resource_path_ + "/" + set_location + "_color.tif";
  bool result = terrain_map_->Load(map_path_, map_color_path_);

  //! @todo(srmainwaring) check result valid before further operations?
  std::cout << "[TerrainPlanner]   Computing distance transforms" << std::endl;
  terrain_map_->AddLayerDistanceTransform(min_elevation_, "distance_surface");
  terrain_map_->AddLayerDistanceTransform(max_elevation_, "max_elevation");
  std::cout << "[TerrainPlanner]   Computing horizontal distance transforms " << std::endl;
  terrain_map_->AddLayerHorizontalDistanceTransform(goal_radius_, "ics_+", "distance_surface");
  terrain_map_->AddLayerHorizontalDistanceTransform(-goal_radius_, "ics_-", "max_elevation");
  std::cout << "[TerrainPlanner]   Computing safety layers" << std::endl;
  terrain_map_->addLayerSafety("safety", "ics_+", "ics_-");

  // if (!align_location) {
  //   // Depending on Gdal versions, lon lat order are reversed
  //   Eigen::Vector3d lv03_local_origin;
  //   GeoConversions::forward(local_origin_latitude_, local_origin_longitude_, local_origin_altitude_,
  //                           lv03_local_origin.x(), lv03_local_origin.y(), lv03_local_origin.z());
  //   if (terrain_map_->getGridMap().isInside(Eigen::Vector2d(0.0, 0.0))) {
  //     double terrain_altitude = terrain_map_->getGridMap().atPosition("elevation", Eigen::Vector2d(0.0, 0.0));
  //     lv03_local_origin(2) = lv03_local_origin(2) - terrain_altitude;
  //   }
  //   terrain_map_->setGlobalOrigin(EPSG::CH1903_LV03, lv03_local_origin);
  // }
  if (result) {
    global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
    global_planner_->setupProblem(start_pos_, goal_pos_, start_loiter_radius_);
    problem_updated_ = true;
    posehistory_vector_.clear();
    MapPublishOnce(grid_map_pub_, terrain_map_->getGridMap());
  }
  res->success = result;
  return true;
}

bool TerrainPlanner::setMaxAltitudeCallback(const std::shared_ptr<planner_msgs::srv::SetString::Request> req,
                                            std::shared_ptr<planner_msgs::srv::SetString::Response> res) {
  bool check_max_altitude = req->align;
  std::cout << "[TerrainPlanner] Max altitude constraint configured: " << check_max_altitude << std::endl;
  global_planner_->setMaxAltitudeCollisionChecks(check_max_altitude);
  res->success = true;
  return true;
}

bool TerrainPlanner::setGoalCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                                     std::shared_ptr<planner_msgs::srv::SetVector3::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  Eigen::Vector3d candidate_goal = Eigen::Vector3d(req->vector.x, req->vector.y, req->vector.z);
  Eigen::Vector3d new_goal;
  std::cout << "[TerrainPlanner] Candidate goal: " << candidate_goal.transpose() << std::endl;
  bool is_goal_safe = validatePosition(terrain_map_->getGridMap(), candidate_goal, new_goal);
  if (is_goal_safe) {
    goal_pos_ = new_goal;
    // mcts_planner_->setGoal(new_goal);
    // problem_updated_ = true;
    res->success = true;
    publishGoal(candidate_goal_pub_, new_goal, goal_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "goal");
    return true;
  } else {
    res->success = false;
    publishGoal(candidate_goal_pub_, new_goal, goal_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "goal");
    return false;
  }
}

bool TerrainPlanner::setStartCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                                      std::shared_ptr<planner_msgs::srv::SetVector3::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  Eigen::Vector3d candidate_start = Eigen::Vector3d(req->vector.x, req->vector.y, req->vector.z);
  Eigen::Vector3d new_start;
  std::cout << "[TerrainPlanner] Candidate start: " << candidate_start.transpose() << std::endl;
  bool is_safe = validatePosition(terrain_map_->getGridMap(), candidate_start, new_start);
  if (is_safe) {
    start_pos_ = new_start;
    res->success = true;
    publishGoal(candidate_start_pub_, new_start, start_loiter_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "start");
    return true;
  } else {
    res->success = false;
    publishGoal(candidate_start_pub_, new_start, start_loiter_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "start");
    return false;
  }
}

bool TerrainPlanner::setCurrentSegmentCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> /*req*/,
                                               std::shared_ptr<planner_msgs::srv::SetService::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  /// TODO: Get center of the last segment of the reference path
  if (!reference_primitive_.segments.empty()) {
    auto last_segment = reference_primitive_.lastSegment();
    if (last_segment.is_periodic) {
      /// TODO: Get the center of the circle
      Eigen::Vector3d segment_start = last_segment.states.front().position;
      Eigen::Vector3d segment_start_tangent = (last_segment.states.front().velocity).normalized();
      auto arc_center =
          PathSegment::getArcCenter(segment_start.head(2), segment_start_tangent.head(2), last_segment.curvature);
      Eigen::Vector3d candidate_start = Eigen::Vector3d(arc_center(0), arc_center(1), 0.0);
      Eigen::Vector3d new_start;
      bool is_safe = validatePosition(terrain_map_->getGridMap(), candidate_start, new_start);
      if (is_safe) {
        start_pos_ = new_start;
        /// TODO: Curvature sign seems to be the opposite from mission items
        start_loiter_radius_ = -1 / last_segment.curvature;
        res->success = true;
        publishGoal(candidate_start_pub_, new_start, goal_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "start");
        return true;
      } else {
        res->success = false;
        publishGoal(candidate_start_pub_, new_start, goal_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "start");
        return false;
      }
    } else {
      std::cout << "[TerrainPlanner] Last segment is not periodic" << std::endl;
    }
  }
  std::cout << "[TerrainPlanner] Could not select current segment, reference is empty" << std::endl;
  res->success = false;
  return true;
}

bool TerrainPlanner::setStartLoiterCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> /*req*/,
                                            std::shared_ptr<planner_msgs::srv::SetService::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  std::cout << "[TerrainPlanner] Current Loiter start: " << mission_loiter_center_.transpose() << std::endl;
  Eigen::Vector3d new_start;
  bool is_safe = validatePosition(terrain_map_->getGridMap(), mission_loiter_center_, new_start);
  if (is_safe) {
    start_pos_ = mission_loiter_center_;
    start_loiter_radius_ = mission_loiter_radius_;
    home_position_ = start_pos_;
    home_position_radius_ = start_loiter_radius_;
    res->success = true;
    publishGoal(candidate_start_pub_, start_pos_, std::abs(mission_loiter_radius_), Eigen::Vector3d(0.0, 1.0, 0.0),
                "start");
    return true;
  } else {
    res->success = false;
    publishGoal(candidate_start_pub_, start_pos_, std::abs(mission_loiter_radius_), Eigen::Vector3d(1.0, 0.0, 0.0),
                "start");
    return false;
  }
}

bool TerrainPlanner::setPlanningCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                                         std::shared_ptr<planner_msgs::srv::SetVector3::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  planner_time_budget_ = req->vector.z;
  problem_updated_ = true;
  plan_time_ = this->get_clock()->now();

  std::cout << "[TerrainPlanner] Planning budget: " << planner_time_budget_ << std::endl;
  std::cout << "[TerrainPlanner] Start position:  " << start_pos_.transpose() << std::endl;
  std::cout << "[TerrainPlanner] Goal position:   " << goal_pos_.transpose() << std::endl;
  std::cout << "[TerrainPlanner] Loiter radius:   " << start_loiter_radius_ << std::endl;

  global_planner_->setupProblem(start_pos_, goal_pos_, start_loiter_radius_);
  planner_mode_ = PLANNER_MODE::GLOBAL;
  res->success = true;
  return true;
}

bool TerrainPlanner::setPathCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> /*req*/,
                                     std::shared_ptr<planner_msgs::srv::SetVector3::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);

  if (!candidate_primitive_.segments.empty()) {
    query_planner_state_ = PLANNER_STATE::NAVIGATE;
    res->success = true;
  } else {
    res->success = false;
  }
  return true;
}

bool TerrainPlanner::setPlannerStateCallback(const std::shared_ptr<planner_msgs::srv::SetPlannerState::Request> req,
                                             std::shared_ptr<planner_msgs::srv::SetPlannerState::Response> res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  int planner_state = static_cast<int>(req->state);
  switch (planner_state) {
    case (1): {
      query_planner_state_ = PLANNER_STATE::HOLD;
      res->success = true;
      break;
    }
    case (2): {
      query_planner_state_ = PLANNER_STATE::NAVIGATE;
      res->success = true;
      break;
    }
    case (3): {
      query_planner_state_ = PLANNER_STATE::ROLLOUT;
      res->success = true;
      break;
    }
    case (4): {
      query_planner_state_ = PLANNER_STATE::ABORT;
      res->success = true;
      break;
    }
    case (5): {
      query_planner_state_ = PLANNER_STATE::RETURN;
      res->success = true;
      break;
    }
    default: {
      res->success = false;
      break;
    }
  }
  std::cout << "planner state: " << planner_state << std::endl;
  return true;
}

void TerrainPlanner::generateCircle(const Eigen::Vector3d end_position, const Eigen::Vector3d end_velocity,
                                    const Eigen::Vector3d center_pos, PathSegment &trajectory) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      cruise_speed_ * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  trajectory.is_periodic = true;
  return;
}

PathSegment TerrainPlanner::generateArcTrajectory(Eigen::Vector3d rate, const double horizon,
                                                  Eigen::Vector3d current_pos, Eigen::Vector3d current_vel,
                                                  const double dt) {
  PathSegment trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.flightpath_angle = std::asin(climb_rate / cruise_speed_);
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = -rate(2) / cruise_speed_;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    double yaw = rate(2) * time + current_yaw;

    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = Eigen::Vector3d(cruise_speed_ * std::cos(yaw), -cruise_speed_ * std::sin(yaw), -climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);

    time = time + dt;
  }
  return trajectory;
}
