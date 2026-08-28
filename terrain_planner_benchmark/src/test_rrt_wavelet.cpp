/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief ROS node to benchmark Dubins-airplane RRT* planning under Part 107
 * altitude limits, i.e. a literal Above-Ground-Level (AGL) floor/ceiling
 * (elevation directly below the vehicle plus a fixed offset) rather than a
 * true 3D clearance radius from nearby terrain.
 *
 * Identical in structure to test_rrt_circle_goal.cpp (same planner, same
 * loiter-circle start/goal handling, same visualization/logging) -- the only
 * difference is that "distance_surface"/"max_elevation" are built via
 * TerrainMap::AddLayerOffsetTransform instead of AddLayerDistanceTransform.
 * Kept as a separate benchmark node rather than changing
 * TerrainMap::AddLayerDistanceTransform's callers in terrain_navigation_ros,
 * so the deployed planner's existing 3D-clearance behavior is untouched.
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <any>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_geo_msgs/msg/quadtree_structure.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "terrain_navigation/data_logger.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

using namespace std::chrono_literals;

void publishCircleSetpoints(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                            const Eigen::Vector3d& position, const double radius, const double marker_size,
                            const Eigen::Vector3d& color) {
  visualization_msgs::msg::Marker circle_marker;
  circle_marker.header.stamp = rclcpp::Clock().now();
  circle_marker.header.frame_id = "map";
  circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle_marker.action = visualization_msgs::msg::Marker::ADD;
  circle_marker.id = 0;
  std::vector<geometry_msgs::msg::Point> points;
  for (double t = 0.0; t <= 1.0; t += 0.02) {
    geometry_msgs::msg::Point point;
    point.x = position.x() + radius * std::cos(t * 2 * M_PI);
    point.y = position.y() + radius * std::sin(t * 2 * M_PI);
    point.z = position.z();
    points.push_back(point);
  }
  points.push_back(points.front());  // close the loop back to the start
  circle_marker.points = points;
  // Line width scales with the loiter radius, not a fixed meter count, so
  // it stays visible whether the map is sertig's ~1.7km or wsmr's ~50km.
  circle_marker.scale.x = radius * 0.1;
  circle_marker.color.a = 1.0;
  circle_marker.color.r = static_cast<float>(color.x());
  circle_marker.color.g = static_cast<float>(color.y());
  circle_marker.color.b = static_cast<float>(color.z());
  circle_marker.pose.orientation.w = 1.0;
  pub->publish(circle_marker);

  // A solid sphere at the exact position, in addition to the loiter circle
  // above -- a thin circle outline is easy to miss at a glance on a
  // wide-area map, especially before zooming in. Sized from `marker_size`
  // (a fraction of the map's own extent, set by the caller), not `radius`
  // (the ~67m physical loiter turn radius) -- otherwise it's imperceptible
  // next to a many-km-wide map like wsmr's.
  visualization_msgs::msg::Marker center_marker = circle_marker;
  center_marker.id = 1;
  center_marker.type = visualization_msgs::msg::Marker::SPHERE;
  center_marker.points.clear();
  center_marker.pose.position.x = position.x();
  center_marker.pose.position.y = position.y();
  center_marker.pose.position.z = position.z();
  center_marker.scale.x = marker_size;
  center_marker.scale.y = marker_size;
  center_marker.scale.z = marker_size;
  pub->publish(center_marker);
}

bool validatePosition(std::shared_ptr<TerrainMap> map, const Eigen::Vector3d goal, Eigen::Vector3d& valid_goal) {
  double upper_surface = map->getGridMap().atPosition("distance_surface", goal.head(2));
  double lower_surface = map->getGridMap().atPosition("max_elevation", goal.head(2));
  const bool is_goal_valid = (upper_surface < lower_surface) ? true : false;
  valid_goal(0) = goal(0);
  valid_goal(1) = goal(1);
  valid_goal(2) = (upper_surface + lower_surface) / 2.0;
  return is_goal_valid;
}

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
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

PathSegment generateArcTrajectory(Eigen::Vector3d rate, const double horizon, Eigen::Vector3d current_pos,
                                  Eigen::Vector3d current_vel, const double dt = 0.1) {
  PathSegment trajectory;
  trajectory.states.clear();

  double cruise_speed_{20.0};

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

PathSegment getLoiterPath(Eigen::Vector3d end_position, Eigen::Vector3d end_velocity, Eigen::Vector3d center_pos) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  PathSegment loiter_trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  return loiter_trajectory;
}

void publishPathSegments(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, Path& trajectory) {
  visualization_msgs::msg::MarkerArray msg;

  std::vector<visualization_msgs::msg::Marker> marker;
  visualization_msgs::msg::Marker mark;
  mark.action = visualization_msgs::msg::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub->publish(msg);

  std::vector<visualization_msgs::msg::Marker> segment_markers;
  int i = 0;
  int segment_idx = 0;
  for (auto& segment : trajectory.segments) {
    Eigen::Vector3d color;
    if (segment_idx % 3 == 0) {  // Green is DUBINS_LEFT
      color = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else if (segment_idx % 3 == 1) {  // Blue is DUBINS_RIGHT
      color = Eigen::Vector3d(0.0, 0.0, 1.0);
    } else {
      color = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
    segment_idx++;
  }
  msg.markers = segment_markers;
  pub->publish(msg);
}

class Part107CirclePlanner : public rclcpp::Node {
 public:
  Part107CirclePlanner() : Node("rrt_part107_planner") {
    // Initialize ROS related publishers for visualization
    start_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("start_position", 1);
    goal_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("goal_position", 1);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 1);
    // transient_local: published once (see end of constructor), so a
    // subscriber (e.g. RViz) that connects slightly later still gets it.
    grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", rclcpp::QoS(1).transient_local());
    trajectory_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree", 1);
    path_segment_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_segments", 1);
    quadtree_structure_pub =
        this->create_publisher<grid_map_geo_msgs::msg::QuadtreeStructure>("quadtree_structure", 1);

    timer = this->create_wall_timer(1s, std::bind(&Part107CirclePlanner::timer_callback, this));

    // A directory containing elevation.wavelet_quadtree, variance.wavelet_quadtree,
    // and extent.txt (as written by generate_wavelet_quadtree.launch.py), not
    // a GeoTIFF -- see TerrainMap::LoadFromWaveletQuadtree.
    std::string map_path = this->declare_parameter("map_path", "");
    std::string location = this->declare_parameter("location", "");
    std::string output_directory = this->declare_parameter("output_directory", "");
    int query_height = this->declare_parameter("query_height", 0);
    min_agl_ = this->declare_parameter("min_agl", 50.0);
    max_agl_ = this->declare_parameter("max_agl", 120.0);

    // Initialize data logger for recording
    auto data_logger = std::make_shared<DataLogger>();
    data_logger->setKeys({"x", "y", "z"});

    // Load terrain map from a wavelet quadtree store. AddLayerOffsetTransform
    // below picks up TerrainMap's compression-error-bounded override
    // automatically, raising/lowering the AGL floor/ceiling by the store's
    // own getCompressionErrorBound() so it stays conservative relative to
    // the true, uncompressed terrain.
    // Materialize exactly the store's actual data footprint -- not
    // extent.txt's recorded bounding box, which for a large, sparsely
    // populated store (e.g. wsmr's ~43x113km) can be both far bigger than
    // the real coverage and off-center relative to it (a padded download
    // whose real survey area sits off to one side). See
    // WaveletTerrainMap::GetPopulatedExtent.
    terrain_map = std::make_shared<TerrainMap>();
    Eigen::Vector2d region_min, region_max;
    if (!TerrainMap::GetPopulatedExtent(map_path, region_min, region_max)) {
      throw std::runtime_error("Wavelet quadtree store at '" + map_path + "' has no populated data");
    }
    const Eigen::Vector2d region_center = (region_min + region_max) / 2.0;
    const grid_map::Length region_extent(region_max.x() - region_min.x(), region_max.y() - region_min.y());
    terrain_map->LoadFromWaveletQuadtree(map_path, region_center, region_extent, query_height);

    // The store doesn't change over the life of this node (no online
    // updateElevation()/checkpoint() calls here), so the QuadtreeStructure
    // message is built once, from the actual multi-resolution cells
    // (getElevationCells()) rather than the dense reconstruction -- this is
    // what lets it show the real compression structure (block/cell
    // boundaries) in RViz, not just the flat elevation surface already
    // published on "grid_map". Left uncolored: the QuadtreeStructure RViz
    // plugin falls back to coloring by cell size when no color is set.
    quadtree_structure_msg.header.frame_id = terrain_map->getGridMap().getFrameId();
    for (const auto& cell : terrain_map->getElevationCells()) {
      grid_map_geo_msgs::msg::QuadtreeCell cell_msg;
      cell_msg.min_corner.x = static_cast<float>(cell.min_corner.x());
      cell_msg.min_corner.y = static_cast<float>(cell.min_corner.y());
      cell_msg.min_corner.z = 0.0f;
      cell_msg.size = static_cast<float>(cell.size);
      cell_msg.elevation = cell.value;
      quadtree_structure_msg.cells.push_back(cell_msg);
    }

    // Part 107 altitude limits are AGL (elevation directly below the
    // vehicle plus a fixed offset), not a 3D clearance radius from nearby
    // terrain -- see TerrainMap::AddLayerOffsetTransform. The RRT solver's
    // own collision checker (TerrainValidityChecker::checkCollision) reads
    // "distance_surface"/"max_elevation" directly, so this is all it needs.
    //
    // No AddLayerHorizontalDistanceTransform("ics_+"/"ics_-") here: it's a
    // brute-force O(cells * radius^2) per-cell circle scan (every cell
    // scans every other cell within `radius`), only used below for the
    // start/goal validity margin -- fine for sertig's ~85k cells, but
    // becomes billions of inner iterations at wsmr's full-resolution scale.
    // validatePosition() checks distance_surface/max_elevation directly
    // instead, which loses the extra horizontal safety margin but not any
    // actual collision-checking fidelity.
    terrain_map->AddLayerOffsetTransform(min_agl_, "distance_surface");
    terrain_map->AddLayerOffsetTransform(max_agl_, "max_elevation");

    // Initialize planner with loaded terrain map
    planner = std::make_shared<TerrainOmplRrt>();
    planner->setMap(terrain_map);
    /// TODO: Get bounds from gridmap
    planner->setBoundsFromMap(terrain_map->getGridMap());

    const double map_width_x = terrain_map->getGridMap().getLength().x();
    const double map_width_y = terrain_map->getGridMap().getLength().y();
    const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();

    start = Eigen::Vector3d(map_pos(0) + 0.4 * map_width_x, map_pos(1) - 0.35 * map_width_y, 0.0);
    Eigen::Vector3d updated_start;
    if (validatePosition(terrain_map, start, updated_start)) {
      start = updated_start;
      std::cout << "Specified start position is valid" << std::endl;
    } else {
      throw std::runtime_error("Specified start position is NOT valid");
    }
    goal = Eigen::Vector3d(map_pos(0) - 0.3 * map_width_x, map_pos(1) + 0.3 * map_width_y, 0.0);
    Eigen::Vector3d updated_goal;
    if (validatePosition(terrain_map, goal, updated_goal)) {
      goal = updated_goal;
      std::cout << "Specified goal position is valid" << std::endl;
    } else {
      throw std::runtime_error("Specified goal position is NOT valid");
    }

    planner->setupProblem(start, goal);
    map_width_x_ = map_width_x;
    map_width_y_ = map_width_y;
    output_directory_ = output_directory;
    location_ = location;
    data_logger_ = data_logger;

    // The terrain map is static for the life of this node (no online
    // updateElevation()/checkpoint() calls here), and re-publishing a huge
    // dense grid_map every second is wasted bandwidth at wsmr's scale --
    // publish it once, latched (transient_local) so RViz still picks it up
    // even if it subscribes slightly after this. Independent of whether
    // planning ever succeeds.
    auto grid_map_message = grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap());
    grid_map_pub->publish(*grid_map_message);
  }

  // Solve() doesn't reset the tree between calls (only setupProblem() /
  // clear() does), so calling it again after a prior failed/approximate
  // attempt keeps extending the SAME tree rather than restarting -- driven
  // from timer_callback below so the search keeps retrying, budget by
  // budget, for as long as the node runs, instead of giving up after one
  // fixed-time attempt like the original single-shot version did.
  void attemptSolve() {
    if (solved_) return;
    const bool found = planner->Solve(kSolveBudgetSeconds, path);
    total_solve_time_ += planner->getSolutionTime();
    std::cout << "[TestRRTPart107] Attempt solve time: " << planner->getSolutionTime()
              << "s (total: " << total_solve_time_ << "s)" << std::endl;
    if (found) {
      solved_ = true;
      std::cout << "[TestRRTPart107] Found Solution!" << std::endl;

      Eigen::Vector3d start_position = path.firstSegment().states.front().position;
      Eigen::Vector3d start_velocity = path.firstSegment().states.front().velocity;
      PathSegment start_loiter_path = getLoiterPath(start_position, start_velocity, start);
      path.prependSegment(start_loiter_path);

      Eigen::Vector3d end_position = path.lastSegment().states.back().position;
      Eigen::Vector3d end_velocity = path.lastSegment().states.back().velocity;
      PathSegment goal_loiter_path = getLoiterPath(end_position, end_velocity, goal);
      path.appendSegment(goal_loiter_path);

      /// TODO: Save planned path into a csv file for plotting
      for (auto& point : path.position()) {
        std::unordered_map<std::string, std::any> state;
        state.insert(std::pair<std::string, double>("x", point(0) + 0.5 * map_width_x_));
        state.insert(std::pair<std::string, double>("y", point(1) + 0.5 * map_width_y_));
        state.insert(std::pair<std::string, double>("z", point(2)));
        data_logger_->record(state);
      }

      data_logger_->setPrintHeader(true);
      std::string output_file_path = output_directory_ + "/" + location_ + "_planned_path_part107.csv";
      data_logger_->writeToFile(output_file_path);
    } else {
      std::cout << "[TestRRTPart107] Unable to find solution, retrying..." << std::endl;
    }
  }

  void timer_callback() {
    std::cout << "Publishing results" << std::endl;
    attemptSolve();
    // Repeatedly publish results
    quadtree_structure_msg.header.stamp = this->now();
    quadtree_structure_pub->publish(quadtree_structure_msg);
    publishTrajectory(path_pub, path.position());
    publishPathSegments(path_segment_pub, path);

    /// TODO: Publish a circle instead of a goal marker!
    // 4% of the map's narrower dimension, so the marker stays a visible
    // fraction of the view regardless of whether the map is sertig-scale
    // (~1.7km) or wsmr-scale (~50km).
    const double marker_size = 0.04 * std::min(map_width_x_, map_width_y_);
    publishCircleSetpoints(start_pos_pub, start, radius, marker_size, Eigen::Vector3d(0.0, 1.0, 0.0));  // green
    publishCircleSetpoints(goal_pos_pub, goal, radius, marker_size, Eigen::Vector3d(1.0, 0.0, 0.0));    // red
    publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
  }

 private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_pos_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pos_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_segment_pub;
  rclcpp::Publisher<grid_map_geo_msgs::msg::QuadtreeStructure>::SharedPtr quadtree_structure_pub;
  rclcpp::TimerBase::SharedPtr timer;

  grid_map_geo_msgs::msg::QuadtreeStructure quadtree_structure_msg;
  std::shared_ptr<TerrainMap> terrain_map;
  std::shared_ptr<TerrainOmplRrt> planner;
  Path path;
  double radius = 66.667;
  double min_agl_{50.0};
  double max_agl_{120.0};
  Eigen::Vector3d start;
  Eigen::Vector3d goal;

  static constexpr double kSolveBudgetSeconds = 10.0;
  bool solved_{false};
  double total_solve_time_{0.0};
  double map_width_x_{0.0};
  double map_width_y_{0.0};
  std::string output_directory_;
  std::string location_;
  std::shared_ptr<DataLogger> data_logger_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto part107_circle_planner = std::make_shared<Part107CirclePlanner>();
  rclcpp::spin(part107_circle_planner);
  rclcpp::shutdown();
  return 0;
}
