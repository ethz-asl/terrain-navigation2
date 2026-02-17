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

#include "terrain_planner/terrain_ompl_rrt.h"

// Constructor
TerrainOmplRrt::TerrainOmplRrt(const double minTurnRadius, const double maxPitchAngle) {
  problem_setup_ = std::make_shared<ompl::OmplSetup>(
      ompl::base::StateSpacePtr(new ompl::base::OwenStateSpace(minTurnRadius, maxPitchAngle)));
}
TerrainOmplRrt::TerrainOmplRrt(const ompl::base::StateSpacePtr& space) {
  problem_setup_ = std::make_shared<ompl::OmplSetup>(space);
}
TerrainOmplRrt::~TerrainOmplRrt() {
  // Destructor
}

void TerrainOmplRrt::configureProblem() {
  problem_setup_->clear();
  problem_setup_->clearStartStates();

  problem_setup_->setDefaultPlanner();
  problem_setup_->setDefaultObjective();
  // assert(map);
  problem_setup_->setTerrainCollisionChecking(map_->getGridMap(), check_max_altitude_);
  // problem_setup_->getStateSpace()->setStateSamplerAllocator(
  //     std::bind(&TerrainOmplRrt::allocTerrainStateSampler, this, std::placeholders::_1));
  // problem_setup_->getStateSpace()->allocStateSampler();
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, lower_bound_.x());
  bounds.setLow(1, lower_bound_.y());
  bounds.setLow(2, lower_bound_.z());

  bounds.setHigh(0, upper_bound_.x());
  bounds.setHigh(1, upper_bound_.y());
  bounds.setHigh(2, upper_bound_.z());

  // Define start and goal positions.
  problem_setup_->getGeometricComponentStateSpace()->as<ompl::base::OwenStateSpace>()->setBounds(bounds);

  problem_setup_->setStateValidityCheckingResolution(0.001);

  planner_data_ = std::make_shared<ompl::base::PlannerData>(problem_setup_->getSpaceInformation());
}

void TerrainOmplRrt::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal,
                                  double start_loiter_radius) {
  configureProblem();
  /// TODO: FiXME
  double radius = problem_setup_->getStateSpace()->as<ompl::base::OwenStateSpace>()->getBounds().low[0];
  double delta_theta = 0.1;
  for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
    ompl::base::ScopedState<ompl::base::OwenStateSpace> start_ompl(problem_setup_->getSpaceInformation());

    start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] =
        start_pos(0) + std::abs(start_loiter_radius) * std::cos(theta);
    start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] =
        start_pos(1) + std::abs(start_loiter_radius) * std::sin(theta);
    start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = start_pos(2);
    double start_yaw = bool(start_loiter_radius > 0) ? theta - M_PI_2 : theta + M_PI_2;
    wrap_pi(start_yaw);
    start_ompl->yaw() = start_yaw;
    problem_setup_->addStartState(start_ompl);
  }

  goal_states_ = std::make_shared<ompl::base::GoalStates>(problem_setup_->getSpaceInformation());
  for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
    ompl::base::ScopedState<ompl::base::OwenStateSpace> goal_ompl(problem_setup_->getSpaceInformation());
    goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = goal(0) + radius * std::cos(theta);
    goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = goal(1) + radius * std::sin(theta);
    goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = goal(2);
    double goal_yaw = theta + M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->yaw() = goal_yaw;
    goal_states_->addState(goal_ompl);
    goal_yaw = theta - M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->yaw() = goal_yaw;
    goal_states_->addState(goal_ompl);  // Add additional state for bidirectional tangents
  }
  problem_setup_->setGoal(goal_states_);

  problem_setup_->setup();

  auto planner_ptr = problem_setup_->getPlanner();
  // std::cout << "Planner Range: " << planner_ptr->as<ompl::geometric::RRTstar>()->getRange() << std::endl;
}

void TerrainOmplRrt::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                  const Eigen::Vector3d& goal, double goal_radius) {
  configureProblem();

  double radius;
  if (goal_radius < 0) {
    radius = problem_setup_->getStateSpace()->as<ompl::base::OwenStateSpace>()->getBounds().low[0];
  } else {
    radius = goal_radius;
  }
  double delta_theta = 0.1;
  ompl::base::ScopedState<ompl::base::OwenStateSpace> start_ompl(problem_setup_->getSpaceInformation());

  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = start_pos(0);
  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = start_pos(1);
  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = start_pos(2);
  double start_yaw = std::atan2(start_vel(1), start_vel(0));
  start_ompl->yaw() = start_yaw;
  problem_setup_->clearStartStates();  // Clear previous goal states
  problem_setup_->addStartState(start_ompl);

  goal_states_ = std::make_shared<ompl::base::GoalStates>(problem_setup_->getSpaceInformation());
  for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
    ompl::base::ScopedState<ompl::base::OwenStateSpace> goal_ompl(problem_setup_->getSpaceInformation());
    goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = goal(0) + radius * std::cos(theta);
    goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = goal(1) + radius * std::sin(theta);
    goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = goal(2);
    double goal_yaw = theta + M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->yaw() = goal_yaw;
    goal_states_->addState(goal_ompl);
    goal_yaw = theta - M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->yaw() = goal_yaw;
    goal_states_->addState(goal_ompl);  // Add additional state for bidirectional tangents
  }
  problem_setup_->setGoal(goal_states_);

  problem_setup_->setup();
}

void TerrainOmplRrt::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                  const std::vector<Eigen::Vector3d>& goal_positions) {
  if (goal_positions.empty()) {
    std::cout << "Failed to configure problem: Goal position list empty" << std::endl;
    return;
  }
  configureProblem();

  double radius;
  double delta_theta = 0.1;
  ompl::base::ScopedState<ompl::base::OwenStateSpace> start_ompl(problem_setup_->getSpaceInformation());

  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = start_pos(0);
  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = start_pos(1);
  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = start_pos(2);
  double start_yaw = std::atan2(start_vel(1), start_vel(0));
  start_ompl->yaw() = start_yaw;
  problem_setup_->clearStartStates();  // Clear previous goal states
  problem_setup_->addStartState(start_ompl);

  goal_states_ = std::make_shared<ompl::base::GoalStates>(problem_setup_->getSpaceInformation());
  for (auto& goal : goal_positions) {
    for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
      ompl::base::ScopedState<ompl::base::OwenStateSpace> goal_ompl(problem_setup_->getSpaceInformation());
      goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = goal(0) + radius * std::cos(theta);
      goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = goal(1) + radius * std::sin(theta);
      goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = goal(2);
      double goal_yaw = theta + M_PI_2;
      wrap_pi(goal_yaw);
      goal_ompl->yaw() = goal_yaw;
      goal_states_->addState(goal_ompl);
      goal_yaw = theta - M_PI_2;
      wrap_pi(goal_yaw);
      goal_ompl->yaw() = goal_yaw;
      goal_states_->addState(goal_ompl);  // Add additional state for bidirectional tangents
    }
  }
  problem_setup_->setGoal(goal_states_);

  problem_setup_->setup();
}

void TerrainOmplRrt::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                  const Eigen::Vector3d& goal, const Eigen::Vector3d& goal_vel) {
  configureProblem();

  ompl::base::ScopedState<ompl::base::OwenStateSpace> start_ompl(problem_setup_->getSpaceInformation());
  ompl::base::ScopedState<ompl::base::OwenStateSpace> goal_ompl(problem_setup_->getSpaceInformation());

  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = start_pos(0);
  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = start_pos(1);
  start_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = start_pos(2);
  double start_yaw = std::atan2(start_vel(1), start_vel(0));
  start_ompl->yaw() = start_yaw;

  goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = goal(0);
  goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = goal(1);
  goal_ompl->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = goal(2);
  double goal_yaw = std::atan2(goal_vel(1), goal_vel(0));
  goal_ompl->yaw() = goal_yaw;

  problem_setup_->setStartAndGoalStates(start_ompl, goal_ompl);
  problem_setup_->setup();
}

void TerrainOmplRrt::setBoundsFromMap(const grid_map::GridMap& map) {
  const Eigen::Vector2d map_pos = map.getPosition();
  /// TODO: Iterate through map to get elevation bounds

  double min_elevation = std::numeric_limits<double>::max();
  double max_elevation = std::numeric_limits<double>::min();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index map_index = *iterator;
    const double minimum_elevation_limit = map.at("distance_surface", map_index);
    if (minimum_elevation_limit < min_elevation) {
      min_elevation = minimum_elevation_limit;
    }
    const double maximum_elevation_limit = map.at("max_elevation", map_index);
    if (maximum_elevation_limit > max_elevation) {
      max_elevation = maximum_elevation_limit;
    }
  }

  const double map_width_x = map.getLength().x();
  const double map_width_y = map.getLength().y();
  double roi_ratio = 0.5;
  Eigen::Vector3d lower_bounds{
      Eigen::Vector3d(map_pos(0) - roi_ratio * map_width_x, map_pos(1) - roi_ratio * map_width_y, min_elevation)};
  Eigen::Vector3d upper_bounds{
      Eigen::Vector3d(map_pos(0) + roi_ratio * map_width_x, map_pos(1) + roi_ratio * map_width_y, max_elevation)};
  std::cout << "[TerrainOmplRrt] Upper bounds: " << upper_bounds.transpose() << std::endl;
  std::cout << "[TerrainOmplRrt] Lower bounds: " << lower_bounds.transpose() << std::endl;
  setBounds(lower_bounds, upper_bounds);
}

bool TerrainOmplRrt::Solve(double time_budget, Path& path) {
  if (problem_setup_->solve(time_budget)) {
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    // problem_setup_.getSolutionPath().print(std::cout);
    problem_setup_->getPlannerData(*planner_data_);
    solve_duration_ = problem_setup_->getLastPlanComputationTime();

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_->haveExactSolutionPath()) {
    std::cout << "Found Exact solution!" << std::endl;
    solutionPathToPath(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

bool TerrainOmplRrt::Solve(double time_budget, std::vector<Eigen::Vector3d>& path) {
  if (problem_setup_->solve(time_budget)) {
    std::cout << "Found solution:" << std::endl;
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    // problem_setup_.getSolutionPath().print(std::cout);

    problem_setup_->getPlannerData(*planner_data_);
    solve_duration_ = problem_setup_->getLastPlanComputationTime();

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_->haveExactSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

bool TerrainOmplRrt::getSolutionPathLength(double& path_length) {
  if (problem_setup_->haveExactSolutionPath()) {
    ompl::geometric::PathGeometric path = problem_setup_->getSolutionPath();
    path.interpolate();
    path_length = path.length();
    return true;
  }
  return false;
}

bool TerrainOmplRrt::getSolutionPath(std::vector<Eigen::Vector3d>& path) {
  if (problem_setup_->haveExactSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

double TerrainOmplRrt::getSegmentCurvature(const ompl::base::OwenStateSpace::PathType& path, int segment_index) const {
  double turn_radius = path.turnRadius_;
  if (turn_radius <= 0.0) {
    return 0.0;
  }

  // Get the segment type from the Dubins path
  const auto& segment_types = *(path.path_.type_);
  if (segment_index < 0 || segment_index >= static_cast<int>(segment_types.size())) {
    return 0.0;
  }

  switch (segment_types[segment_index]) {
    case ompl::base::DubinsStateSpace::DUBINS_LEFT:
      return 1.0 / turn_radius;
    case ompl::base::DubinsStateSpace::DUBINS_RIGHT:
      return -1.0 / turn_radius;
    case ompl::base::DubinsStateSpace::DUBINS_STRAIGHT:
    default:
      return 0.0;
  }
}

PathSegment TerrainOmplRrt::extractPathSegment(ompl::base::State* from, ompl::base::State* to,
                                               ompl::base::OwenStateSpace::PathType& path, double t_start, double t_end,
                                               double curvature, double horizontal_length, double dt) const {
  ompl::base::State* state = problem_setup_->getStateSpace()->allocState();
  PathSegment trajectory;
  trajectory.curvature = curvature;

  // Get start position for flightpath_angle calculation
  problem_setup_->getStateSpace()->as<ompl::base::OwenStateSpace>()->interpolate(from, to, t_start, path, state);
  Eigen::Vector3d start_position = dubinsairplanePosition(state);

  for (double t = t_start; t <= t_end; t += dt) {
    // Append to trajectory
    problem_setup_->getStateSpace()->as<ompl::base::OwenStateSpace>()->interpolate(from, to, t, path, state);
    State segment_state;
    segment_state.position = dubinsairplanePosition(state);
    double yaw = dubinsairplaneYaw(state);
    Eigen::Vector3d velocity = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
    segment_state.velocity = velocity;
    trajectory.states.emplace_back(segment_state);
  }
  // Append end to trajectory
  problem_setup_->getStateSpace()->as<ompl::base::OwenStateSpace>()->interpolate(from, to, t_end, path, state);
  State segment_end_state;
  segment_end_state.position = dubinsairplanePosition(state);
  double end_yaw = dubinsairplaneYaw(state);
  Eigen::Vector3d end_velocity = Eigen::Vector3d(std::cos(end_yaw), std::sin(end_yaw), 0.0);
  segment_end_state.velocity = end_velocity;
  trajectory.states.emplace_back(segment_end_state);

  // Compute flightpath_angle from altitude change over 2D Dubins path length
  Eigen::Vector3d end_position = segment_end_state.position;
  double dz = end_position(2) - start_position(2);
  if (horizontal_length > 1e-6) {
    trajectory.flightpath_angle = std::atan2(dz, horizontal_length);
  } else {
    trajectory.flightpath_angle = 0.0;
  }

  problem_setup_->getStateSpace()->freeState(state);
  return trajectory;
}

void TerrainOmplRrt::solutionPathToPath(ompl::geometric::PathGeometric path, Path& trajectory_segments,
                                        double resolution) const {
  trajectory_segments.segments.clear();

  std::vector<ompl::base::State*>& state_vector = path.getStates();
  for (size_t idx = 0; idx < state_vector.size() - 1; idx++) {
    auto from = state_vector[idx];    // Start of the segment
    auto to = state_vector[idx + 1];  // End of the segment
    auto dubins_path = problem_setup_->getStateSpace()->as<ompl::base::OwenStateSpace>()->getPath(from, to);

    if (dubins_path->phi_ == 0.) {
      if (dubins_path->numTurns_ == 0) {
        // Low path case
        double t_start{0.0};
        double t_end{0.0};
        double length = dubins_path->path_.length();
        for (int segment_idx = 0; segment_idx < 3; segment_idx++) {
          t_end = segment_idx == 2 ? 1.0 : dubins_path->path_.length_[segment_idx] / length + t_start;
          double segment_curvature = getSegmentCurvature(*dubins_path, segment_idx);
          // Actual horizontal length = turnRadius * normalized length
          double segment_horizontal_length = dubins_path->turnRadius_ * dubins_path->path_.length_[segment_idx];
          std::cout << "t_start: " << t_start << " t_end: " << t_end << std::endl;
          auto trajectory =
              extractPathSegment(from, to, *dubins_path, t_start, t_end, segment_curvature, segment_horizontal_length);
          if (trajectory.states.size() > 1) {
            trajectory_segments.segments.push_back(trajectory);
          }
          t_start = t_end;
        }
      } else {
        // high altitude path

        // Parse Trochoidal periodic paths
        // lengthPeriodicPath is already in actual units (2*pi*turnRadius)
        double lengthPeriodicPath = 2.0 * M_PI * dubins_path->turnRadius_;
        // lengthPath needs to be scaled by turnRadius
        auto lengthPath = dubins_path->turnRadius_ * dubins_path->path_.length();
        auto lengthTotal = lengthPath + lengthPeriodicPath * dubins_path->numTurns_;
        double t_start, t_end;
        // Curvature for the periodic helical turns - direction based on first segment type
        double periodic_curvature = getSegmentCurvature(*dubins_path, 0);
        for (int k = 0; k < static_cast<int>(dubins_path->numTurns_); k++) {
          t_start = lengthPeriodicPath * k / lengthTotal;
          t_end = lengthPeriodicPath * (k + 1) / lengthTotal;
          auto trajectory =
              extractPathSegment(from, to, *dubins_path, t_start, t_end, periodic_curvature, lengthPeriodicPath);
          if (trajectory.states.size() > 1) {
            trajectory_segments.segments.push_back(trajectory);
          }
        }
        // Remaining Dubins path - extract each segment individually
        double dubins_t_start = lengthPeriodicPath * dubins_path->numTurns_ / lengthTotal;
        for (int segment_idx = 0; segment_idx < 3; segment_idx++) {
          double segment_horizontal_length = dubins_path->turnRadius_ * dubins_path->path_.length_[segment_idx];
          double segment_length_ratio = segment_horizontal_length / lengthTotal;
          double dubins_t_end = segment_idx == 2 ? 1.0 : dubins_t_start + segment_length_ratio;
          double segment_curvature = getSegmentCurvature(*dubins_path, segment_idx);
          auto trajectory = extractPathSegment(from, to, *dubins_path, dubins_t_start, dubins_t_end, segment_curvature,
                                               segment_horizontal_length);
          if (trajectory.states.size() > 1) {
            trajectory_segments.segments.push_back(trajectory);
          }
          dubins_t_start = dubins_t_end;
        }
      }
    } else {
      // medium altitude path
      auto lengthTurn = dubins_path->turnRadius_ * std::abs(dubins_path->phi_);
      auto lengthPath = dubins_path->turnRadius_ * dubins_path->path_.length();
      auto lengthTotal = lengthTurn + lengthPath;
      {
        // Initial turn curvature is determined by sign of phi_, using actual turn radius
        double initial_turn_curvature = (dubins_path->phi_ > 0.0 ? 1.0 : -1.0) / dubins_path->turnRadius_;
        auto trajectory = extractPathSegment(from, to, *dubins_path, 0.0, lengthTurn / lengthTotal,
                                             initial_turn_curvature, lengthTurn);
        if (trajectory.states.size() > 1) {
          trajectory_segments.segments.push_back(trajectory);
        }
      }
      // Remaining Dubins path - extract each segment individually
      double dubins_t_start = lengthTurn / lengthTotal;
      for (int segment_idx = 0; segment_idx < 3; segment_idx++) {
        double segment_horizontal_length = dubins_path->turnRadius_ * dubins_path->path_.length_[segment_idx];
        double segment_length_ratio = segment_horizontal_length / lengthTotal;
        double dubins_t_end = segment_idx == 2 ? 1.0 : dubins_t_start + segment_length_ratio;
        double segment_curvature = getSegmentCurvature(*dubins_path, segment_idx);
        auto trajectory = extractPathSegment(from, to, *dubins_path, dubins_t_start, dubins_t_end, segment_curvature,
                                             segment_horizontal_length);
        if (trajectory.states.size() > 1) {
          trajectory_segments.segments.push_back(trajectory);
        }
        dubins_t_start = dubins_t_end;
      }
    }
  }
}

void TerrainOmplRrt::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
                                                    std::vector<Eigen::Vector3d>& trajectory_points) const {
  trajectory_points.clear();
  path.interpolate();

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    auto position = dubinsairplanePosition(state_ptr);
    trajectory_points.emplace_back(position);
  }
}
