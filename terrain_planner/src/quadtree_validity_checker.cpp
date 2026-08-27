#include "terrain_planner/quadtree_validity_checker.h"

#include <ompl/base/spaces/OwenStateSpace.h>

namespace ompl {

QuadtreeValidityChecker::QuadtreeValidityChecker(const base::SpaceInformationPtr& space_info,
                                                 std::shared_ptr<const HashedWaveletQuadtree> elevation_tree,
                                                 const Eigen::Vector2d& world_origin, const Eigen::Vector2d& local_min,
                                                 const Eigen::Vector2d& local_max, double min_agl, double max_agl,
                                                 double error_margin, bool check_max_altitude)
    : base::StateValidityChecker(space_info),
      elevation_tree_(std::move(elevation_tree)),
      world_origin_(world_origin),
      local_min_(local_min),
      local_max_(local_max),
      min_clearance_(min_agl + error_margin),
      max_clearance_(max_agl - error_margin),
      check_collision_max_altitude_(check_max_altitude) {}

bool QuadtreeValidityChecker::isValid(const base::State* state) const {
  const auto* position_state =
      state->as<ompl::base::OwenStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const Eigen::Vector3d position(position_state->values[0], position_state->values[1], position_state->values[2]);
  return !checkCollision(position);
}

bool QuadtreeValidityChecker::checkCollision(const Eigen::Vector3d& position) const {
  const Eigen::Vector2d position_2d = position.head<2>();
  if ((position_2d.array() < local_min_.array()).any() || (position_2d.array() > local_max_.array()).any()) {
    return true;  // Do not allow vehicle to go outside the map
  }
  const double elevation = elevation_tree_->getCellValue(position_2d + world_origin_);
  if (position.z() < elevation + min_clearance_) return true;
  if (check_collision_max_altitude_ && position.z() > elevation + max_clearance_) return true;
  return false;
}

}  // namespace ompl
