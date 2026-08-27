#ifndef TERRAIN_PLANNER_QUADTREE_VALIDITY_CHECKER_H
#define TERRAIN_PLANNER_QUADTREE_VALIDITY_CHECKER_H

#include <ompl/base/StateValidityChecker.h>

#include <Eigen/Dense>
#include <grid_map_geo_wavelet_quadtree/hashed_wavelet_quadtree.hpp>
#include <memory>

namespace ompl {

/**
 * @brief State validity checker that queries a HashedWaveletQuadtree
 * elevation store directly, instead of precomputed "distance_surface" /
 * "max_elevation" layers of a dense grid_map (see TerrainValidityChecker).
 *
 * Enforces the same Above-Ground-Level (AGL) corridor as
 * TerrainMap::AddLayerOffsetTransform: a state is valid iff
 *   elevation + min_agl + error_margin <= z <= elevation + max_agl -
 * error_margin where `elevation` is the tree's finest-resolution value directly
 * below the state. Each check is an O(tree height) lookup, so neither memory
 * nor setup cost scales with the planning region's area, and collision checking
 * stays at full resolution even if the dense grid_map is materialized coarsely
 * (e.g. only for visualization).
 */
class QuadtreeValidityChecker : public base::StateValidityChecker {
 public:
  /**
   * @param space_info OMPL space information.
   * @param elevation_tree world-frame indexed elevation store.
   * @param world_origin world-frame position of the planner's local (0, 0),
   * i.e. added to each state's (x, y) before querying the tree.
   * @param local_min local-frame min (x, y) corner of the valid region;
   * states outside [local_min, local_max] are invalid.
   * @param local_max local-frame max (x, y) corner of the valid region.
   * @param min_agl minimum height above terrain.
   * @param max_agl maximum height above terrain.
   * @param error_margin tightens both limits, e.g. by the store's compression
   * error bound (WaveletTerrainMap::getCompressionErrorBound()) so the
   * corridor stays conservative relative to the true terrain.
   * @param check_max_altitude if false, max_agl is not enforced.
   */
  QuadtreeValidityChecker(const base::SpaceInformationPtr& space_info,
                          std::shared_ptr<const HashedWaveletQuadtree> elevation_tree,
                          const Eigen::Vector2d& world_origin, const Eigen::Vector2d& local_min,
                          const Eigen::Vector2d& local_max, double min_agl, double max_agl, double error_margin,
                          bool check_max_altitude = true);

  bool isValid(const base::State* state) const override;

  /**
   * @brief Check collisions against the quadtree
   *
   * @param position local-frame position
   * @return true in collision (or outside the valid region)
   */
  bool checkCollision(const Eigen::Vector3d& position) const;

 protected:
  std::shared_ptr<const HashedWaveletQuadtree> elevation_tree_;
  Eigen::Vector2d world_origin_;
  Eigen::Vector2d local_min_;
  Eigen::Vector2d local_max_;
  double min_clearance_;
  double max_clearance_;
  bool check_collision_max_altitude_{true};
};

}  // namespace ompl

#endif
