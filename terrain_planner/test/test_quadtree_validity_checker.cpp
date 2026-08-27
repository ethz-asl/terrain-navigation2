#include <gtest/gtest.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/OwenStateSpace.h>

#include "terrain_planner/quadtree_validity_checker.h"

namespace {
constexpr double kGround = 100.0;
constexpr double kBump = 200.0;
constexpr double kMinAgl = 50.0;
constexpr double kMaxAgl = 120.0;

// 64x64m of flat terrain at kGround (world frame [1000, 1064)^2), with a
// kBump-high 4x4m block in its middle, i.e. at world [1030, 1034)^2.
std::shared_ptr<HashedWaveletQuadtree> makeTerrain() {
  auto tree = std::make_shared<HashedWaveletQuadtree>(6, 1.0);
  for (int i = 0; i < 64; i++) {
    for (int j = 0; j < 64; j++) {
      const bool is_bump = (i >= 30 && i < 34 && j >= 30 && j < 34);
      tree->setCellValue(Eigen::Vector2d(1000.5 + i, 1000.5 + j), is_bump ? kBump : kGround);
    }
  }
  return tree;
}
}  // namespace

class QuadtreeValidityCheckerTest : public ::testing::Test {
 protected:
  // The checker keeps a non-owning pointer to the space information, so the
  // fixture owns it.
  ompl::QuadtreeValidityChecker makeChecker(double error_margin, bool check_max_altitude = true) {
    // Local (0, 0) sits at world (1032, 1032), the corner shared by the bump's
    // four center cells.
    return ompl::QuadtreeValidityChecker(space_info_, makeTerrain(), Eigen::Vector2d(1032.0, 1032.0),
                                         Eigen::Vector2d(-32.0, -32.0), Eigen::Vector2d(32.0, 32.0), kMinAgl, kMaxAgl,
                                         error_margin, check_max_altitude);
  }

  ompl::base::SpaceInformationPtr space_info_{
      std::make_shared<ompl::base::SpaceInformation>(std::make_shared<ompl::base::OwenStateSpace>())};
};

TEST_F(QuadtreeValidityCheckerTest, agl_corridor) {
  const auto checker = makeChecker(0.0);
  const Eigen::Vector2d flat(-20.5, -20.5);
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMinAgl - 1.0)));
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMinAgl + 1.0)));
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMaxAgl - 1.0)));
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMaxAgl + 1.0)));
}

TEST_F(QuadtreeValidityCheckerTest, follows_terrain_in_world_frame) {
  const auto checker = makeChecker(0.0);
  // Valid above flat ground, but below the AGL floor over the bump.
  const double z = kGround + kMinAgl + 1.0;
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(-20.5, -20.5, z)));
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(0.5, 0.5, z)));
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(-1.5, -1.5, z)));
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(0.5, 0.5, kBump + kMinAgl + 1.0)));
}

TEST_F(QuadtreeValidityCheckerTest, error_margin_tightens_corridor) {
  const auto checker = makeChecker(3.0);
  const Eigen::Vector2d flat(-20.5, -20.5);
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMinAgl + 2.0)));
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMinAgl + 4.0)));
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMaxAgl - 2.0)));
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(flat.x(), flat.y(), kGround + kMaxAgl - 4.0)));
}

TEST_F(QuadtreeValidityCheckerTest, max_altitude_check_optional) {
  const auto checker = makeChecker(0.0, false);
  EXPECT_FALSE(checker.checkCollision(Eigen::Vector3d(-20.5, -20.5, kGround + kMaxAgl + 100.0)));
}

TEST_F(QuadtreeValidityCheckerTest, outside_bounds) {
  const auto checker = makeChecker(0.0);
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(40.0, 0.0, kGround + kMinAgl + 1.0)));
  EXPECT_TRUE(checker.checkCollision(Eigen::Vector3d(0.0, -40.0, kGround + kMinAgl + 1.0)));
}

TEST_F(QuadtreeValidityCheckerTest, is_valid_reads_owen_state) {
  const auto checker = makeChecker(0.0);
  ompl::base::ScopedState<ompl::base::OwenStateSpace> state(space_info_);
  state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = -20.5;
  state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = -20.5;
  state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = kGround + kMinAgl + 1.0;
  EXPECT_TRUE(checker.isValid(state.get()));
  state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = kGround + kMinAgl - 1.0;
  EXPECT_FALSE(checker.isValid(state.get()));
}
