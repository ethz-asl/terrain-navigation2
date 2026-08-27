#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "grid_map_geo_wavelet_quadtree/hashed_wavelet_quadtree.hpp"
#include "terrain_navigation/terrain_map.h"

namespace {

// Builds a small, perfectly flat on-disk wavelet quadtree store (elevation +
// variance) under `dir`, with an extent.txt recording `error_bound` as the
// compression tolerance. Flat terrain makes AddLayerDistanceTransform's
// geometric offset degenerate to a plain vertical shift, so the expected
// "distance_surface"/"max_elevation" values are exact, not approximate.
void buildFlatWaveletQuadtreeStore(const std::string& dir, const Eigen::Vector2d& center, float flat_elevation,
                                   double error_bound) {
  std::filesystem::create_directories(dir);
  HashedWaveletQuadtree elevation(6, 1.0, flat_elevation);
  HashedWaveletQuadtree variance(6, 1.0, 1.0f);
  for (int ix = -10; ix < 10; ++ix) {
    for (int iy = -10; iy < 10; ++iy) {
      const Eigen::Vector2d world_pos = center + Eigen::Vector2d(ix + 0.5, iy + 0.5);
      elevation.setCellValue(world_pos, flat_elevation);
      variance.setCellValue(world_pos, 1.0f);
    }
  }
  ASSERT_TRUE(elevation.saveToFile(dir + "/elevation.wavelet_quadtree"));
  ASSERT_TRUE(variance.saveToFile(dir + "/variance.wavelet_quadtree"));
  std::ofstream extent_file(dir + "/extent.txt");
  extent_file << center.x() << " " << center.y() << " " << 16.0 << " " << 16.0 << " " << error_bound << "\n";
}

}  // namespace

TEST(TerrainMapCompressionBoundTest, DistanceTransformIsUnaffectedWhenLossless) {
  const std::string dir = "/tmp/test_terrain_map_compression_bound_lossless";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(0.0, 0.0);
  buildFlatWaveletQuadtreeStore(dir, center, 100.0f, /*error_bound=*/0.0);

  TerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(10.0, 10.0), 0));
  ASSERT_NEAR(map.getCompressionErrorBound(), 0.0, 1e-9);

  map.AddLayerDistanceTransform(5.0, "distance_surface");
  map.AddLayerDistanceTransform(20.0, "max_elevation");
  EXPECT_NEAR(map.getGridMap().atPosition("distance_surface", Eigen::Vector2d(0.0, 0.0)), 105.0, 1e-2);
  EXPECT_NEAR(map.getGridMap().atPosition("max_elevation", Eigen::Vector2d(0.0, 0.0)), 120.0, 1e-2);
}

TEST(TerrainMapCompressionBoundTest, DistanceSurfaceFloorIsRaisedByCompressionErrorBound) {
  const std::string dir = "/tmp/test_terrain_map_compression_bound_floor";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(0.0, 0.0);
  buildFlatWaveletQuadtreeStore(dir, center, 100.0f, /*error_bound=*/3.0);

  TerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(10.0, 10.0), 0));
  ASSERT_NEAR(map.getCompressionErrorBound(), 3.0, 1e-9);

  map.AddLayerDistanceTransform(5.0, "distance_surface");
  // A reconstructed elevation that underestimates the true terrain by up to
  // the error bound would make an unbounded floor unsafe, so the floor must
  // be raised by exactly the error bound: 100 + 3 (bound) + 5 (clearance).
  EXPECT_NEAR(map.getGridMap().atPosition("distance_surface", Eigen::Vector2d(0.0, 0.0)), 108.0, 1e-2);
}

TEST(TerrainMapCompressionBoundTest, MaxElevationCeilingIsLoweredByCompressionErrorBound) {
  const std::string dir = "/tmp/test_terrain_map_compression_bound_ceiling";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(0.0, 0.0);
  buildFlatWaveletQuadtreeStore(dir, center, 100.0f, /*error_bound=*/3.0);

  TerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(10.0, 10.0), 0));

  map.AddLayerDistanceTransform(20.0, "max_elevation");
  // A reconstructed elevation that overestimates the true terrain by up to
  // the error bound would make an unbounded ceiling unsafe, so the ceiling
  // must be lowered by exactly the error bound: 100 - 3 (bound) + 20 (limit).
  EXPECT_NEAR(map.getGridMap().atPosition("max_elevation", Eigen::Vector2d(0.0, 0.0)), 117.0, 1e-2);
}

TEST(TerrainMapCompressionBoundTest, OffsetTransformIgnoresNearbyTerrainUnlikeDistanceTransform) {
  const std::string dir = "/tmp/test_terrain_map_offset_vs_distance_transform";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(0.0, 0.0);
  buildFlatWaveletQuadtreeStore(dir, center, 100.0f, /*error_bound=*/0.0);

  TerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(10.0, 10.0), 0));

  // Raise one nearby cell into a spike, close enough to fall inside the
  // surface_distance=5 clearance radius used below (3 m horizontal offset).
  grid_map::Index spike_index;
  ASSERT_TRUE(map.getGridMap().getIndex(Eigen::Vector2d(3.0, 0.0), spike_index));
  map.getGridMap().at("elevation", spike_index) = 200.0f;

  // AGL: only the elevation directly below the query point matters, so the
  // nearby spike has no effect at (0, 0).
  map.AddLayerOffsetTransform(5.0, "distance_surface");
  EXPECT_NEAR(map.getGridMap().atPosition("distance_surface", Eigen::Vector2d(0.0, 0.0)), 105.0, 1e-2);

  // True 3D clearance: the spike falls within the 5 m clearance radius, so
  // the floor at (0, 0) must be raised to keep a genuine 5 m sphere clear of
  // it -- well above the AGL-only value.
  map.AddLayerDistanceTransform(5.0, "distance_surface");
  EXPECT_GT(map.getGridMap().atPosition("distance_surface", Eigen::Vector2d(0.0, 0.0)), 150.0);
}

TEST(TerrainMapCompressionBoundTest, UnrecognizedLayerNameFallsBackToUnmodifiedTransform) {
  const std::string dir = "/tmp/test_terrain_map_compression_bound_other_layer";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(0.0, 0.0);
  buildFlatWaveletQuadtreeStore(dir, center, 100.0f, /*error_bound=*/3.0);

  TerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(10.0, 10.0), 0));

  map.AddLayerDistanceTransform(5.0, "some_other_surface");
  EXPECT_NEAR(map.getGridMap().atPosition("some_other_surface", Eigen::Vector2d(0.0, 0.0)), 105.0, 1e-2);
}
