#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "convex_plane_decomposition/PlaneDecompositionPipeline.h"

namespace convex_plane_decomposition {
namespace {

PlaneDecompositionPipeline::Config testConfig() {
  PlaneDecompositionPipeline::Config config;
  config.preprocessingParameters.resolution = -1.0;
  config.preprocessingParameters.kernelSize = 3;
  config.preprocessingParameters.numberOfRepeats = 1;
  config.slidingWindowPlaneExtractorParameters.kernel_size = 3;
  config.slidingWindowPlaneExtractorParameters.min_number_points_per_label = 6;
  config.slidingWindowPlaneExtractorParameters.plane_inclination_threshold = std::cos(80.0 * M_PI / 180.0);
  config.slidingWindowPlaneExtractorParameters.local_plane_inclination_threshold = std::cos(80.0 * M_PI / 180.0);
  config.slidingWindowPlaneExtractorParameters.plane_patch_error_threshold = 0.005;
  config.slidingWindowPlaneExtractorParameters.include_ransac_refinement = false;
  config.contourExtractionParameters.marginSize = 0;
  return config;
}

grid_map::GridMap makePlane(double slopeX = 0.0, double slopeY = 0.0) {
  grid_map::GridMap map({"elevation"});
  map.setFrameId("test_map");
  map.setTimestamp(123456789ULL);
  map.setGeometry(grid_map::Length(1.2, 1.0), 0.05, grid_map::Position(0.6, 0.0));
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    map.at("elevation", *iterator) = static_cast<float>(0.2 + slopeX * position.x() + slopeY * position.y());
  }
  return map;
}

void expectFiniteBoundaries(const PlanarTerrain& terrain) {
  for (const auto& region : terrain.planarRegions) {
    const auto& transform = region.transformPlaneToWorld;
    EXPECT_TRUE(transform.matrix().allFinite());
    for (const auto& point : region.boundaryWithInset.boundary.outer_boundary()) {
      EXPECT_TRUE(std::isfinite(point.x()));
      EXPECT_TRUE(std::isfinite(point.y()));
    }
  }
}

TEST(TestSyntheticPlaneExtraction, SinglePlane) {
  PlaneDecompositionPipeline pipeline(testConfig());
  pipeline.update(makePlane(), "elevation");
  const auto& terrain = pipeline.getPlanarTerrain();
  ASSERT_EQ(terrain.planarRegions.size(), 1U);
  EXPECT_GT(terrain.planarRegions.front().transformPlaneToWorld.linear().col(2).z(), 0.999);
  EXPECT_EQ(terrain.gridMap.getTimestamp(), 123456789ULL);
  EXPECT_EQ(terrain.gridMap.getFrameId(), "test_map");
  expectFiniteBoundaries(terrain);
}

TEST(TestSyntheticPlaneExtraction, KnownNormalPlane) {
  constexpr double slopeX = 0.15;
  constexpr double slopeY = -0.10;
  PlaneDecompositionPipeline pipeline(testConfig());
  pipeline.update(makePlane(slopeX, slopeY), "elevation");
  const auto& terrain = pipeline.getPlanarTerrain();
  ASSERT_EQ(terrain.planarRegions.size(), 1U);
  const Eigen::Vector3d expected = Eigen::Vector3d(-slopeX, -slopeY, 1.0).normalized();
  const Eigen::Vector3d actual = terrain.planarRegions.front().transformPlaneToWorld.linear().col(2);
  EXPECT_GT(actual.dot(expected), 0.999);
}

TEST(TestSyntheticPlaneExtraction, TwoSeparatedPlanes) {
  auto map = makePlane();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    if (position.x() > 0.55 && position.x() < 0.65) {
      map.at("elevation", *iterator) = static_cast<float>(2.0 * (position.x() - 0.55));
    } else if (position.x() >= 0.65) {
      map.at("elevation", *iterator) = 0.4F;
    }
  }
  PlaneDecompositionPipeline pipeline(testConfig());
  pipeline.update(std::move(map), "elevation");
  EXPECT_GE(pipeline.getPlanarTerrain().planarRegions.size(), 2U);
  expectFiniteBoundaries(pipeline.getPlanarTerrain());
}

TEST(TestSyntheticPlaneExtraction, RejectsEmptyAndAllNanMaps) {
  PlaneDecompositionPipeline pipeline(testConfig());
  grid_map::GridMap empty;
  EXPECT_THROW(pipeline.update(std::move(empty), "elevation"), std::invalid_argument);

  grid_map::GridMap allNan({"elevation"});
  allNan.setGeometry(grid_map::Length(0.5, 0.5), 0.05);
  allNan.get("elevation").setConstant(std::numeric_limits<float>::quiet_NaN());
  EXPECT_THROW(pipeline.update(std::move(allNan), "elevation"), std::invalid_argument);
}

TEST(TestSyntheticPlaneExtraction, HandlesInvalidCellsWithoutNonfiniteBoundaries) {
  auto map = makePlane();
  map.get("elevation").block(5, 5, 3, 4).setConstant(std::numeric_limits<float>::quiet_NaN());
  PlaneDecompositionPipeline pipeline(testConfig());
  EXPECT_NO_THROW(pipeline.update(std::move(map), "elevation"));
  ASSERT_FALSE(pipeline.getPlanarTerrain().planarRegions.empty());
  expectFiniteBoundaries(pipeline.getPlanarTerrain());
}

TEST(TestSyntheticPlaneExtraction, RestrictsRegionsToObservedSupportAfterResampling) {
  auto config = testConfig();
  config.preprocessingParameters.resolution = 0.1;
  auto map = makePlane();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    if (position.x() < 0.2 || position.x() > 1.0 || position.y() < -0.35 || position.y() > 0.35) {
      map.at("elevation", *iterator) = std::numeric_limits<float>::quiet_NaN();
    }
  }

  PlaneDecompositionPipeline pipeline(config);
  pipeline.update(std::move(map), "elevation");
  const auto& terrain = pipeline.getPlanarTerrain();
  ASSERT_FALSE(terrain.planarRegions.empty());
  for (const auto& region : terrain.planarRegions) {
    for (const auto& point : region.boundaryWithInset.boundary.outer_boundary()) {
      const Eigen::Vector3d world = positionInWorldFrameFromPosition2dInPlane(point, region.transformPlaneToWorld);
      EXPECT_GE(world.x(), 0.15);
      EXPECT_LE(world.x(), 1.05);
      EXPECT_GE(world.y(), -0.40);
      EXPECT_LE(world.y(), 0.40);
    }
  }
}

TEST(TestSyntheticPlaneExtraction, BoundariesRemainInsideTruncatedMap) {
  PlaneDecompositionPipeline pipeline(testConfig());
  pipeline.update(makePlane(), "elevation");
  const auto& terrain = pipeline.getPlanarTerrain();
  ASSERT_FALSE(terrain.planarRegions.empty());
  expectFiniteBoundaries(terrain);
  for (const auto& region : terrain.planarRegions) {
    for (const auto& point : region.boundaryWithInset.boundary.outer_boundary()) {
      const Eigen::Vector3d world = positionInWorldFrameFromPosition2dInPlane(point, region.transformPlaneToWorld);
      EXPECT_GE(world.x(), -1e-6);
      EXPECT_LE(world.x(), 1.2 + 1e-6);
      EXPECT_GE(world.y(), -0.5 - 1e-6);
      EXPECT_LE(world.y(), 0.5 + 1e-6);
    }
  }
}

}  // namespace
}  // namespace convex_plane_decomposition
