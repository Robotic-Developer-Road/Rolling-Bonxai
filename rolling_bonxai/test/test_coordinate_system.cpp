#include <gtest/gtest.h>
#include "rolling_bonxai/coordinate_system.hpp"

using namespace RollingBonxai;

// ============================================================================
// ChunkCoordinateSystem Tests
// ============================================================================

TEST(ChunkCoordinateSystem, OriginMapsToZeroChunk) {
    ChunkCoordinateSystem cs(0.2,10.0);
    auto chunk = cs.positionToChunkCoordinate(0.0, 0.0, 0.0);
    
    EXPECT_EQ(chunk.x, 0);
    EXPECT_EQ(chunk.y, 0);
    EXPECT_EQ(chunk.z, 0);
}

TEST(ChunkCoordinateSystem, ChunkCenters) {
    ChunkCoordinateSystem cs(0.2,10.0);
    
    auto pos = cs.chunkToPositionCoordinate(ChunkCoord(1, 0, 0));
    EXPECT_DOUBLE_EQ(pos.x(), 10.0);
    EXPECT_DOUBLE_EQ(pos.y(), 0.0);
    EXPECT_DOUBLE_EQ(pos.z(), 0.0);
    
    auto pos2 = cs.chunkToPositionCoordinate(ChunkCoord(-2, 3, -1));
    EXPECT_DOUBLE_EQ(pos2.x(), -20.0);
    EXPECT_DOUBLE_EQ(pos2.y(), 30.0);
    EXPECT_DOUBLE_EQ(pos2.z(), -10.0);
}

TEST(ChunkCoordinateSystem, BoundaryBehavior) {
    ChunkCoordinateSystem cs(0.2,10.0);
    
    // Position exactly on positive boundary (half-open: [min, max))
    auto chunk1 = cs.positionToChunkCoordinate(5.0, 0.0, 0.0);
    EXPECT_EQ(chunk1.x, 1);  // round(0.5) = 1
    
    // Position just inside lower boundary
    auto chunk2 = cs.positionToChunkCoordinate(4.99, 0.0, 0.0);
    EXPECT_EQ(chunk2.x, 0);
    
    // Position just inside upper boundary
    auto chunk3 = cs.positionToChunkCoordinate(-4.99, 0.0, 0.0);
    EXPECT_EQ(chunk3.x, 0);
    
    // Position on negative boundary
    auto chunk4 = cs.positionToChunkCoordinate(-5.0, 0.0, 0.0);
    EXPECT_EQ(chunk4.x, -1);  // round(-0.5) = -1
}

TEST(ChunkCoordinateSystem, IsPositionInChunk) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord chunk(0, 0, 0);  // Center at (0,0,0), bounds [-5, 5)
    
    // Inside chunk
    EXPECT_TRUE(cs.isPositionInChunk(Eigen::Vector3d(0, 0, 0), chunk));
    EXPECT_TRUE(cs.isPositionInChunk(Eigen::Vector3d(4.9, 4.9, 4.9), chunk));
    EXPECT_TRUE(cs.isPositionInChunk(Eigen::Vector3d(-4.9, -4.9, -4.9), chunk));
    
    // On lower boundary (inclusive)
    EXPECT_TRUE(cs.isPositionInChunk(Eigen::Vector3d(-5.0, 0, 0), chunk));
    
    // On upper boundary (exclusive)
    EXPECT_FALSE(cs.isPositionInChunk(Eigen::Vector3d(5.0, 0, 0), chunk));
    
    // Outside chunk
    EXPECT_FALSE(cs.isPositionInChunk(Eigen::Vector3d(5.1, 0, 0), chunk));
    EXPECT_FALSE(cs.isPositionInChunk(Eigen::Vector3d(-5.1, 0, 0), chunk));
}

TEST(ChunkCoordinateSystem, DistanceToBoundary) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord chunk(0, 0, 0);
    
    // At center: distance = 5.0 (half chunk size)
    double dist1 = cs.distanceToBoundary(Eigen::Vector3d(0, 0, 0), chunk);
    EXPECT_DOUBLE_EQ(dist1, 5.0);
    
    // Near boundary
    double dist2 = cs.distanceToBoundary(Eigen::Vector3d(4.5, 0, 0), chunk);
    EXPECT_DOUBLE_EQ(dist2, 0.5);
    
    // On boundary
    double dist3 = cs.distanceToBoundary(Eigen::Vector3d(-5.0, 0, 0), chunk);
    EXPECT_NEAR(dist3, 0.0, 1e-9);
    
    // Outside chunk (should return -1.0)
    double dist4 = cs.distanceToBoundary(Eigen::Vector3d(10.0, 0, 0), chunk);
    EXPECT_DOUBLE_EQ(dist4, -1.0);
}

TEST(ChunkCoordinateSystem, DistanceToBoundaryPerAxis) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord chunk(0, 0, 0);
    
    Eigen::Vector3d pos(2.0, -3.0, 4.0);
    
    // X-axis: min(2.0-(-5), 5-2.0) = min(7.0, 3.0) = 3.0
    double dist_x = cs.distanceToBoundary(pos, chunk, ChunkCoordinateSystem::AxisType::X);
    EXPECT_DOUBLE_EQ(dist_x, 3.0);
    
    // Y-axis: min(|-3-(-5)|, |5-(-3)|) = min(2.0, 8.0) = 2.0
    double dist_y = cs.distanceToBoundary(pos, chunk, ChunkCoordinateSystem::AxisType::Y);
    EXPECT_DOUBLE_EQ(dist_y, 2.0);
    
    // Z-axis: min(4-(-5), 5-4) = min(9.0, 1.0) = 1.0
    double dist_z = cs.distanceToBoundary(pos, chunk, ChunkCoordinateSystem::AxisType::Z);
    EXPECT_DOUBLE_EQ(dist_z, 1.0);
    
    // INDETERMINATE should return -1.0
    double dist_invalid = cs.distanceToBoundary(pos, chunk, 
                                                ChunkCoordinateSystem::AxisType::INDETERMINATE);
    EXPECT_DOUBLE_EQ(dist_invalid, -1.0);
}

TEST(ChunkCoordinateSystem, FaceNeighbours) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord center(0, 0, 0);
    
    auto neighbors = cs.getFaceNeighbours(center);
    EXPECT_EQ(neighbors.size(), 6);
    
    // Check all 6 face neighbors are present
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), ChunkCoord(1, 0, 0)), neighbors.end());
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), ChunkCoord(-1, 0, 0)), neighbors.end());
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), ChunkCoord(0, 1, 0)), neighbors.end());
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), ChunkCoord(0, -1, 0)), neighbors.end());
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), ChunkCoord(0, 0, 1)), neighbors.end());
    EXPECT_NE(std::find(neighbors.begin(), neighbors.end(), ChunkCoord(0, 0, -1)), neighbors.end());
}

TEST(ChunkCoordinateSystem, AllNeighbours) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord center(5, 10, -3);
    
    auto neighbors = cs.getAllNeighbours(center);
    
    // Should have 26 neighbors (3³ - 1)
    EXPECT_EQ(neighbors.size(), 26);
    
    // Center should NOT be in neighbors
    EXPECT_EQ(std::find(neighbors.begin(), neighbors.end(), center), neighbors.end());
}

TEST(ChunkCoordinateSystem, ForEachNeighbourRadius1) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord center(0, 0, 0);
    
    int count = 0;
    cs.forEachNeighbour(center, 1, [&count, &center](const ChunkCoord& nb) {
        count++;
        EXPECT_NE(nb, center);  // Should not iterate over center
    });
    
    EXPECT_EQ(count, 26);  // 3³ - 1
}

TEST(ChunkCoordinateSystem, ForEachNeighbourRadius2) {
    ChunkCoordinateSystem cs(0.2,10.0);
    ChunkCoord center(0, 0, 0);
    
    int count = 0;
    cs.forEachNeighbour(center, 2, [&count]([[maybe_unused]] const ChunkCoord& nb) {
        count++;
    });
    
    EXPECT_EQ(count, 124);  // 5³ - 1
}

TEST(ChunkCoordinateSystem, GetNeighbourType) {
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(0, 0, 0)), 
              ChunkCoordinateSystem::NeighbourType::SOURCE);
    
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(1, 0, 0)), 
              ChunkCoordinateSystem::NeighbourType::FACE);
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(0, -1, 0)), 
              ChunkCoordinateSystem::NeighbourType::FACE);
    
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(1, 1, 0)), 
              ChunkCoordinateSystem::NeighbourType::EDGE);
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(-1, 0, -1)), 
              ChunkCoordinateSystem::NeighbourType::EDGE);
    
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(1, 1, 1)), 
              ChunkCoordinateSystem::NeighbourType::CORNER);
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(-1, -1, -1)), 
              ChunkCoordinateSystem::NeighbourType::CORNER);
    
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(2, 0, 0)), 
              ChunkCoordinateSystem::NeighbourType::INDETERMINATE);
    EXPECT_EQ(ChunkCoordinateSystem::getNeighbourType(ChunkCoord(1, 1, 2)), 
              ChunkCoordinateSystem::NeighbourType::INDETERMINATE);
}

TEST(ChunkCoordinateSystem, RoundToInt) {
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(0.0), 0);
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(0.4), 0);
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(0.5), 1);  // Round half up
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(0.6), 1);
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(-0.4), 0);
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(-0.5), -1);  // Round half away from zero
    EXPECT_EQ(ChunkCoordinateSystem::roundToInt(-0.6), -1);
}

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}