#include <gtest/gtest.h>
#include "rolling_bonxai/chunk_loading_policies.hpp"
#include <chrono>
#include <thread>

using namespace RollingBonxai;

// ============================================================================
// Test Fixture: PolicyTestBase
// ============================================================================

class PolicyTestBase : public ::testing::Test {
protected:
    void SetUp() override {
        // Default setup
        current_chunk_ = ChunkCoord{0, 0, 0};
        robot_position_ = Eigen::Vector3d{0.0, 0.0, 0.0};
        current_time_ = std::chrono::steady_clock::now();
    }
    
    // Helper: Create PolicyContext
    PolicyContext createContext() {
        return PolicyContext{
            robot_position_,
            current_chunk_,
            current_time_,
            metadata_
        };
    }
    
    // Helper: Add chunk metadata
    void addMetadata(const ChunkCoord& coord, int64_t creation_time_ns) {
        ChunkMetadata meta;
        meta.creation_time = creation_time_ns;
        meta.last_access_time = creation_time_ns;
        meta.access_count = 1;
        meta.is_dirty = false;
        metadata_[coord] = meta;
    }
    
    ChunkCoord current_chunk_;
    Eigen::Vector3d robot_position_;
    std::chrono::steady_clock::time_point current_time_;
    std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash> metadata_;
};

// ============================================================================
// NeighborhoodPolicy Tests
// ============================================================================

class NeighborhoodPolicyTest : public PolicyTestBase {};

TEST_F(NeighborhoodPolicyTest, Construction_ValidRadius) {
    EXPECT_NO_THROW(NeighborhoodPolicy policy(0));
    EXPECT_NO_THROW(NeighborhoodPolicy policy(1));
    EXPECT_NO_THROW(NeighborhoodPolicy policy(5));
}

TEST_F(NeighborhoodPolicyTest, Construction_InvalidRadius) {
    EXPECT_THROW(NeighborhoodPolicy policy(-1), std::invalid_argument);
    EXPECT_THROW(NeighborhoodPolicy policy(-10), std::invalid_argument);
}

TEST_F(NeighborhoodPolicyTest, GetRadius) {
    NeighborhoodPolicy policy1(1);
    EXPECT_EQ(policy1.getRadius(), 1);
    
    NeighborhoodPolicy policy2(3);
    EXPECT_EQ(policy2.getRadius(), 3);
}

TEST_F(NeighborhoodPolicyTest, GetName) {
    NeighborhoodPolicy policy(1);
    std::string name = policy.getName();
    EXPECT_TRUE(name.find("NeighborhoodPolicy") != std::string::npos);
    EXPECT_TRUE(name.find("radius=1") != std::string::npos);
}

TEST_F(NeighborhoodPolicyTest, GetDescription) {
    NeighborhoodPolicy policy(1);
    std::string desc = policy.getDescription();
    EXPECT_FALSE(desc.empty());
    EXPECT_TRUE(desc.find("Chebyshev") != std::string::npos);
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_CurrentChunk) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Current chunk should always load
    EXPECT_TRUE(policy.shouldLoad(current_chunk_, context));
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_FaceNeighbors_Radius1) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // All 6 face neighbors should load
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 0, 0}, context));   // +X
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{-1, 0, 0}, context));  // -X
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 1, 0}, context));   // +Y
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, -1, 0}, context));  // -Y
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 1}, context));   // +Z
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, -1}, context));  // -Z
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_EdgeNeighbors_Radius1) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Edge neighbors (2 axes differ by 1)
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 1, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, -1, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 0, 1}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 1, 1}, context));
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_CornerNeighbors_Radius1) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Corner neighbors (all 3 axes differ by 1)
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 1, 1}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{-1, -1, -1}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, -1, 1}, context));
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_OutsideRadius) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Beyond radius should not load
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{2, 0, 0}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 2, 0}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 2}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{2, 2, 2}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{5, 5, 5}, context));
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_Radius2) {
    NeighborhoodPolicy policy(2);
    auto context = createContext();
    
    // Within radius 2
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{2, 0, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 2, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 1, 1}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{2, 2, 2}, context));
    
    // Beyond radius 2
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{3, 0, 0}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{2, 2, 3}, context));
}

TEST_F(NeighborhoodPolicyTest, ShouldLoad_NonOriginChunk) {
    NeighborhoodPolicy policy(1);
    current_chunk_ = ChunkCoord{5, 10, -3};
    auto context = createContext();
    
    // Current chunk
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{5, 10, -3}, context));
    
    // Neighbors
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{6, 10, -3}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{5, 11, -3}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{5, 10, -2}, context));
    
    // Outside
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{7, 10, -3}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 0}, context));
}

TEST_F(NeighborhoodPolicyTest, ShouldEvict_InverseOfShouldLoad) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Inside radius: should NOT evict
    EXPECT_FALSE(policy.shouldEvict(ChunkCoord{0, 0, 0}, context));
    EXPECT_FALSE(policy.shouldEvict(ChunkCoord{1, 0, 0}, context));
    EXPECT_FALSE(policy.shouldEvict(ChunkCoord{1, 1, 1}, context));
    
    // Outside radius: should evict
    EXPECT_TRUE(policy.shouldEvict(ChunkCoord{2, 0, 0}, context));
    EXPECT_TRUE(policy.shouldEvict(ChunkCoord{5, 5, 5}, context));
}

TEST_F(NeighborhoodPolicyTest, GetPriority_CurrentChunk) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Current chunk should have highest priority
    double priority = policy.getPriority(current_chunk_, context);
    EXPECT_DOUBLE_EQ(priority, 1.0);
}

TEST_F(NeighborhoodPolicyTest, GetPriority_DecreaseWithDistance) {
    NeighborhoodPolicy policy(3);
    auto context = createContext();
    
    double p0 = policy.getPriority(ChunkCoord{0, 0, 0}, context);  // dist=0
    double p1 = policy.getPriority(ChunkCoord{1, 0, 0}, context);  // dist=1
    double p2 = policy.getPriority(ChunkCoord{2, 0, 0}, context);  // dist=2
    double p3 = policy.getPriority(ChunkCoord{3, 0, 0}, context);  // dist=3
    
    // Priority should decrease with distance
    EXPECT_GT(p0, p1);
    EXPECT_GT(p1, p2);
    EXPECT_GT(p2, p3);
    
    // Verify formula: 1.0 / (1.0 + dist)
    EXPECT_DOUBLE_EQ(p0, 1.0);
    EXPECT_DOUBLE_EQ(p1, 0.5);
    EXPECT_DOUBLE_EQ(p2, 1.0/3.0);
    EXPECT_DOUBLE_EQ(p3, 0.25);
}

TEST_F(NeighborhoodPolicyTest, GetPriority_ChebyshevDistance) {
    NeighborhoodPolicy policy(3);
    auto context = createContext();
    
    // Chebyshev distance uses max of absolute differences
    double p_edge = policy.getPriority(ChunkCoord{2, 1, 0}, context);  // max(2,1,0) = 2
    double p_corner = policy.getPriority(ChunkCoord{2, 2, 2}, context); // max(2,2,2) = 2
    
    // Same Chebyshev distance should give same priority
    EXPECT_DOUBLE_EQ(p_edge, p_corner);
    EXPECT_DOUBLE_EQ(p_edge, 1.0/3.0);
}

TEST_F(NeighborhoodPolicyTest, GetPriority_OutsideRadius) {
    NeighborhoodPolicy policy(1);
    auto context = createContext();
    
    // Chunks outside radius still get priority (for queue ordering)
    double p = policy.getPriority(ChunkCoord{5, 5, 5}, context);
    EXPECT_GT(p, 0.0);
    EXPECT_LT(p, 1.0);
}

TEST_F(NeighborhoodPolicyTest, GetPriority_AllWithinRadius_InRange) {
    NeighborhoodPolicy policy(2);
    auto context = createContext();
    
    // All priorities should be in [0, 1]
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dz = -2; dz <= 2; ++dz) {
                ChunkCoord coord{dx, dy, dz};
                double p = policy.getPriority(coord, context);
                EXPECT_GE(p, 0.0);
                EXPECT_LE(p, 1.0);
            }
        }
    }
}

// // ============================================================================
// // PlanarNeighborhoodPolicy Tests
// // ============================================================================

class PlanarNeighborhoodPolicyTest : public PolicyTestBase {};

TEST_F(PlanarNeighborhoodPolicyTest, Construction_Valid_Absolute) {
    EXPECT_NO_THROW(PlanarNeighborhoodPolicy policy(1, 0, 2, false));
    EXPECT_NO_THROW(PlanarNeighborhoodPolicy policy(1, -5, 10, false));
    EXPECT_NO_THROW(PlanarNeighborhoodPolicy policy(2, 0, 0, false));  // Single layer
}

TEST_F(PlanarNeighborhoodPolicyTest, Construction_Valid_Relative) {
    EXPECT_NO_THROW(PlanarNeighborhoodPolicy policy(1, 0, 2, true));
    EXPECT_NO_THROW(PlanarNeighborhoodPolicy policy(1, 5, 10, true));
    EXPECT_NO_THROW(PlanarNeighborhoodPolicy policy(2, 0, 0, true));
}

TEST_F(PlanarNeighborhoodPolicyTest, Construction_InvalidRadius) {
    EXPECT_THROW(PlanarNeighborhoodPolicy policy(-1, 0, 2, false), std::invalid_argument);
}

TEST_F(PlanarNeighborhoodPolicyTest, Construction_InvalidZRange_Absolute) {
    // Absolute mode: z_min must be <= z_max
    EXPECT_THROW(PlanarNeighborhoodPolicy policy(1, 5, 2, false), std::invalid_argument);
}

TEST_F(PlanarNeighborhoodPolicyTest, Construction_InvalidZRange_Relative) {
    // Relative mode: both must be non-negative
    EXPECT_THROW(PlanarNeighborhoodPolicy policy(1, -1, 5, true), std::invalid_argument);
    EXPECT_THROW(PlanarNeighborhoodPolicy policy(1, 5, -2, true), std::invalid_argument);
    EXPECT_THROW(PlanarNeighborhoodPolicy policy(1, -3, -1, true), std::invalid_argument);
}

TEST_F(PlanarNeighborhoodPolicyTest, Getters) {
    PlanarNeighborhoodPolicy policy(2, 1, 5, false);
    EXPECT_EQ(policy.getRadius(), 2);
    EXPECT_EQ(policy.getZMin(), 1);
    EXPECT_EQ(policy.getZMax(), 5);
    EXPECT_FALSE(policy.isRelative());
    
    PlanarNeighborhoodPolicy policy_rel(3, 2, 8, true);
    EXPECT_EQ(policy_rel.getRadius(), 3);
    EXPECT_EQ(policy_rel.getZMin(), 2);
    EXPECT_EQ(policy_rel.getZMax(), 8);
    EXPECT_TRUE(policy_rel.isRelative());
}

TEST_F(PlanarNeighborhoodPolicyTest, GetName) {
    PlanarNeighborhoodPolicy policy(1, 0, 2, false);
    std::string name = policy.getName();
    EXPECT_TRUE(name.find("PlanarNeighborhoodPolicy") != std::string::npos);
    EXPECT_TRUE(name.find("radius=1") != std::string::npos);
    EXPECT_TRUE(name.find("z=[0,2]") != std::string::npos);
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_Absolute_WithinRange) {
    PlanarNeighborhoodPolicy policy(1, 0, 2, false);
    current_chunk_ = ChunkCoord{0, 0, 1};  // At z=1
    auto context = createContext();
    
    // Within horizontal radius and vertical range
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 0}, context));  // z=0
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 1}, context));  // z=1
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 2}, context));  // z=2
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 0, 1}, context));  // horizontal neighbor
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 1, 0}, context));  // diagonal + in range
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_Absolute_OutsideZRange) {
    PlanarNeighborhoodPolicy policy(1, 0, 2, false);
    current_chunk_ = ChunkCoord{0, 0, 1};
    auto context = createContext();
    
    // Outside vertical range
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -1}, context));  // z=-1
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 3}, context));   // z=3
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 10}, context));  // z=10
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{1, 1, -2}, context));  // diagonal but out of range
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_Absolute_NegativeZCoordinates) {
    PlanarNeighborhoodPolicy policy(7, -5, 0, false);
    current_chunk_ = ChunkCoord{0, 0, -2};
    auto context = createContext();
    
    // Within negative z range
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, -5}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, -3}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 1, -2}, context));
    
    // Outside range
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -6}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 1}, context));
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_Relative_Above) {
    PlanarNeighborhoodPolicy policy(50, 0, 2, true);  // 0 below, 2 above with a radius of 50
    current_chunk_ = ChunkCoord{0, 0, 10};  // At z=10
    auto context = createContext();
    
    // Within relative range
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 8}, context));   // 2 below
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 9}, context));   // 1 below
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 10}, context));  // current
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 11}, context));  // 1 above
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 12}, context));  // 2 above
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 13}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 14}, context)); // 6 above (> z_max)
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_Relative_NoAbove) {
    PlanarNeighborhoodPolicy policy(50, 4, 0, true);  // 4 below, 0 above
    current_chunk_ = ChunkCoord{0, 0, 5};
    auto context = createContext();
    
    // Should load below
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 4}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 3}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 2}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 1}, context));
    // But not below the threshold
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 0}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -1}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -2}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -3}, context));
    
    // Should load current and above
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 0, 5}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 6}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 7}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, 8}, context));
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_Relative_NegativeZChunk) {
    PlanarNeighborhoodPolicy policy(10, 3, 3, true);  // 3 below, 3 above
    current_chunk_ = ChunkCoord{0, 0, -5};  // Negative z coordinate
    auto context = createContext();
    
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, -8}, context));  // 3 below
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, -5}, context));  // current
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, -2}, context));  // 3 above
    
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -9}, context)); // 4 below
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 0, -1}, context)); // 4 above
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldLoad_OutsideHorizontalRadius) {
    PlanarNeighborhoodPolicy policy(1, 0, 2, false);
    current_chunk_ = ChunkCoord{0, 0, 1};
    auto context = createContext();
    
    // Within z range but outside horizontal radius
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{2, 0, 1}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{0, 2, 1}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{2, 2, 1}, context));
}

TEST_F(PlanarNeighborhoodPolicyTest, ShouldEvict_InverseOfShouldLoad) {
    PlanarNeighborhoodPolicy policy(1, 0, 2, false);
    current_chunk_ = ChunkCoord{0, 0, 1};
    auto context = createContext();
    
    // Inside range: should NOT evict
    EXPECT_FALSE(policy.shouldEvict(ChunkCoord{0, 0, 1}, context));
    EXPECT_FALSE(policy.shouldEvict(ChunkCoord{1, 1, 0}, context));
    
    // Outside range: should evict
    EXPECT_TRUE(policy.shouldEvict(ChunkCoord{0, 0, 5}, context));
    EXPECT_TRUE(policy.shouldEvict(ChunkCoord{2, 0, 1}, context));
}

TEST_F(PlanarNeighborhoodPolicyTest, GetPriority_SameAsNeighborhood_WithinRange) {
    PlanarNeighborhoodPolicy policy(2, -5, 5, false);
    current_chunk_ = ChunkCoord{0, 0, 0};
    auto context = createContext();
    
    // Priority should match NeighborhoodPolicy for chunks in range
    double p0 = policy.getPriority(ChunkCoord{0, 0, 0}, context);
    double p1 = policy.getPriority(ChunkCoord{1, 0, 0}, context);
    double p2 = policy.getPriority(ChunkCoord{2, 2, 2}, context);
    
    EXPECT_DOUBLE_EQ(p0, 1.0);
    EXPECT_DOUBLE_EQ(p1, 0.5);
    EXPECT_GT(p1, p2);
}

TEST_F(PlanarNeighborhoodPolicyTest, GetPriority_ZeroForOutOfRange) {
    PlanarNeighborhoodPolicy policy(1, 0, 2, false);
    current_chunk_ = ChunkCoord{0, 0, 1};
    auto context = createContext();
    
    // Outside z range should get 0 priority
    EXPECT_DOUBLE_EQ(policy.getPriority(ChunkCoord{0, 0, 5}, context), 0.0);
    EXPECT_DOUBLE_EQ(policy.getPriority(ChunkCoord{0, 0, -1}, context), 0.0);
}

// // ============================================================================
// // TemporalNeighborhoodPolicy Tests
// // ============================================================================

class TemporalNeighborhoodPolicyTest : public PolicyTestBase {};

TEST_F(TemporalNeighborhoodPolicyTest, Construction_Valid) {
    EXPECT_NO_THROW(TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60)));
    EXPECT_NO_THROW(TemporalNeighborhoodPolicy policy(2, 0.0, std::chrono::hours(1)));
    EXPECT_NO_THROW(TemporalNeighborhoodPolicy policy(1, 1.0, std::chrono::minutes(30)));
}

TEST_F(TemporalNeighborhoodPolicyTest, Construction_InvalidRadius) {
    EXPECT_THROW(
        TemporalNeighborhoodPolicy policy(-1, 0.3, std::chrono::seconds(60)),
        std::invalid_argument
    );
}

TEST_F(TemporalNeighborhoodPolicyTest, Construction_InvalidAgeWeightage) {
    EXPECT_THROW(
        TemporalNeighborhoodPolicy policy(1, -0.1, std::chrono::seconds(60)),
        std::invalid_argument
    );
    EXPECT_THROW(
        TemporalNeighborhoodPolicy policy(1, 1.1, std::chrono::seconds(60)),
        std::invalid_argument
    );
}

TEST_F(TemporalNeighborhoodPolicyTest, Getters) {
    TemporalNeighborhoodPolicy policy(2, 0.3, std::chrono::seconds(120));
    EXPECT_EQ(policy.getRadius(), 2);
    EXPECT_EQ(policy.getMaxAge(), std::chrono::seconds(120));
    EXPECT_DOUBLE_EQ(policy.getAgeWeightage(), 0.3);
}

TEST_F(TemporalNeighborhoodPolicyTest, GetName) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(3600));
    std::string name = policy.getName();
    EXPECT_TRUE(name.find("TemporalNeighborhoodPolicy") != std::string::npos);
    EXPECT_TRUE(name.find("radius=1") != std::string::npos);
    EXPECT_TRUE(name.find("max_age=3600s") != std::string::npos);
}

TEST_F(TemporalNeighborhoodPolicyTest, ShouldLoad_NewChunk_NoMetadata) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60));
    auto context = createContext();
    
    // Chunks with no metadata should always load (within radius)
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{0, 0, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 0, 0}, context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 1, 1}, context));
}

TEST_F(TemporalNeighborhoodPolicyTest, ShouldLoad_NewChunk_WithinAge) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60));
    
    // Add recent metadata (30 seconds ago)
    auto now = std::chrono::steady_clock::now();
    auto recent_time = now - std::chrono::seconds(30);
    int64_t recent_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        recent_time.time_since_epoch()
    ).count();
    
    addMetadata(ChunkCoord{1, 0, 0}, recent_ns);
    
    current_time_ = now;
    auto context = createContext();
    
    // Within age limit
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord{1, 0, 0}, context));
}

TEST_F(TemporalNeighborhoodPolicyTest, ShouldLoad_OldChunk_ExceedsAge) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60));
    
    // Add old metadata (120 seconds ago, exceeds 60s limit)
    auto now = std::chrono::steady_clock::now();
    auto old_time = now - std::chrono::seconds(120);
    int64_t old_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        old_time.time_since_epoch()
    ).count();
    
    addMetadata(ChunkCoord{1, 0, 0}, old_ns);
    
    current_time_ = now;
    auto context = createContext();
    
    // Exceeds age limit
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{1, 0, 0}, context));
}

TEST_F(TemporalNeighborhoodPolicyTest, ShouldLoad_ExactlyAtMaxAge) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60));
    
    // Add metadata exactly 60 seconds ago
    auto now = std::chrono::steady_clock::now();
    auto exact_time = now - std::chrono::seconds(60);
    int64_t exact_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        exact_time.time_since_epoch()
    ).count();
    
    addMetadata(ChunkCoord{1, 0, 0}, exact_ns);
    
    current_time_ = now;
    auto context = createContext();
    
    // Exactly at limit should NOT load (age < max_age, not <=)
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{1, 0, 0}, context));
}

TEST_F(TemporalNeighborhoodPolicyTest, ShouldLoad_OutsideRadius_EvenIfNew) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60));
    auto context = createContext();
    
    // Outside radius should not load, even without metadata
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{2, 0, 0}, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord{5, 5, 5}, context));
}

TEST_F(TemporalNeighborhoodPolicyTest, ShouldEvict_InverseOfShouldLoad) {
    TemporalNeighborhoodPolicy policy(1, 0.3, std::chrono::seconds(60));
    
    auto now = std::chrono::steady_clock::now();
    auto recent = now - std::chrono::seconds(30);
    auto old = now - std::chrono::seconds(120);
    
    addMetadata(ChunkCoord{1, 0, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        recent.time_since_epoch()).count());
    addMetadata(ChunkCoord{0, 0, 2}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        old.time_since_epoch()).count());
    
    current_time_ = now;
    auto context = createContext();
    
    // Recent chunk should NOT evict
    EXPECT_FALSE(policy.shouldEvict(ChunkCoord{1, 0, 0}, context));
    
    // Old chunk should evict
    EXPECT_TRUE(policy.shouldEvict(ChunkCoord{0, 0, 2}, context));
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_NewChunk_FullPriority) {
    TemporalNeighborhoodPolicy policy(2, 0.3, std::chrono::seconds(60));
    auto context = createContext();
    
    // New chunks (no metadata) get full distance-based priority
    double p_current = policy.getPriority(ChunkCoord{0, 0, 0}, context);
    double p_dist1 = policy.getPriority(ChunkCoord{1, 0, 0}, context);
    
    EXPECT_DOUBLE_EQ(p_current, 1.0);
    EXPECT_DOUBLE_EQ(p_dist1, 0.5);
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_NewerChunk_HigherPriority) {
    TemporalNeighborhoodPolicy policy(2, 0.3, std::chrono::seconds(100));
    
    auto now = std::chrono::steady_clock::now();
    
    // Add two chunks at same distance but different ages
    auto recent = now - std::chrono::seconds(10);  // 10s old
    auto older = now - std::chrono::seconds(80);   // 80s old
    
    addMetadata(ChunkCoord{1, 0, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        recent.time_since_epoch()).count());
    addMetadata(ChunkCoord{0, 1, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        older.time_since_epoch()).count());
    
    current_time_ = now;
    auto context = createContext();
    
    double p_recent = policy.getPriority(ChunkCoord{1, 0, 0}, context);
    double p_older = policy.getPriority(ChunkCoord{0, 1, 0}, context);
    
    // Recent chunk should have higher priority
    EXPECT_GT(p_recent, p_older);
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_AgeWeightage_ZeroPureDistance) {
    TemporalNeighborhoodPolicy policy(2, 0.0, std::chrono::seconds(100));
    
    auto now = std::chrono::steady_clock::now();
    auto old = now - std::chrono::seconds(90);
    
    addMetadata(ChunkCoord{1, 0, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        old.time_since_epoch()).count());
    
    current_time_ = now;
    auto context = createContext();
    
    // With age_weightage=0.0, priority should be pure distance-based
    double p = policy.getPriority(ChunkCoord{1, 0, 0}, context);
    EXPECT_DOUBLE_EQ(p, 0.5);  // Pure distance priority: 1/(1+1) = 0.5
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_AgeWeightage_OnePureAge) {
    TemporalNeighborhoodPolicy policy(2, 1.0, std::chrono::seconds(100));
    
    auto now = std::chrono::steady_clock::now();
    
    // Brand new chunk at distance 1
    auto brand_new = now - std::chrono::seconds(0);
    addMetadata(ChunkCoord{1, 0, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        brand_new.time_since_epoch()).count());
    
    // Half-life chunk at distance 1
    auto half_old = now - std::chrono::seconds(50);
    addMetadata(ChunkCoord{0, 1, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        half_old.time_since_epoch()).count());
    
    current_time_ = now;
    auto context = createContext();
    
    // With age_weightage=1.0, priority should be pure age-based
    double p_new = policy.getPriority(ChunkCoord{1, 0, 0}, context);
    double p_half = policy.getPriority(ChunkCoord{0, 1, 0}, context);
    
    // Brand new: freshness = 1.0
    // Half old: freshness = 0.5
    EXPECT_DOUBLE_EQ(p_new, 1.0);
    EXPECT_DOUBLE_EQ(p_half, 0.5);
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_AgeWeightage_Balanced) {
    TemporalNeighborhoodPolicy policy(2, 0.5, std::chrono::seconds(100));
    
    auto now = std::chrono::steady_clock::now();
    auto half_old = now - std::chrono::seconds(50);
    
    addMetadata(ChunkCoord{1, 0, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        half_old.time_since_epoch()).count());
    
    current_time_ = now;
    auto context = createContext();
    
    double p = policy.getPriority(ChunkCoord{1, 0, 0}, context);
    
    // dist_priority = 1/(1+1) = 0.5
    // freshness = 1 - 50/100 = 0.5
    // priority = 0.5 * 0.5 + 0.5 * 0.5 = 0.5
    EXPECT_DOUBLE_EQ(p, 0.5);
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_OldChunk_ZeroPriority) {
    TemporalNeighborhoodPolicy policy(2, 0.3, std::chrono::seconds(60));
    
    auto now = std::chrono::steady_clock::now();
    auto too_old = now - std::chrono::seconds(120);
    
    addMetadata(ChunkCoord{1, 0, 0}, std::chrono::duration_cast<std::chrono::nanoseconds>(
        too_old.time_since_epoch()).count());
    
    current_time_ = now;
    auto context = createContext();
    
    // Chunks exceeding max_age should get 0 priority
    EXPECT_DOUBLE_EQ(policy.getPriority(ChunkCoord{1, 0, 0}, context), 0.0);
}

TEST_F(TemporalNeighborhoodPolicyTest, GetPriority_AllInRange) {
    TemporalNeighborhoodPolicy policy(2, 0.3, std::chrono::seconds(100));
    
    auto now = std::chrono::steady_clock::now();
    auto context = createContext();
    
    // All valid priorities should be in [0, 1]
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dz = -2; dz <= 2; ++dz) {
                ChunkCoord coord{dx, dy, dz};
                double p = policy.getPriority(coord, context);
                EXPECT_GE(p, 0.0);
                EXPECT_LE(p, 1.0);
            }
        }
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}