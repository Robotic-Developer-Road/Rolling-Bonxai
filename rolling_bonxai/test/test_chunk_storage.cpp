/**
 * @file test_chunk_storage.cpp
 * @brief Unit tests for VectorChunkStorage, HashMapChunkStorage, and DequeChunkStorage
 * 
 * Tests basic functionality of all three storage implementations:
 * - Insert, find, contains, erase
 * - Size, capacity, empty
 * - Iteration
 * - Move semantics
 */

#include <gtest/gtest.h>
#include "rolling_bonxai/chunk_storage.hpp"
#include "rolling_bonxai/common.hpp"
#include "bonxai_map/occupancy_map.hpp"

using namespace RollingBonxai;

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Create a simple test chunk with known resolution
 */
std::unique_ptr<Bonxai::OccupancyMap> createTestMap(double resolution = 0.1) {
    return std::make_unique<Bonxai::OccupancyMap>(resolution);
}

/**
 * @brief Create a ManagedChunk for testing
 */
ManagedChunk createTestChunk(const ChunkCoord& coord, bool dirty = false) {
    return ManagedChunk(createTestMap(), coord, dirty);
}

// =============================================================================
// VectorChunkStorage Tests
// =============================================================================

TEST(VectorChunkStorage, DefaultConstruction) {
    VectorChunkStorage storage;
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_TRUE(storage.empty());
    EXPECT_GE(storage.capacity(), 27);  // Should reserve space for at least 27
}

TEST(VectorChunkStorage, InsertAndFind) {
    VectorChunkStorage storage;
    ChunkCoord coord{1, 2, 3};
    
    // Insert chunk
    storage.insert(coord, createTestChunk(coord));
    
    EXPECT_EQ(storage.size(), 1);
    EXPECT_FALSE(storage.empty());
    
    // Find the chunk
    ManagedChunk* found = storage.find(coord);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->getChunkCoord(), coord);
}

TEST(VectorChunkStorage, FindNonExistent) {
    VectorChunkStorage storage;
    ChunkCoord coord{5, 5, 5};
    
    ManagedChunk* found = storage.find(coord);
    EXPECT_EQ(found, nullptr);
}

TEST(VectorChunkStorage, Contains) {
    VectorChunkStorage storage;
    ChunkCoord coord{0, 0, 0};
    
    EXPECT_FALSE(storage.contains(coord));
    
    storage.insert(coord, createTestChunk(coord));
    
    EXPECT_TRUE(storage.contains(coord));
}

TEST(VectorChunkStorage, Erase) {
    VectorChunkStorage storage;
    ChunkCoord coord{1, 1, 1};
    
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
    
    storage.erase(coord);
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_FALSE(storage.contains(coord));
}

TEST(VectorChunkStorage, EraseNonExistent) {
    VectorChunkStorage storage;
    ChunkCoord coord{99, 99, 99};
    
    // Should be no-op
    storage.erase(coord);
    EXPECT_EQ(storage.size(), 0);
}

TEST(VectorChunkStorage, InsertMultiple) {
    VectorChunkStorage storage;
    
    std::vector<ChunkCoord> coords = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
        {1, 1, 1}
    };
    
    for (const auto& coord : coords) {
        storage.insert(coord, createTestChunk(coord));
    }
    
    EXPECT_EQ(storage.size(), coords.size());
    
    // Verify all chunks findable
    for (const auto& coord : coords) {
        EXPECT_TRUE(storage.contains(coord));
        EXPECT_NE(storage.find(coord), nullptr);
    }
}

TEST(VectorChunkStorage, InsertDuplicate) {
    VectorChunkStorage storage;
    ChunkCoord coord{5, 5, 5};
    
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
    
    // Insert duplicate (should be no-op)
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
}

TEST(VectorChunkStorage, Clear) {
    VectorChunkStorage storage;
    
    // Insert several chunks
    for (int i = 0; i < 10; ++i) {
        storage.insert(ChunkCoord{i, 0, 0}, createTestChunk(ChunkCoord{i, 0, 0}));
    }
    
    EXPECT_EQ(storage.size(), 10);
    
    storage.clear();
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_TRUE(storage.empty());
}

TEST(VectorChunkStorage, Iteration) {
    VectorChunkStorage storage;
    
    std::vector<ChunkCoord> coords = {
        {0, 0, 0},
        {1, 1, 1},
        {2, 2, 2}
    };
    
    for (const auto& coord : coords) {
        storage.insert(coord, createTestChunk(coord));
    }
    
    // Mutable iteration
    size_t count = 0;
    storage.forEachChunk([&]([[maybe_unused]] const ChunkCoord& coord, ManagedChunk& chunk) {
        EXPECT_NE(chunk.getMutableMap(), nullptr);
        count++;
    });
    EXPECT_EQ(count, coords.size());
    
    // Const iteration
    count = 0;
    const VectorChunkStorage& const_storage = storage;
    const_storage.forEachChunk([&]([[maybe_unused]] const ChunkCoord& coord, const ManagedChunk& chunk) {
        EXPECT_NE(chunk.getConstMap(), nullptr);
        count++;
    });
    EXPECT_EQ(count, coords.size());
}

TEST(VectorChunkStorage, MoveSemantics) {
    VectorChunkStorage storage1;
    ChunkCoord coord{1, 2, 3};
    storage1.insert(coord, createTestChunk(coord));
    
    // Move construction
    VectorChunkStorage storage2(std::move(storage1));
    
    EXPECT_EQ(storage2.size(), 1);
    EXPECT_TRUE(storage2.contains(coord));
    
    // Move assignment
    VectorChunkStorage storage3;
    storage3 = std::move(storage2);
    
    EXPECT_EQ(storage3.size(), 1);
    EXPECT_TRUE(storage3.contains(coord));
}

// // =============================================================================
// // HashMapChunkStorage Tests
// // =============================================================================

TEST(HashMapChunkStorage, DefaultConstruction) {
    HashMapChunkStorage storage(27);
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_TRUE(storage.empty());
}

TEST(HashMapChunkStorage, InsertAndFind) {
    HashMapChunkStorage storage(27);
    ChunkCoord coord{10, 20, 30};
    
    storage.insert(coord, createTestChunk(coord));
    
    EXPECT_EQ(storage.size(), 1);
    
    ManagedChunk* found = storage.find(coord);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->getChunkCoord(), coord);
}

TEST(HashMapChunkStorage, FindNonExistent) {
    HashMapChunkStorage storage(27);
    ChunkCoord coord{100, 100, 100};
    
    ManagedChunk* found = storage.find(coord);
    EXPECT_EQ(found, nullptr);
}

TEST(HashMapChunkStorage, Contains) {
    HashMapChunkStorage storage(27);
    ChunkCoord coord{0, 0, 0};
    
    EXPECT_FALSE(storage.contains(coord));
    
    storage.insert(coord, createTestChunk(coord));
    
    EXPECT_TRUE(storage.contains(coord));
}

TEST(HashMapChunkStorage, Erase) {
    HashMapChunkStorage storage(27);
    ChunkCoord coord{5, 5, 5};
    
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
    
    storage.erase(coord);
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_FALSE(storage.contains(coord));
}

TEST(HashMapChunkStorage, InsertMultiple) {
    HashMapChunkStorage storage(100);
    
    // Insert many chunks
    for (int i = 0; i < 50; ++i) {
        ChunkCoord coord{i, i, i};
        storage.insert(coord, createTestChunk(coord));
    }
    
    EXPECT_EQ(storage.size(), 50);
    
    // Verify all findable
    for (int i = 0; i < 50; ++i) {
        ChunkCoord coord{i, i, i};
        EXPECT_TRUE(storage.contains(coord));
    }
}

TEST(HashMapChunkStorage, InsertDuplicate) {
    HashMapChunkStorage storage(27);
    ChunkCoord coord{7, 7, 7};
    
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
    
    // Duplicate insert (should be no-op)
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
}

TEST(HashMapChunkStorage, Clear) {
    HashMapChunkStorage storage(100);
    
    for (int i = 0; i < 20; ++i) {
        storage.insert(ChunkCoord{i, 0, 0}, createTestChunk(ChunkCoord{i, 0, 0}));
    }
    
    EXPECT_EQ(storage.size(), 20);
    
    storage.clear();
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_TRUE(storage.empty());
}

TEST(HashMapChunkStorage, Iteration) {
    HashMapChunkStorage storage(27);
    
    std::vector<ChunkCoord> coords = {
        {-1, -1, -1},
        {0, 0, 0},
        {1, 1, 1}
    };
    
    for (const auto& coord : coords) {
        storage.insert(coord, createTestChunk(coord));
    }
    
    // Mutable iteration
    size_t count = 0;
    storage.forEachChunk([&]([[maybe_unused]] const ChunkCoord& coord,[[maybe_unused]] ManagedChunk& chunk) {
        count++;
    });
    EXPECT_EQ(count, coords.size());
    
    // Const iteration
    count = 0;
    const HashMapChunkStorage& const_storage = storage;
    const_storage.forEachChunk([&]([[maybe_unused]] const ChunkCoord& coord, [[maybe_unused]] const ManagedChunk& chunk) {
        count++;
    });
    EXPECT_EQ(count, coords.size());
}

TEST(HashMapChunkStorage, MoveSemantics) {
    HashMapChunkStorage storage1(27);
    ChunkCoord coord{3, 3, 3};
    storage1.insert(coord, createTestChunk(coord));
    
    // Move construction
    HashMapChunkStorage storage2(std::move(storage1));
    
    EXPECT_EQ(storage2.size(), 1);
    EXPECT_TRUE(storage2.contains(coord));
    
    // Move assignment
    HashMapChunkStorage storage3(27);
    storage3 = std::move(storage2);
    
    EXPECT_EQ(storage3.size(), 1);
    EXPECT_TRUE(storage3.contains(coord));
}

// =============================================================================
// DequeChunkStorage Tests
// =============================================================================

TEST(DequeChunkStorage, DefaultConstruction) {
    DequeChunkStorage storage;
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_TRUE(storage.empty());
}

TEST(DequeChunkStorage, InsertAndFind) {
    DequeChunkStorage storage;
    ChunkCoord coord{-5, 10, -15};
    
    storage.insert(coord, createTestChunk(coord));
    
    EXPECT_EQ(storage.size(), 1);
    
    ManagedChunk* found = storage.find(coord);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->getChunkCoord(), coord);
}

TEST(DequeChunkStorage, FindNonExistent) {
    DequeChunkStorage storage;
    ChunkCoord coord{50, 50, 50};
    
    ManagedChunk* found = storage.find(coord);
    EXPECT_EQ(found, nullptr);
}

TEST(DequeChunkStorage, Contains) {
    DequeChunkStorage storage;
    ChunkCoord coord{0, 0, 0};
    
    EXPECT_FALSE(storage.contains(coord));
    
    storage.insert(coord, createTestChunk(coord));
    
    EXPECT_TRUE(storage.contains(coord));
}

TEST(DequeChunkStorage, Erase) {
    DequeChunkStorage storage;
    ChunkCoord coord{2, 2, 2};
    
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
    
    storage.erase(coord);
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_FALSE(storage.contains(coord));
}

TEST(DequeChunkStorage, InsertMultiple) {
    DequeChunkStorage storage;
    
    for (int i = 0; i < 30; ++i) {
        ChunkCoord coord{i % 5, i / 5, 0};
        storage.insert(coord, createTestChunk(coord));
    }
    
    EXPECT_EQ(storage.size(), 30);
}

TEST(DequeChunkStorage, InsertDuplicate) {
    DequeChunkStorage storage;
    ChunkCoord coord{8, 8, 8};
    
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
    
    // Duplicate (should be no-op)
    storage.insert(coord, createTestChunk(coord));
    EXPECT_EQ(storage.size(), 1);
}

TEST(DequeChunkStorage, Clear) {
    DequeChunkStorage storage;
    
    for (int i = 0; i < 15; ++i) {
        storage.insert(ChunkCoord{i, i, i}, createTestChunk(ChunkCoord{i, i, i}));
    }
    
    EXPECT_EQ(storage.size(), 15);
    
    storage.clear();
    
    EXPECT_EQ(storage.size(), 0);
    EXPECT_TRUE(storage.empty());
}

TEST(DequeChunkStorage, Iteration) {
    DequeChunkStorage storage;
    
    std::vector<ChunkCoord> coords = {
        {10, 20, 30},
        {-10, -20, -30},
        {0, 0, 0}
    };
    
    for (const auto& coord : coords) {
        storage.insert(coord, createTestChunk(coord));
    }
    
    // Mutable iteration
    size_t count = 0;
    storage.forEachChunk([&]([[maybe_unused]] const ChunkCoord& coord, [[maybe_unused]] ManagedChunk& chunk) {
        count++;
    });
    EXPECT_EQ(count, coords.size());
    
    // Const iteration
    count = 0;
    const DequeChunkStorage& const_storage = storage;
    const_storage.forEachChunk([&]([[maybe_unused]] const ChunkCoord& coord, [[maybe_unused]] const ManagedChunk& chunk) {
        count++;
    });
    EXPECT_EQ(count, coords.size());
}

TEST(DequeChunkStorage, MoveSemantics) {
    DequeChunkStorage storage1;
    ChunkCoord coord{7, 8, 9};
    storage1.insert(coord, createTestChunk(coord));
    
    // Move construction
    DequeChunkStorage storage2(std::move(storage1));
    
    EXPECT_EQ(storage2.size(), 1);
    EXPECT_TRUE(storage2.contains(coord));
    
    // Move assignment
    DequeChunkStorage storage3;
    storage3 = std::move(storage2);
    
    EXPECT_EQ(storage3.size(), 1);
    EXPECT_TRUE(storage3.contains(coord));
}

// =============================================================================
// Polymorphic Tests (via ChunkStorage interface)
// =============================================================================

template <typename StorageT>
class ChunkStorageTypedTest : public ::testing::Test {
protected:
    std::unique_ptr<ChunkStorage> createStorage() {
        if constexpr (std::is_same_v<StorageT, VectorChunkStorage>) {
            return std::make_unique<VectorChunkStorage>(27);
        } else if constexpr (std::is_same_v<StorageT, HashMapChunkStorage>) {
            return std::make_unique<HashMapChunkStorage>(27);
        } else {
            return std::make_unique<DequeChunkStorage>();
        }
    }
};

using StorageTypes = ::testing::Types<VectorChunkStorage, HashMapChunkStorage, DequeChunkStorage>;
TYPED_TEST_SUITE(ChunkStorageTypedTest, StorageTypes);

TYPED_TEST(ChunkStorageTypedTest, BasicOperations) {
    auto storage = this->createStorage();
    
    ChunkCoord coord{1, 2, 3};
    
    // Initially empty
    EXPECT_TRUE(storage->empty());
    EXPECT_FALSE(storage->contains(coord));
    
    // Insert
    storage->insert(coord, createTestChunk(coord));
    EXPECT_FALSE(storage->empty());
    EXPECT_TRUE(storage->contains(coord));
    EXPECT_EQ(storage->size(), 1);
    
    // Find
    ManagedChunk* found = storage->find(coord);
    ASSERT_NE(found, nullptr);
    
    // Erase
    storage->erase(coord);
    EXPECT_TRUE(storage->empty());
    EXPECT_FALSE(storage->contains(coord));
}

TYPED_TEST(ChunkStorageTypedTest, NeighborhoodInsertion) {
    auto storage = this->createStorage();
    
    // Insert 27 chunks (3x3x3 neighborhood)
    std::vector<ChunkCoord> neighborhood;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                ChunkCoord coord{dx, dy, dz};
                neighborhood.push_back(coord);
                storage->insert(coord, createTestChunk(coord));
            }
        }
    }
    
    EXPECT_EQ(storage->size(), 27);
    
    // Verify all chunks are findable
    for (const auto& coord : neighborhood) {
        EXPECT_TRUE(storage->contains(coord));
        EXPECT_NE(storage->find(coord), nullptr);
    }
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}