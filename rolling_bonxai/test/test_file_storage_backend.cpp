#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include "rolling_bonxai/file_storage_backend.hpp"
#include "bonxai_map/occupancy_map.hpp"

namespace fs = std::filesystem;

class FileStorageBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create unique temporary directory for each test
        test_dir_ = fs::temp_directory_path() / "rolling_bonxai_test" / 
                    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        
        // Ensure clean state
        if (fs::exists(test_dir_)) {
            fs::remove_all(test_dir_);
        }
    }
    
    void TearDown() override {
        // Clean up test directory
        if (fs::exists(test_dir_)) {
            std::error_code ec;
            fs::remove_all(test_dir_, ec);
        }
    }
    
    // Helper: Create a simple test grid
    std::unique_ptr<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>> createTestGrid(double resolution = 0.1) {
        auto grid = std::make_unique<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>>(resolution);
        
        // Add some test data
        auto accessor = grid->createAccessor();
        accessor.setValue(Bonxai::CoordT{0, 0, 0}, Bonxai::Occupancy::CellOcc{});
        accessor.setValue(Bonxai::CoordT{1, 2, 3}, Bonxai::Occupancy::CellOcc{});
        accessor.setValue(Bonxai::CoordT{-5, 10, -3}, Bonxai::Occupancy::CellOcc{});
        
        return grid;
    }
    
    // Helper: Verify grid contents match
    void verifyGridsMatch(const Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>& grid1,
                          const Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>& grid2) {
        EXPECT_EQ(grid1.activeCellsCount(), grid2.activeCellsCount());
        EXPECT_DOUBLE_EQ(grid1.voxelSize(), grid2.voxelSize());
    }
    
    fs::path test_dir_;
};

// ============================================================================
// Constructor and Initialization Tests
// ============================================================================

TEST_F(FileStorageBackendTest, ConstructorDoesNotCreateDirectory) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    
    // Directory should NOT exist yet
    EXPECT_FALSE(fs::exists(test_dir_));
    EXPECT_FALSE(backend.isBackendInit());
}

TEST_F(FileStorageBackendTest, InitStorageBackendCreatesDirectory) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    
    ASSERT_TRUE(backend.initStorageBackend());
    
    EXPECT_TRUE(fs::exists(test_dir_));
    EXPECT_TRUE(fs::is_directory(test_dir_));
    EXPECT_TRUE(backend.isBackendInit());
}

TEST_F(FileStorageBackendTest, InitStorageBackendIdempotent) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    
    ASSERT_TRUE(backend.initStorageBackend());
    EXPECT_TRUE(backend.isBackendInit());
    
    // Second init should still succeed
    ASSERT_TRUE(backend.initStorageBackend());
    EXPECT_TRUE(backend.isBackendInit());
}

TEST_F(FileStorageBackendTest, GetBaseDirReturnsCorrectPath) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    
    EXPECT_EQ(backend.getBaseDir(), test_dir_);
    EXPECT_EQ(backend.getBaseDirStr(), test_dir_.string());
}

// // ============================================================================
// // Save and Load Tests
// // ============================================================================

TEST_F(FileStorageBackendTest, SaveCreatesChunkDirectory) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{5, 10, 2};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    
    // Check directory structure
    auto chunk_dir = test_dir_ / "5_10_2";
    EXPECT_TRUE(fs::exists(chunk_dir));
    EXPECT_TRUE(fs::is_directory(chunk_dir));
}

TEST_F(FileStorageBackendTest, SaveCreatesChunkFile) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{5, 10, 2};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    
    auto chunk_file = test_dir_ / "5_10_2" / "5_10_2.chunk";
    EXPECT_TRUE(fs::exists(chunk_file));
    EXPECT_TRUE(fs::is_regular_file(chunk_file));
    EXPECT_GT(fs::file_size(chunk_file), 0);
}

TEST_F(FileStorageBackendTest, SaveCreatesTimestampFile) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{5, 10, 2};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    
    auto timestamp_file = test_dir_ / "5_10_2" / "5_10_2.timestamp";
    EXPECT_TRUE(fs::exists(timestamp_file));
    EXPECT_TRUE(fs::is_regular_file(timestamp_file));
}

TEST_F(FileStorageBackendTest, SaveAndLoadRoundTrip) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{7, -3, 15};
    auto original_grid = createTestGrid();
    
    // Save
    ASSERT_TRUE(backend.save(coord, *original_grid));
    
    // Load
    auto loaded_grid = backend.load(coord);
    ASSERT_NE(loaded_grid, nullptr);
    
    // Verify
    verifyGridsMatch(*original_grid, *loaded_grid);
}

TEST_F(FileStorageBackendTest, LoadNonExistentChunkReturnsNullptr) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{99, 99, 99};
    
    auto loaded_grid = backend.load(coord);
    EXPECT_EQ(loaded_grid, nullptr);
}

TEST_F(FileStorageBackendTest, SaveNegativeCoordinates) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{-3, 7, -2};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    
    // Check directory name
    auto chunk_dir = test_dir_ / "-3_7_-2";
    EXPECT_TRUE(fs::exists(chunk_dir));
    
    // Verify load works
    auto loaded_grid = backend.load(coord);
    ASSERT_NE(loaded_grid, nullptr);
}

TEST_F(FileStorageBackendTest, SaveMultipleChunks) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    std::vector<RollingBonxai::ChunkCoord> coords = {
        {0, 0, 0},
        {1, 2, 3},
        {-5, -10, 15},
        {100, 200, 300}
    };
    
    for (const auto& coord : coords) {
        auto grid = createTestGrid();
        ASSERT_TRUE(backend.save(coord, *grid));
    }
    
    // Verify all exist
    for (const auto& coord : coords) {
        EXPECT_TRUE(backend.exists(coord));
    }
    
    // Verify all can be loaded
    for (const auto& coord : coords) {
        auto loaded = backend.load(coord);
        EXPECT_NE(loaded, nullptr);
    }
}

// // ============================================================================
// // Exists Tests
// // ============================================================================

TEST_F(FileStorageBackendTest, ExistsReturnsFalseForNonExistent) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{42, 42, 42};
    EXPECT_FALSE(backend.exists(coord));
}

TEST_F(FileStorageBackendTest, ExistsReturnsTrueAfterSave) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{10, 20, 30};
    auto grid = createTestGrid();
    
    EXPECT_FALSE(backend.exists(coord));
    
    ASSERT_TRUE(backend.save(coord, *grid));
    
    EXPECT_TRUE(backend.exists(coord));
}

// // ============================================================================
// // Timestamp Tests
// // ============================================================================

TEST_F(FileStorageBackendTest, TimestampCreatedOnFirstSave) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{1, 2, 3};
    auto grid = createTestGrid();
    
    auto before_save = std::chrono::system_clock::now();
    ASSERT_TRUE(backend.save(coord, *grid));
    auto after_save = std::chrono::system_clock::now();
    
    auto timestamp_opt = backend.loadTimestamp(coord);
    ASSERT_TRUE(timestamp_opt.has_value());
    
    auto timestamp = timestamp_opt.value();
    
    // Creation, modified, and accessed should all be set
    EXPECT_GT(timestamp.creation_time_ns, 0);
    EXPECT_GT(timestamp.last_modified_ns, 0);
    EXPECT_GT(timestamp.last_accessed_ns, 0);
    EXPECT_EQ(timestamp.access_count, 0);
    
    // Should be roughly around current time
    auto creation_time = timestamp.getCreationTime();
    EXPECT_GE(creation_time, before_save - std::chrono::seconds(1));
    EXPECT_LE(creation_time, after_save + std::chrono::seconds(1));
}

TEST_F(FileStorageBackendTest, TimestampModifiedOnSecondSave) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{5, 5, 5};
    auto grid = createTestGrid();
    
    // First save
    ASSERT_TRUE(backend.save(coord, *grid));
    auto first_timestamp = backend.loadTimestamp(coord);
    ASSERT_TRUE(first_timestamp.has_value());
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Second save
    ASSERT_TRUE(backend.save(coord, *grid));
    auto second_timestamp = backend.loadTimestamp(coord);
    ASSERT_TRUE(second_timestamp.has_value());
    
    // Creation time should be unchanged
    EXPECT_EQ(first_timestamp->creation_time_ns, second_timestamp->creation_time_ns);
    
    // Modified time should be updated
    EXPECT_GT(second_timestamp->last_modified_ns, first_timestamp->last_modified_ns);
}

TEST_F(FileStorageBackendTest, TimestampAccessedOnLoad) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{3, 3, 3};
    auto grid = createTestGrid();
    
    // Save
    ASSERT_TRUE(backend.save(coord, *grid));
    auto after_save = backend.loadTimestamp(coord);
    ASSERT_TRUE(after_save.has_value());
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Load
    auto loaded = backend.load(coord);
    ASSERT_NE(loaded, nullptr);
    
    auto after_load = backend.loadTimestamp(coord);
    ASSERT_TRUE(after_load.has_value());
    
    // Access count should increment
    EXPECT_EQ(after_load->access_count, 1);
    
    // Accessed time should be updated
    EXPECT_GT(after_load->last_accessed_ns, after_save->last_accessed_ns);
}

TEST_F(FileStorageBackendTest, TimestampAccessCountIncrements) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{8, 8, 8};
    auto grid = createTestGrid();
    
    // Save
    ASSERT_TRUE(backend.save(coord, *grid));
    
    // Load multiple times
    for (int i = 1; i <= 5; ++i) {
        auto loaded = backend.load(coord);
        ASSERT_NE(loaded, nullptr);
        
        auto timestamp = backend.loadTimestamp(coord);
        ASSERT_TRUE(timestamp.has_value());
        EXPECT_EQ(timestamp->access_count, i);
    }
}

TEST_F(FileStorageBackendTest, LoadTimestampReturnsNulloptForNonExistent) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{99, 99, 99};
    
    auto timestamp = backend.loadTimestamp(coord);
    EXPECT_FALSE(timestamp.has_value());
}

TEST_F(FileStorageBackendTest, UpdateModifiedTime) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{4, 4, 4};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    auto before = backend.loadTimestamp(coord);
    ASSERT_TRUE(before.has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    ASSERT_TRUE(backend.updateModifiedTime(coord));
    auto after = backend.loadTimestamp(coord);
    ASSERT_TRUE(after.has_value());
    
    EXPECT_GT(after->last_modified_ns, before->last_modified_ns);
    EXPECT_EQ(after->creation_time_ns, before->creation_time_ns);
    EXPECT_EQ(after->access_count, before->access_count);
}

TEST_F(FileStorageBackendTest, UpdateAccessTime) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{6, 6, 6};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    auto before = backend.loadTimestamp(coord);
    ASSERT_TRUE(before.has_value());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    ASSERT_TRUE(backend.updateAccessTime(coord));
    auto after = backend.loadTimestamp(coord);
    ASSERT_TRUE(after.has_value());
    
    EXPECT_GT(after->last_accessed_ns, before->last_accessed_ns);
    EXPECT_EQ(after->access_count, before->access_count + 1);
}

// // ============================================================================
// // YAML Format Tests
// // ============================================================================

TEST_F(FileStorageBackendTest, TimestampFileIsValidYAML) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{2, 2, 2};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    
    // Read timestamp file manually
    auto timestamp_path = test_dir_ / "2_2_2" / "2_2_2.timestamp";
    ASSERT_TRUE(fs::exists(timestamp_path));
    
    // Parse with YAML-CPP
    YAML::Node node = YAML::LoadFile(timestamp_path.string());
    
    EXPECT_TRUE(node["creation"]);
    EXPECT_TRUE(node["modified"]);
    EXPECT_TRUE(node["accessed"]);
    EXPECT_TRUE(node["count"]);
    
    EXPECT_GT(node["creation"].as<int64_t>(), 0);
    EXPECT_GT(node["modified"].as<int64_t>(), 0);
    EXPECT_GT(node["accessed"].as<int64_t>(), 0);
    EXPECT_GE(node["count"].as<uint64_t>(), 0);
}

// // ============================================================================
// // ChunkTimestamp Helper Method Tests
// // ============================================================================

TEST_F(FileStorageBackendTest, ChunkTimestampGetAge) {
    RollingBonxai::ChunkTimestamp timestamp;
    timestamp.creation_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        (std::chrono::system_clock::now() - std::chrono::seconds(100)).time_since_epoch()
    ).count();
    
    auto age = timestamp.getAge();
    
    // Should be approximately 100 seconds (allow some tolerance)
    EXPECT_GE(age.count(), 99);
    EXPECT_LE(age.count(), 101);
}

TEST_F(FileStorageBackendTest, ChunkTimestampGetTimeSinceModified) {
    RollingBonxai::ChunkTimestamp timestamp;
    timestamp.last_modified_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        (std::chrono::system_clock::now() - std::chrono::seconds(50)).time_since_epoch()
    ).count();
    
    auto time_since = timestamp.getTimeSinceModified();
    
    // Should be approximately 50 seconds
    EXPECT_GE(time_since.count(), 49);
    EXPECT_LE(time_since.count(), 51);
}

TEST_F(FileStorageBackendTest, ChunkTimestampGetTimeSinceAccessed) {
    RollingBonxai::ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        (std::chrono::system_clock::now() - std::chrono::seconds(30)).time_since_epoch()
    ).count();
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    // Should be approximately 30 seconds
    EXPECT_GE(time_since.count(), 29);
    EXPECT_LE(time_since.count(), 31);
}

// // ============================================================================
// // Edge Cases and Error Handling
// // ============================================================================

TEST_F(FileStorageBackendTest, SaveWithoutInitFails) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    // Don't call initStorageBackend()
    
    RollingBonxai::ChunkCoord coord{1, 1, 1};
    auto grid = createTestGrid();
    
    // Save should still work (creates directories as needed)
    // But this tests the behavior when base_dir doesn't exist
    EXPECT_TRUE(backend.save(coord, *grid));
}

TEST_F(FileStorageBackendTest, LoadCorruptedChunkReturnsNullptr) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{7, 7, 7};
    auto grid = createTestGrid();
    
    // Save valid chunk
    ASSERT_TRUE(backend.save(coord, *grid));
    
    // Corrupt the chunk file
    auto chunk_path = test_dir_ / "7_7_7" / "7_7_7.chunk";
    std::ofstream corrupt(chunk_path, std::ios::trunc);
    corrupt << "CORRUPTED DATA";
    corrupt.close();
    
    // Load should return nullptr
    auto loaded = backend.load(coord);
    EXPECT_EQ(loaded, nullptr);
}

TEST_F(FileStorageBackendTest, SaveOverwritesExistingChunk) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{9, 9, 9};
    
    // Save first grid
    auto grid1 = createTestGrid(0.1);
    ASSERT_TRUE(backend.save(coord, *grid1));
    
    // Save second grid with different resolution
    auto grid2 = createTestGrid(0.2);
    ASSERT_TRUE(backend.save(coord, *grid2));
    
    // Load should get the second grid
    auto loaded = backend.load(coord);
    ASSERT_NE(loaded, nullptr);
    EXPECT_DOUBLE_EQ(loaded->voxelSize(), 0.2);
}

TEST_F(FileStorageBackendTest, LargeCoordinateValues) {
    RollingBonxai::FileStorageBackend backend(test_dir_);
    ASSERT_TRUE(backend.initStorageBackend());
    
    RollingBonxai::ChunkCoord coord{1000000, -1000000, 999999};
    auto grid = createTestGrid();
    
    ASSERT_TRUE(backend.save(coord, *grid));
    EXPECT_TRUE(backend.exists(coord));
    
    auto loaded = backend.load(coord);
    EXPECT_NE(loaded, nullptr);
}

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}