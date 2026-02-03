#include <gtest/gtest.h>
#include "rolling_bonxai/common.hpp"
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <thread>

using namespace RollingBonxai;
using Hasher = RollingBonxai::ChunkCoordHash;
// ============================================================================
// ChunkCoord Tests
// ============================================================================

TEST(ChunkCoordTest, DefaultConstructor) {
    ChunkCoord c;
    EXPECT_EQ(c.x, 0);
    EXPECT_EQ(c.y, 0);
    EXPECT_EQ(c.z, 0);
}

TEST(ChunkCoordTest, ParameterizedConstructor) {
    ChunkCoord c(5, -3, 10);
    EXPECT_EQ(c.x, 5);
    EXPECT_EQ(c.y, -3);
    EXPECT_EQ(c.z, 10);
}

TEST(ChunkCoordTest, EqualityOperator) {
    ChunkCoord c1(1, 2, 3);
    ChunkCoord c2(1, 2, 3);
    ChunkCoord c3(1, 2, 4);
    
    EXPECT_TRUE(c1 == c2);
    EXPECT_FALSE(c1 == c3);
    EXPECT_FALSE(c1 != c2);
    EXPECT_TRUE(c1 != c3);
}

TEST(ChunkCoordTest, AdditionOperator) {
    ChunkCoord c1(1, 2, 3);
    ChunkCoord c2(4, -1, 2);
    ChunkCoord result = c1 + c2;
    
    EXPECT_EQ(result.x, 5);
    EXPECT_EQ(result.y, 1);
    EXPECT_EQ(result.z, 5);
}

// ============================================================================
// ChunkHash Tests
// ============================================================================

TEST(ChunkCoordHashTest, Deterministic)
{
    Hasher h;
    ChunkCoord c{1, 2, 3};

    auto v1 = h(c);
    auto v2 = h(c);

    EXPECT_EQ(v1, v2);
}

TEST(ChunkCoordHashTest, DifferentCoordsDifferentHash)
{
    Hasher h;

    ChunkCoord a{1, 2, 3};
    ChunkCoord b{1, 2, 4};
    ChunkCoord c{1, 3, 3};
    ChunkCoord d{2, 2, 3};

    EXPECT_NE(h(a), h(b));
    EXPECT_NE(h(a), h(c));
    EXPECT_NE(h(a), h(d));
}

TEST(ChunkCoordHashTest, AxisSensitivity)
{
    Hasher h;
    ChunkCoord base{10, 20, 30};

    auto h0 = h(base);

    EXPECT_NE(h0, h({11, 20, 30}));
    EXPECT_NE(h0, h({10, 21, 30}));
    EXPECT_NE(h0, h({10, 20, 31}));
}


TEST(ChunkCoordHashTest, HandlesNegativeCoordinates)
{
    Hasher h;

    ChunkCoord a{-1, 0, 0};
    ChunkCoord b{0, -1, 0};
    ChunkCoord c{0, 0, -1};

    EXPECT_NE(h(a), h(b));
    EXPECT_NE(h(a), h(c));
    EXPECT_NE(h(b), h(c));
}

TEST(ChunkCoordHashTest, NegativeAndPositivesNotSame)
{
    Hasher h;

    ChunkCoord a{10, 12, 77};
    ChunkCoord b{-10, 12, 77};
    ChunkCoord c{10, -12, 77};
    ChunkCoord d{10, 12, -77};
    ChunkCoord e{-10, -12, -77};

    EXPECT_NE(h(a), h(b));
    EXPECT_NE(h(a), h(c));
    EXPECT_NE(h(a), h(d));
    EXPECT_NE(h(a), h(e));
    EXPECT_NE(h(b), h(c));
    EXPECT_NE(h(b), h(d));
    EXPECT_NE(h(b), h(e));
    EXPECT_NE(h(c), h(d));
    EXPECT_NE(h(c), h(e));
    EXPECT_NE(h(d), h(e));
}

TEST(ChunkCoordHashTest, CoordinateOrderMatters)
{
    Hasher h;

    std::array<ChunkCoord,6> possibilities = {{{1, 2, 3},
                                              {1, 3, 2},
                                              {2, 1, 3},
                                              {2, 3, 1},
                                              {3, 1, 2},
                                              {3, 2, 1}}};

    for (size_t i = 0 ; i < possibilities.size() -1 ; ++i) {
        for (size_t j = i + 1 ; j < possibilities.size() ; ++j) {
            auto &a = possibilities[i];
            auto &b = possibilities[j];
            EXPECT_NE(h(a), h(b));
        }
    }
}

TEST(ChunkCoordHashTest, ZeroCoordinate)
{
    Hasher h;
    ChunkCoord zero{0, 0, 0};

    auto value = h(zero);

    EXPECT_NE(value, 0u);  // Not required by spec, but good sanity
}

TEST(ChunkCoordHashTest, WorksInUnorderedMap)
{
    std::unordered_map<ChunkCoord, int, Hasher> map;
    ChunkCoord a(1,2,3);
    ChunkCoord b(4,5,6);
    map[a] = 42;
    map[b] = 99;

    EXPECT_EQ(map[a], 42);
    EXPECT_EQ(map[b], 99);
}

TEST(ChunkCoordHashTest, LowCollisionSmallGrid)
{
    Hasher h;
    std::unordered_set<std::size_t> hashes;

    constexpr int N = 20;
    for (int x = -N; x <= N; ++x)
        for (int y = -N; y <= N; ++y)
            for (int z = -N; z <= N; ++z)
                hashes.insert(h({x, y, z}));

    const std::size_t expected = (2*N + 1) * (2*N + 1) * (2*N + 1);
    EXPECT_GT(hashes.size(), expected * 0.99); // tolerate tiny collisions
}

// ============================================================================
// ChunkTimestamp Tests
// ============================================================================

class ChunkTimestampTest : public ::testing::Test {
protected:
    // Helper: Get current time in nanoseconds since epoch
    int64_t getCurrentTimeNs() const {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
    
    // Helper: Get time N seconds ago in nanoseconds
    int64_t getTimeNSecondsAgo(int seconds) const {
        auto past = std::chrono::system_clock::now() - std::chrono::seconds(seconds);
        auto duration = past.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(ChunkTimestampTest, DefaultConstructorInitializesToZero) {
    ChunkTimestamp timestamp;
    
    EXPECT_EQ(timestamp.creation_time_ns, 0);
    EXPECT_EQ(timestamp.last_modified_ns, 0);
    EXPECT_EQ(timestamp.last_accessed_ns, 0);
    EXPECT_EQ(timestamp.access_count, 0);
}

// ============================================================================
// Time Point Conversion Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetCreationTimeReturnsCorrectTimePoint) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getCurrentTimeNs();
    
    auto time_point = timestamp.getCreationTime();
    auto now = std::chrono::system_clock::now();
    
    // Should be very close to current time (within 1 second)
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - time_point);
    EXPECT_LE(std::abs(diff.count()), 1);
}

TEST_F(ChunkTimestampTest, GetLastModifiedReturnsCorrectTimePoint) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getTimeNSecondsAgo(60);  // 60 seconds ago
    
    auto time_point = timestamp.getLastModified();
    auto now = std::chrono::system_clock::now();
    
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - time_point);
    EXPECT_GE(diff.count(), 59);  // At least 59 seconds ago
    EXPECT_LE(diff.count(), 61);  // At most 61 seconds ago
}

TEST_F(ChunkTimestampTest, GetLastAccessedReturnsCorrectTimePoint) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getTimeNSecondsAgo(30);  // 30 seconds ago
    
    auto time_point = timestamp.getLastAccessed();
    auto now = std::chrono::system_clock::now();
    
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - time_point);
    EXPECT_GE(diff.count(), 29);
    EXPECT_LE(diff.count(), 31);
}

TEST_F(ChunkTimestampTest, ConversionWithZeroTimestamp) {
    ChunkTimestamp timestamp;
    // All fields are 0 (epoch)
    
    auto creation = timestamp.getCreationTime();
    auto modified = timestamp.getLastModified();
    auto accessed = timestamp.getLastAccessed();
    
    // Should all be at epoch (January 1, 1970)
    auto epoch = std::chrono::system_clock::time_point{};
    EXPECT_EQ(creation, epoch);
    EXPECT_EQ(modified, epoch);
    EXPECT_EQ(accessed, epoch);
}

// ============================================================================
// Age Calculation Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetAgeWithRecentCreation) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getTimeNSecondsAgo(100);  // Created 100 seconds ago
    
    auto age = timestamp.getAge();
    
    EXPECT_GE(age.count(), 99);   // At least 99 seconds
    EXPECT_LE(age.count(), 101);  // At most 101 seconds
}

TEST_F(ChunkTimestampTest, GetAgeWithOldCreation) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getTimeNSecondsAgo(3600);  // Created 1 hour ago
    
    auto age = timestamp.getAge();
    
    EXPECT_GE(age.count(), 3599);
    EXPECT_LE(age.count(), 3601);
}

TEST_F(ChunkTimestampTest, GetAgeWithZeroCreationTime) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = 0;  // Epoch
    
    auto age = timestamp.getAge();
    
    // Should be approximately current time since epoch
    auto now = std::chrono::system_clock::now();
    auto expected_age = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()
    );
    
    // Within 1 second tolerance
    EXPECT_NEAR(age.count(), expected_age.count(), 1);
}

TEST_F(ChunkTimestampTest, GetAgeIncreasesContinuously) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getCurrentTimeNs();
    
    auto age1 = timestamp.getAge();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    auto age2 = timestamp.getAge();
    
    // Second measurement should show older age
    EXPECT_GT(age2.count(), age1.count());
}

// ============================================================================
// Time Since Modified Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedRecent) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getTimeNSecondsAgo(50);  // Modified 50 seconds ago
    
    auto time_since = timestamp.getTimeSinceModified();
    
    EXPECT_GE(time_since.count(), 49);
    EXPECT_LE(time_since.count(), 51);
}

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedOld) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getTimeNSecondsAgo(7200);  // Modified 2 hours ago
    
    auto time_since = timestamp.getTimeSinceModified();
    
    EXPECT_GE(time_since.count(), 7199);
    EXPECT_LE(time_since.count(), 7201);
}

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedJustNow) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getCurrentTimeNs();
    
    auto time_since = timestamp.getTimeSinceModified();
    
    // Should be very small (less than 1 second)
    EXPECT_LE(time_since.count(), 1);
}

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedWithZero) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = 0;  // Never modified (or at epoch)
    
    auto time_since = timestamp.getTimeSinceModified();
    
    // Should be approximately time since epoch
    auto now = std::chrono::system_clock::now();
    auto expected = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()
    );
    
    EXPECT_NEAR(time_since.count(), expected.count(), 1);
}

// ============================================================================
// Time Since Accessed Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedRecent) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getTimeNSecondsAgo(30);  // Accessed 30 seconds ago
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    EXPECT_GE(time_since.count(), 29);
    EXPECT_LE(time_since.count(), 31);
}

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedLongAgo) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getTimeNSecondsAgo(86400);  // Accessed 24 hours ago
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    EXPECT_GE(time_since.count(), 86399);
    EXPECT_LE(time_since.count(), 86401);
}

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedJustNow) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getCurrentTimeNs();
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    // Should be very small
    EXPECT_LE(time_since.count(), 1);
}

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedWithZero) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = 0;
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    // Should be approximately time since epoch
    auto now = std::chrono::system_clock::now();
    auto expected = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()
    );
    
    EXPECT_NEAR(time_since.count(), expected.count(), 1);
}

// ============================================================================
// Realistic Timestamp Scenario Tests
// ============================================================================

TEST_F(ChunkTimestampTest, TypicalChunkLifecycle) {
    ChunkTimestamp timestamp;
    
    // Chunk created 1 hour ago
    timestamp.creation_time_ns = getTimeNSecondsAgo(3600);
    
    // Last modified 30 minutes ago
    timestamp.last_modified_ns = getTimeNSecondsAgo(1800);
    
    // Last accessed 5 minutes ago
    timestamp.last_accessed_ns = getTimeNSecondsAgo(300);
    
    // Accessed 10 times
    timestamp.access_count = 10;
    
    // Verify age
    auto age = timestamp.getAge();
    EXPECT_GE(age.count(), 3599);
    EXPECT_LE(age.count(), 3601);
    
    // Verify time since modified
    auto since_modified = timestamp.getTimeSinceModified();
    EXPECT_GE(since_modified.count(), 1799);
    EXPECT_LE(since_modified.count(), 1801);
    
    // Verify time since accessed
    auto since_accessed = timestamp.getTimeSinceAccessed();
    EXPECT_GE(since_accessed.count(), 299);
    EXPECT_LE(since_accessed.count(), 301);
    
    // Verify access count
    EXPECT_EQ(timestamp.access_count, 10);
}

TEST_F(ChunkTimestampTest, NewlyCreatedChunk) {
    ChunkTimestamp timestamp;
    
    auto now_ns = getCurrentTimeNs();
    timestamp.creation_time_ns = now_ns;
    timestamp.last_modified_ns = now_ns;
    timestamp.last_accessed_ns = now_ns;
    timestamp.access_count = 0;
    
    // All time-since values should be near zero
    EXPECT_LE(timestamp.getAge().count(), 1);
    EXPECT_LE(timestamp.getTimeSinceModified().count(), 1);
    EXPECT_LE(timestamp.getTimeSinceAccessed().count(), 1);
    EXPECT_EQ(timestamp.access_count, 0);
}

TEST_F(ChunkTimestampTest, FrequentlyAccessedChunk) {
    ChunkTimestamp timestamp;
    
    // Created a week ago
    timestamp.creation_time_ns = getTimeNSecondsAgo(604800);
    
    // Modified yesterday
    timestamp.last_modified_ns = getTimeNSecondsAgo(86400);
    
    // Accessed just now
    timestamp.last_accessed_ns = getCurrentTimeNs();
    
    // Accessed 1000 times
    timestamp.access_count = 1000;
    
    auto age = timestamp.getAge();
    EXPECT_GE(age.count(), 604799);
    EXPECT_LE(age.count(), 604801);
    
    auto since_accessed = timestamp.getTimeSinceAccessed();
    EXPECT_LE(since_accessed.count(), 1);
    
    EXPECT_EQ(timestamp.access_count, 1000);
}

TEST_F(ChunkTimestampTest, StaleChunk) {
    ChunkTimestamp timestamp;
    
    // Created 30 days ago
    timestamp.creation_time_ns = getTimeNSecondsAgo(2592000);
    
    // Last modified 30 days ago
    timestamp.last_modified_ns = getTimeNSecondsAgo(2592000);
    
    // Last accessed 7 days ago
    timestamp.last_accessed_ns = getTimeNSecondsAgo(604800);
    
    // Only accessed once
    timestamp.access_count = 1;
    
    auto age = timestamp.getAge();
    EXPECT_GE(age.count(), 2591999);
    
    auto since_modified = timestamp.getTimeSinceModified();
    EXPECT_GE(since_modified.count(), 2591999);
    
    auto since_accessed = timestamp.getTimeSinceAccessed();
    EXPECT_GE(since_accessed.count(), 604799);
    EXPECT_LE(since_accessed.count(), 604801);
}

// ============================================================================
// Comparison and Relative Time Tests
// ============================================================================

TEST_F(ChunkTimestampTest, CompareModifiedVsAccessed) {
    ChunkTimestamp timestamp;
    
    // Modified 1 hour ago
    timestamp.last_modified_ns = getTimeNSecondsAgo(3600);
    
    // Accessed 10 minutes ago (more recent)
    timestamp.last_accessed_ns = getTimeNSecondsAgo(600);
    
    auto since_modified = timestamp.getTimeSinceModified();
    auto since_accessed = timestamp.getTimeSinceAccessed();
    
    // Accessed should be more recent (smaller time-since value)
    EXPECT_LT(since_accessed.count(), since_modified.count());
}

TEST_F(ChunkTimestampTest, AccessCountRanges) {
    ChunkTimestamp timestamp;
    
    // Test various access counts
    std::vector<uint64_t> counts = {0, 1, 10, 100, 1000, 1000000, UINT64_MAX};
    
    for (auto count : counts) {
        timestamp.access_count = count;
        EXPECT_EQ(timestamp.access_count, count);
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(ChunkTimestampTest, MaxInt64Timestamp) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = INT64_MAX;
    
    // Should not crash or overflow
    auto time_point = timestamp.getCreationTime();
    
    // This is far in the future, age will be negative when cast
    // (implementation-defined behavior, but should not crash)
    auto age = timestamp.getAge();
    (void)age;  // Just verify it doesn't crash
}

TEST_F(ChunkTimestampTest, NegativeTimestamp) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = -1000000000;  // Before epoch
    
    // Should not crash
    auto time_point = timestamp.getCreationTime();
    auto age = timestamp.getAge();
    (void)time_point;
    (void)age;
}

TEST_F(ChunkTimestampTest, AllFieldsMaxValues) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = INT64_MAX;
    timestamp.last_modified_ns = INT64_MAX;
    timestamp.last_accessed_ns = INT64_MAX;
    timestamp.access_count = UINT64_MAX;
    
    // Should not crash
    EXPECT_EQ(timestamp.creation_time_ns, INT64_MAX);
    EXPECT_EQ(timestamp.last_modified_ns, INT64_MAX);
    EXPECT_EQ(timestamp.last_accessed_ns, INT64_MAX);
    EXPECT_EQ(timestamp.access_count, UINT64_MAX);
}

TEST_F(ChunkTimestampTest, TimeProgressionSimulation) {
    ChunkTimestamp timestamp;
    
    // Simulate chunk lifecycle
    auto start_time = getCurrentTimeNs();
    timestamp.creation_time_ns = start_time;
    timestamp.last_modified_ns = start_time;
    timestamp.last_accessed_ns = start_time;
    timestamp.access_count = 0;
    
    // Wait and access
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timestamp.last_accessed_ns = getCurrentTimeNs();
    timestamp.access_count++;
    
    // Wait and access again
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timestamp.last_accessed_ns = getCurrentTimeNs();
    timestamp.access_count++;
    
    // Wait and modify
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timestamp.last_modified_ns = getCurrentTimeNs();
    
    // Verify state
    EXPECT_EQ(timestamp.access_count, 2);
    EXPECT_GT(timestamp.last_accessed_ns, start_time);
    EXPECT_GT(timestamp.last_modified_ns, start_time);
    EXPECT_EQ(timestamp.creation_time_ns, start_time);
    
    // Age should be at least 150ms
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - timestamp.getCreationTime()
    );
    EXPECT_GE(age_ms.count(), 150);
}
// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}