# Rolling Bonsai V2: Component PRD - Center-Origin Chunks

## Implementation Tracking

### Components Selected for V2:
1. **Center-Origin Chunks** ← Current

### Components Not Yet Selected:
- Hysteresis on Transitions
- Predictive Pre-Loading
- Policy Composition
- Async I/O
- LRU Caching
- Lazy Saves with Dirty Tracking

---

## Overview

Center-origin chunks form the foundational coordinate system for Rolling Bonsai V2. This component replaces the corner-origin system from V1, addressing the initialization thrashing issue by positioning chunk boundaries away from common spawn points.

## Motivation

**V1 Problem:**
- Robot spawning at world origin (0,0,0) is at the corner of chunk (0,0,0)
- Localization noise of ±10cm causes robot position to fluctuate between 4 adjacent chunks
- Each fluctuation triggers chunk transition logic, causing I/O thrashing
- System unstable during initialization phase

**V2 Solution:**
- Robot at world origin (0,0,0) is at the **center** of chunk (0,0,0)
- Distance to nearest boundary: chunk_size/2 (e.g., 5m for 10m chunks)
- Localization noise must exceed 5m to cause transition (impossible for typical SLAM systems)
- System stable at all positions, not just origin

## Requirements

### Functional Requirements

**FR-1.1: Chunk Coordinate Definition**
- A chunk with coordinate (i, j, k) shall be centered at world position:
  - `center_x = i × chunk_size`
  - `center_y = j × chunk_size`
  - `center_z = k × chunk_size`

**FR-1.2: Chunk Spatial Extent**
- Each chunk shall span a cubic region:
  - X-axis: `[center_x - chunk_size/2, center_x + chunk_size/2)`
  - Y-axis: `[center_y - chunk_size/2, center_y + chunk_size/2)`
  - Z-axis: `[center_z - chunk_size/2, center_z + chunk_size/2)`
- Boundaries use half-open interval: `[min, max)` to ensure no overlap

**FR-1.3: World Position to Chunk Coordinate Conversion**
- The system shall provide a function `positionToChunk(x, y, z) -> (i, j, k)` using:
  - `i = round(x / chunk_size)`
  - `j = round(y / chunk_size)`
  - `k = round(z / chunk_size)`
- Where `round()` returns the nearest integer (0.5 rounds up)

**FR-1.4: Chunk Coordinate to World Position Conversion**
- The system shall provide a function `chunkToPosition(i, j, k) -> (x, y, z)` using:
  - `x = i × chunk_size`
  - `y = j × chunk_size`
  - `z = k × chunk_size`
- This returns the **center** of the chunk

**FR-1.5: Boundary Position Calculation**
- The system shall provide functions to calculate chunk boundaries:
  - `getChunkMinBound(coord, axis) -> double`: Returns minimum boundary
  - `getChunkMaxBound(coord, axis) -> double`: Returns maximum boundary
- For chunk (i, j, k) along X-axis: min = i×size - size/2, max = i×size + size/2

**FR-1.6: Chunk Size Configuration**
- Chunk size shall be configurable at system initialization
- Chunk size must be > 0.0
- Once set, chunk size is immutable for the lifetime of a RollingBonsai instance
- Typical values: 5.0m to 50.0m

**FR-1.7: Chunk Storage and Lookup**
- The system shall use `std::vector` for storing active chunks (primary implementation)
- Linear search shall be used for chunk lookup
- Optional: `std::unordered_map` implementation for benchmarking comparison

**FR-1.8: Coordinate Range Support**
- The system shall support chunk coordinates in the range `[-2^31, 2^31-1]` for each axis
- Using `int32_t` for chunk coordinates
- At 10m chunk size: ±21,474,836 km coverage per axis

### Non-Functional Requirements

**NFR-1.1: Conversion Performance**
- `positionToChunk()` shall complete in < 100ns (single call)
- `chunkToPosition()` shall complete in < 50ns (single call)
- No heap allocations during conversion
- Suitable for hot-path usage (called every frame)

**NFR-1.2: Lookup Performance**
- Chunk lookup in vector (27-100 chunks) shall complete in < 200ns (average)
- Linear search is acceptable for working set sizes typical in robotics
- If working set exceeds 500 chunks, consider hash map upgrade

**NFR-1.3: Numerical Stability**
- Coordinate conversion shall be stable for positions up to ±10^6 meters
- Round-trip conversion error shall be < 1mm:
  - `pos1 -> chunk -> pos2`: `|pos1 - pos2| < 0.001m`
- No accumulation of floating-point errors over repeated conversions

**NFR-1.4: Thread Safety**
- All conversion functions shall be thread-safe and reentrant
- No mutable shared state in conversion logic
- Const-correct API

**NFR-1.5: Memory Footprint**
- Chunk coordinate (`ChunkCoord` struct) shall be exactly 12 bytes (3 × int32_t)
- No vtable overhead (no virtual functions in ChunkCoord)
- Suitable for storage in large collections (millions of coordinates)
- Vector storage is more cache-friendly than hash map for small N

## Design

### Core Data Structures

```cpp
namespace rolling_bonsai {

// Chunk coordinate in grid space
struct ChunkCoord {
    int32_t x;
    int32_t y;
    int32_t z;
    
    // Default constructible
    ChunkCoord() : x(0), y(0), z(0) {}
    
    // Value constructor
    ChunkCoord(int32_t x_, int32_t y_, int32_t z_) 
        : x(x_), y(y_), z(z_) {}
    
    // Equality comparison (required for linear search)
    bool operator==(const ChunkCoord& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    bool operator!=(const ChunkCoord& other) const {
        return !(*this == other);
    }
    
    // Arithmetic (for neighbor calculation)
    ChunkCoord operator+(const ChunkCoord& offset) const {
        return ChunkCoord{x + offset.x, y + offset.y, z + offset.z};
    }
    
    ChunkCoord operator-(const ChunkCoord& offset) const {
        return ChunkCoord{x - offset.x, y - offset.y, z - offset.z};
    }
};

// Optional: Hash function for benchmarking unordered_map
// Not used in primary implementation, but available for performance comparison
struct ChunkCoordHash {
    std::size_t operator()(const ChunkCoord& coord) const {
        // FNV-1a hash
        std::size_t hash = 2166136261u;
        hash ^= static_cast<std::size_t>(coord.x);
        hash *= 16777619u;
        hash ^= static_cast<std::size_t>(coord.y);
        hash *= 16777619u;
        hash ^= static_cast<std::size_t>(coord.z);
        hash *= 16777619u;
        return hash;
    }
};

// 3D position in world space
struct Position3D {
    double x;
    double y;
    double z;
    
    Position3D() : x(0), y(0), z(0) {}
    Position3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

} // namespace rolling_bonsai
```

### Coordinate Conversion API

```cpp
namespace rolling_bonsai {

class ChunkCoordinateSystem {
public:
    // Constructor
    explicit ChunkCoordinateSystem(double chunk_size);
    
    // World position -> Chunk coordinate
    ChunkCoord positionToChunk(const Position3D& pos) const;
    ChunkCoord positionToChunk(double x, double y, double z) const;
    
    // Chunk coordinate -> World position (center)
    Position3D chunkToPosition(const ChunkCoord& coord) const;
    
    // Chunk boundary queries
    Position3D getChunkMinBound(const ChunkCoord& coord) const;
    Position3D getChunkMaxBound(const ChunkCoord& coord) const;
    
    // Specific axis boundaries
    double getChunkMinBound(const ChunkCoord& coord, int axis) const;
    double getChunkMaxBound(const ChunkCoord& coord, int axis) const;
    
    // Query chunk size
    double getChunkSize() const { return chunk_size_; }
    
    // Test if position is inside chunk
    bool isPositionInChunk(const Position3D& pos, const ChunkCoord& chunk) const;
    
    // Distance from position to chunk boundary (minimum distance to any face)
    double distanceToBoundary(const Position3D& pos, const ChunkCoord& chunk) const;
    
private:
    double chunk_size_;
    double half_chunk_size_;  // Cached for performance
};

} // namespace rolling_bonsai
```

### Chunk Storage Approach

**Primary Implementation: Vector with Linear Search**

```cpp
namespace rolling_bonsai {

// Simple storage for active chunks
template<typename T>
class ChunkStorage {
public:
    // Add a chunk
    void insert(const ChunkCoord& coord, T value) {
        chunks_.emplace_back(coord, std::move(value));
    }
    
    // Find a chunk (returns nullptr if not found)
    T* find(const ChunkCoord& coord) {
        for (auto& [chunk_coord, value] : chunks_) {
            if (chunk_coord == coord) {
                return &value;
            }
        }
        return nullptr;
    }
    
    // Remove a chunk
    bool erase(const ChunkCoord& coord) {
        auto it = std::find_if(chunks_.begin(), chunks_.end(),
            [&](const auto& pair) { return pair.first == coord; });
        
        if (it != chunks_.end()) {
            chunks_.erase(it);
            return true;
        }
        return false;
    }
    
    // Check if chunk exists
    bool contains(const ChunkCoord& coord) const {
        return std::find_if(chunks_.begin(), chunks_.end(),
            [&](const auto& pair) { return pair.first == coord; }) != chunks_.end();
    }
    
    // Iterate all chunks
    auto begin() { return chunks_.begin(); }
    auto end() { return chunks_.end(); }
    auto begin() const { return chunks_.begin(); }
    auto end() const { return chunks_.end(); }
    
    // Size
    size_t size() const { return chunks_.size(); }
    bool empty() const { return chunks_.empty(); }
    
    // Clear all
    void clear() { chunks_.clear(); }
    
private:
    std::vector<std::pair<ChunkCoord, T>> chunks_;
};

} // namespace rolling_bonsai
```

**Rationale for Vector:**
- **Simplicity:** No hash function, no collision handling
- **Cache-friendly:** Contiguous memory, sequential access
- **Performance:** For N < 100 chunks, linear search is < 200ns
- **Memory efficient:** No hash table overhead, no bucket allocations
- **Easier debugging:** Can iterate and inspect all chunks easily

**When to Consider Hash Map:**
- Working set exceeds 500 chunks (rare in robotics)
- Profiling shows lookup is bottleneck (measure first!)
- Need O(1) worst-case guarantees (not just average)

### Implementation Considerations

#### 1. Rounding Behavior

**Issue:** `std::round()` behavior at exact 0.5 values
- IEEE 754 "round half to even" (banker's rounding)
- We want consistent "round half up" for symmetry

**Solution:**
```cpp
inline int32_t roundToInt(double value) {
    // Round half away from zero for symmetry
    return static_cast<int32_t>(value + (value >= 0 ? 0.5 : -0.5));
}

ChunkCoord positionToChunk(double x, double y, double z) const {
    return ChunkCoord{
        roundToInt(x / chunk_size_),
        roundToInt(y / chunk_size_),
        roundToInt(z / chunk_size_)
    };
}
```

#### 2. Boundary Epsilon Tolerance

**Issue:** Floating-point positions exactly on boundaries
- Position x = 5.0 with chunk_size = 10.0
- Which chunk: (0,y,z) or (1,y,z)?

**Decision:** Half-open intervals `[min, max)`
- Minimum boundary inclusive
- Maximum boundary exclusive
- Position exactly on max boundary belongs to next chunk

**Implementation:**
```cpp
bool isPositionInChunk(const Position3D& pos, const ChunkCoord& chunk) const {
    const double half = half_chunk_size_;
    const double cx = chunk.x * chunk_size_;
    const double cy = chunk.y * chunk_size_;
    const double cz = chunk.z * chunk_size_;
    
    return (pos.x >= cx - half) && (pos.x < cx + half)
        && (pos.y >= cy - half) && (pos.y < cy + half)
        && (pos.z >= cz - half) && (pos.z < cz + half);
}
```

#### 3. Vector vs Hash Map Performance Comparison

**Expected Performance (27 chunks):**
```
Vector linear search:  ~50-100ns
Hash map lookup:       ~30ns
Difference:            ~20-70ns (negligible in 1ms budget)
```

**Expected Performance (100 chunks):**
```
Vector linear search:  ~150-200ns
Hash map lookup:       ~30ns
Difference:            ~120-170ns (still < 0.02% of 1ms budget)
```

**Breakeven Point:**
```
Hash overhead (hash computation + indirection) ≈ 30ns
Linear search: 2ns per comparison × N
Breakeven: N ≈ 100-200 chunks (accounting for cache effects)
```

**Benchmark Both:**
Include both implementations and measure in your specific use case.

#### 4. Neighbor Calculation

**Common Operation:** Get the 26 neighbors of a chunk

**API:**
```cpp
// Get all 26 neighbors (3x3x3 - center)
std::vector<ChunkCoord> getNeighbors(const ChunkCoord& center, int radius = 1) const;

// Get face neighbors only (6 neighbors)
std::vector<ChunkCoord> getFaceNeighbors(const ChunkCoord& center) const;

// Iterate neighbors without allocation (for performance-critical code)
template<typename Func>
void forEachNeighbor(const ChunkCoord& center, int radius, Func func) const;
```

**Implementation:**
```cpp
template<typename Func>
void forEachNeighbor(const ChunkCoord& center, int radius, Func func) const {
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dz = -radius; dz <= radius; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;  // Skip center
                ChunkCoord neighbor{center.x + dx, center.y + dy, center.z + dz};
                func(neighbor);
            }
        }
    }
}
```

#### 5. Distance Calculations

**Use Case:** Determine how close robot is to chunk boundary

**API:**
```cpp
// Distance to nearest boundary in any direction
double distanceToBoundary(const Position3D& pos, const ChunkCoord& chunk) const;

// Distance to boundary in specific direction (axis: 0=x, 1=y, 2=z)
// Returns negative if past boundary (outside chunk)
double distanceToBoundary(const Position3D& pos, const ChunkCoord& chunk, int axis) const;
```

**Implementation:**
```cpp
double distanceToBoundary(const Position3D& pos, const ChunkCoord& chunk) const {
    const double half = half_chunk_size_;
    const double cx = chunk.x * chunk_size_;
    const double cy = chunk.y * chunk_size_;
    const double cz = chunk.z * chunk_size_;
    
    // Distance to each face
    double dx = std::min(std::abs(pos.x - (cx - half)), 
                         std::abs(pos.x - (cx + half)));
    double dy = std::min(std::abs(pos.y - (cy - half)), 
                         std::abs(pos.y - (cy + half)));
    double dz = std::min(std::abs(pos.z - (cz - half)), 
                         std::abs(pos.z - (cz + half)));
    
    // Return minimum (closest boundary)
    return std::min({dx, dy, dz});
}
```

### Edge Cases and Error Handling

#### Case 1: Invalid Chunk Size
```cpp
ChunkCoordinateSystem(double chunk_size) {
    if (chunk_size <= 0.0) {
        throw std::invalid_argument("Chunk size must be positive");
    }
    if (std::isnan(chunk_size) || std::isinf(chunk_size)) {
        throw std::invalid_argument("Chunk size must be finite");
    }
    chunk_size_ = chunk_size;
    half_chunk_size_ = chunk_size * 0.5;
}
```

#### Case 2: Extreme Coordinates
- Position = 10^9 meters → chunk coordinate ≈ 10^8 (within int32 range at 10m chunks)
- Position = 10^10 meters → chunk coordinate ≈ 10^9 (exceeds int32, overflow)

**Mitigation Option 1: Clamp**
```cpp
ChunkCoord positionToChunk(double x, double y, double z) const {
    // Clamp to prevent overflow
    const double max_pos = static_cast<double>(INT32_MAX) * chunk_size_;
    const double min_pos = static_cast<double>(INT32_MIN) * chunk_size_;
    
    x = std::clamp(x, min_pos, max_pos);
    y = std::clamp(y, min_pos, max_pos);
    z = std::clamp(z, min_pos, max_pos);
    
    return ChunkCoord{
        roundToInt(x / chunk_size_),
        roundToInt(y / chunk_size_),
        roundToInt(z / chunk_size_)
    };
}
```

**Mitigation Option 2: Document as precondition** (faster, user responsible)
- Acceptable for robotics where positions rarely exceed ±10km

#### Case 3: Position Exactly on Boundary
- Position = (5.0, 0, 0), chunk_size = 10.0
- Boundary between chunks (0,0,0) and (1,0,0)

**Behavior:**
- `positionToChunk(5.0, 0, 0)` → depends on rounding
- With round-half-up: (1, 0, 0)
- Consistent with half-open interval `[min, max)`

### Testing Strategy

#### Unit Tests

**Test 1: Origin Mapping**
```cpp
TEST(ChunkCoordinateSystem, OriginMapsToZeroChunk) {
    ChunkCoordinateSystem coord_sys(10.0);
    auto chunk = coord_sys.positionToChunk(0.0, 0.0, 0.0);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
}
```

**Test 2: Chunk Centers**
```cpp
TEST(ChunkCoordinateSystem, ChunkCenters) {
    ChunkCoordinateSystem coord_sys(10.0);
    
    // Chunk (1,0,0) centered at (10,0,0)
    auto pos = coord_sys.chunkToPosition(ChunkCoord(1, 0, 0));
    EXPECT_DOUBLE_EQ(pos.x, 10.0);
    EXPECT_DOUBLE_EQ(pos.y, 0.0);
    EXPECT_DOUBLE_EQ(pos.z, 0.0);
}
```

**Test 3: Round-Trip Conversion**
```cpp
TEST(ChunkCoordinateSystem, RoundTripConversion) {
    ChunkCoordinateSystem coord_sys(10.0);
    Position3D original(15.7, -23.4, 8.9);
    
    auto chunk = coord_sys.positionToChunk(original);
    auto center = coord_sys.chunkToPosition(chunk);
    
    // Should be within chunk boundaries
    EXPECT_TRUE(coord_sys.isPositionInChunk(original, chunk));
    
    // Center should be close to original (within chunk_size/2)
    double dx = std::abs(original.x - center.x);
    double dy = std::abs(original.y - center.y);
    double dz = std::abs(original.z - center.z);
    EXPECT_LT(dx, 5.0);
    EXPECT_LT(dy, 5.0);
    EXPECT_LT(dz, 5.0);
}
```

**Test 4: Boundary Behavior**
```cpp
TEST(ChunkCoordinateSystem, BoundaryBehavior) {
    ChunkCoordinateSystem coord_sys(10.0);
    
    // Position exactly on boundary
    Position3D on_boundary(5.0, 0.0, 0.0);
    auto chunk = coord_sys.positionToChunk(on_boundary);
    
    // Should consistently choose one side
    EXPECT_TRUE(chunk == ChunkCoord(0,0,0) || chunk == ChunkCoord(1,0,0));
    
    // Slightly inside should be unambiguous
    Position3D inside(4.99, 0.0, 0.0);
    EXPECT_EQ(coord_sys.positionToChunk(inside), ChunkCoord(0,0,0));
    
    Position3D outside(5.01, 0.0, 0.0);
    EXPECT_EQ(coord_sys.positionToChunk(outside), ChunkCoord(1,0,0));
}
```

**Test 5: Negative Coordinates**
```cpp
TEST(ChunkCoordinateSystem, NegativeCoordinates) {
    ChunkCoordinateSystem coord_sys(10.0);
    
    auto chunk = coord_sys.positionToChunk(-15.3, 7.8, -2.1);
    EXPECT_EQ(chunk.x, -2);  // round(-1.53) = -2
    EXPECT_EQ(chunk.y, 1);   // round(0.78) = 1
    EXPECT_EQ(chunk.z, 0);   // round(-0.21) = 0
}
```

**Test 6: Distance to Boundary**
```cpp
TEST(ChunkCoordinateSystem, DistanceToBoundary) {
    ChunkCoordinateSystem coord_sys(10.0);
    ChunkCoord chunk(0, 0, 0);
    
    // At center: distance = 5.0 (half chunk size)
    Position3D center(0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(coord_sys.distanceToBoundary(center, chunk), 5.0);
    
    // Near boundary: distance = 0.5
    Position3D near(4.5, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(coord_sys.distanceToBoundary(near, chunk), 0.5);
    
    // On boundary: distance = 0.0
    Position3D on(5.0, 0.0, 0.0);
    EXPECT_NEAR(coord_sys.distanceToBoundary(on, chunk), 0.0, 1e-9);
}
```

**Test 7: Neighbor Calculation**
```cpp
TEST(ChunkCoordinateSystem, NeighborCalculation) {
    ChunkCoordinateSystem coord_sys(10.0);
    ChunkCoord center(0, 0, 0);
    
    auto neighbors = coord_sys.getNeighbors(center, 1);
    
    // Should have 26 neighbors (3^3 - 1)
    EXPECT_EQ(neighbors.size(), 26);
    
    // Check specific neighbors
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), 
                         ChunkCoord(1,0,0)) != neighbors.end());
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), 
                         ChunkCoord(0,1,0)) != neighbors.end());
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), 
                         ChunkCoord(0,0,1)) != neighbors.end());
    
    // Center should NOT be in neighbors
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), 
                         ChunkCoord(0,0,0)) == neighbors.end());
}
```

**Test 8: Vector Storage Performance**
```cpp
TEST(ChunkStorage, LinearSearchPerformance) {
    ChunkStorage<int> storage;
    
    // Load 27 chunks (typical neighborhood)
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                storage.insert(ChunkCoord{dx, dy, dz}, 42);
            }
        }
    }
    
    EXPECT_EQ(storage.size(), 27);
    
    // Search for chunk (worst case: at end)
    ChunkCoord target{1, 1, 1};
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100000; ++i) {
        auto* result = storage.find(target);
        benchmark::DoNotOptimize(result);
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    auto avg_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        end - start).count() / 100000;
    
    // Should be < 200ns per search for 27 chunks
    EXPECT_LT(avg_ns, 200);
}
```

**Test 9: Vector vs Hash Map Comparison (Optional)**
```cpp
TEST(ChunkStorage, VectorVsHashMapBenchmark) {
    const int NUM_CHUNKS = 100;
    const int NUM_SEARCHES = 100000;
    
    // Setup vector storage
    ChunkStorage<int> vector_storage;
    std::unordered_map<ChunkCoord, int, ChunkCoordHash> hash_storage;
    
    // Populate both
    for (int i = 0; i < NUM_CHUNKS; ++i) {
        ChunkCoord coord{i, 0, 0};
        vector_storage.insert(coord, i);
        hash_storage[coord] = i;
    }
    
    // Benchmark vector
    auto start_vec = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_SEARCHES; ++i) {
        ChunkCoord target{i % NUM_CHUNKS, 0, 0};
        auto* result = vector_storage.find(target);
        benchmark::DoNotOptimize(result);
    }
    auto end_vec = std::chrono::high_resolution_clock::now();
    auto vec_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        end_vec - start_vec).count() / NUM_SEARCHES;
    
    // Benchmark hash map
    auto start_hash = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_SEARCHES; ++i) {
        ChunkCoord target{i % NUM_CHUNKS, 0, 0};
        auto it = hash_storage.find(target);
        benchmark::DoNotOptimize(it);
    }
    auto end_hash = std::chrono::high_resolution_clock::now();
    auto hash_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        end_hash - start_hash).count() / NUM_SEARCHES;
    
    std::cout << "Vector search (N=" << NUM_CHUNKS << "): " 
              << vec_ns << "ns\n";
    std::cout << "Hash map search (N=" << NUM_CHUNKS << "): " 
              << hash_ns << "ns\n";
    std::cout << "Ratio: " << (double)vec_ns / hash_ns << "x\n";
    
    // For 100 chunks, vector should still be < 500ns
    EXPECT_LT(vec_ns, 500);
}
```

#### Performance Tests

**Benchmark 1: Conversion Performance**
```cpp
BENCHMARK(PositionToChunk) {
    ChunkCoordinateSystem coord_sys(10.0);
    Position3D pos(123.456, -789.012, 345.678);
    
    for (int i = 0; i < 1000000; ++i) {
        benchmark::DoNotOptimize(coord_sys.positionToChunk(pos));
    }
}
// Target: < 100ns per call
```

**Benchmark 2: Vector Search Scaling**
```cpp
void BM_VectorSearch(benchmark::State& state) {
    const int N = state.range(0);
    ChunkStorage<int> storage;
    
    for (int i = 0; i < N; ++i) {
        storage.insert(ChunkCoord{i, 0, 0}, i);
    }
    
    ChunkCoord target{N-1, 0, 0};  // Worst case: last element
    
    for (auto _ : state) {
        auto* result = storage.find(target);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_VectorSearch)->Arg(27)->Arg(100)->Arg(500)->Arg(1000);
```

### Integration Considerations

#### With Future Components

**Hysteresis System:**
- Will use `distanceToBoundary()` to determine transition timing
- Needs fast boundary queries (< 1μs)

**Predictive Preloading:**
- Will use `positionToChunk()` to determine current and upcoming chunks
- Needs fast conversion in hot path (called every update)

**Policy System:**
- Policies will use `ChunkCoord` as primary key
- Vector storage simplifies iteration over active chunks for policy evaluation

**Storage Backend:**
- Will serialize `ChunkCoord` as part of chunk metadata
- Should be stable across versions (int32_t x3 = 12 bytes)

### Documentation Requirements

**API Documentation:**
- Doxygen comments for all public functions
- Usage examples for common operations
- Performance characteristics (Big-O, typical latency)
- Explanation of vector vs hash map tradeoffs

**Design Documentation:**
- Rationale for center-origin over corner-origin
- Boundary behavior and edge cases
- Coordinate system conventions (right-handed, etc.)
- Storage approach decision (vector vs hash map)

**Migration Guide:**
- How to convert V1 (corner-origin) data to V2
- API differences from V1

### Acceptance Criteria

- [ ] `ChunkCoord` struct is exactly 12 bytes (3 × int32_t)
- [ ] Robot at world origin (0,0,0) maps to chunk (0,0,0)
- [ ] `positionToChunk()` completes in < 100ns (average, 1M iterations)
- [ ] Round-trip conversion has < 1mm error for positions within ±10km
- [ ] Vector storage lookup for 27 chunks completes in < 200ns (average)
- [ ] Vector storage lookup for 100 chunks completes in < 500ns (average)
- [ ] All unit tests pass (9 tests minimum)
- [ ] Supports positions up to ±2^20 meters without overflow (at 10m chunks)
- [ ] Thread-safe: concurrent calls from multiple threads produce correct results
- [ ] Zero heap allocations in conversion functions (verified with profiler)
- [ ] Documentation complete: API docs + design doc + migration guide
- [ ] Benchmark data comparing vector vs hash map for N ∈ {27, 100, 500, 1000}

---

## Implementation Plan

### Phase 1: Core Data Structures (1 day)
- Implement `ChunkCoord` struct with equality operators
- Implement `Position3D` struct  
- Implement `ChunkCoordHash` (optional, for benchmarking)
- Unit tests for comparison

### Phase 2: Coordinate System (2 days)
- Implement `ChunkCoordinateSystem` class
- Conversion functions: position ↔ chunk
- Boundary query functions
- Unit tests for all conversion functions

### Phase 3: Storage Implementation (1 day)
- Implement `ChunkStorage<T>` template with vector backend
- Implement insert, find, erase, contains operations
- Optional: Hash map variant for benchmarking
- Unit tests for storage operations

### Phase 4: Utility Functions (1 day)
- Neighbor calculation functions
- Distance to boundary functions
- In-chunk testing functions
- Unit tests for utilities

### Phase 5: Optimization & Testing (1 day)
- Performance benchmarks for conversion
- Performance benchmarks for storage (vector vs hash)
- Thread safety verification
- Edge case testing
- Scaling tests (N = 27, 100, 500, 1000)

### Phase 6: Documentation (1 day)
- API documentation (Doxygen)
- Design document
- Usage examples
- Migration guide from V1
- Benchmark results and analysis

**Total Estimated Time: 7 days**

---

## Risks and Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Vector search too slow for large working sets | Medium | Low | Benchmark early, keep hash map as fallback |
| Floating-point rounding inconsistencies | High | Medium | Use explicit rounding function with defined behavior |
| Performance regression from V1 | Medium | Low | Benchmark conversion + lookup combined latency |
| Boundary ambiguity confusion | Low | Medium | Clear documentation, consistent half-open intervals |
| Vector overhead during insertion/deletion | Low | Low | Chunks loaded/evicted infrequently (not hot path) |

---

## Design Decisions

### Why Vector Over Hash Map?

**Decision:** Use `std::vector` with linear search as primary implementation

**Reasoning:**
1. **Typical working set:** 27-100 chunks
2. **Linear search cost:** ~50-200ns for this range
3. **Acceptable overhead:** < 0.02% of 1ms control loop budget
4. **Simplicity:** No hash function, easier to debug
5. **Cache-friendly:** Contiguous memory layout
6. **Lower memory overhead:** No hash table buckets

**When to reconsider:**
- Working set routinely exceeds 500 chunks
- Profiling shows chunk lookup is measurable bottleneck
- Random access patterns (not spatial locality)

**Keeping hash option:**
- Include `ChunkCoordHash` for easy A/B testing
- Can swap storage backend without changing API
- Benchmark both on real robot data

---

**Component Version:** 1.0  
**Component Status:** Ready for Implementation  
**Dependencies:** None (foundational component)  
**Estimated Effort:** 7 person-days
