# Rolling Bonsai V2: Component PRD - Chunk Loading Policy

## Implementation Tracking

### Components Selected for V2:
1. **Center-Origin Chunks** ✓ Complete
2. **Hysteresis on Transitions** ✓ Complete
3. **Chunk Loading Policy** ← Current
4. **Async I/O with Lazy Saves** (Next)

### Components Slated for V3:
- Predictive Pre-Loading
- Policy Composition (combining multiple policies)
- LRU Caching

---

## Overview

The Chunk Loading Policy system provides a configurable, extensible framework for determining which chunks should be loaded into memory and which should be evicted. Different robot types and use cases require different chunk management strategies. This component enables users to select from built-in policies or implement custom policies tailored to their specific needs.

**V2 Scope:** Single policy selection (no composition). Users choose ONE policy that best fits their use case.

## Motivation

**Problem: One Size Doesn't Fit All**

Different robots have vastly different requirements:

```
Ground Robot (warehouse navigation):
- Only needs chunks at ground level (z = 0, 1, 2)
- Wastes memory loading underground/aerial chunks
- 3D neighborhood loads 27 chunks, but only uses 9

Aerial Drone (inspection):
- Needs full 3D neighborhood
- May ignore very old map data (dynamic environment)
- 27 chunks all relevant

High-Speed Robot:
- Needs larger neighborhood (sees farther ahead)
- 5×5×5 = 125 chunks instead of 3×3×3 = 27

Mission-Specific:
- Only load chunks along planned path
- Ignore chunks far from mission area
```

**Solution: Pluggable Policy System**

Allow users to select the policy that matches their robot:
```cpp
// Ground robot
auto policy = std::make_unique<PlanarNeighborhoodPolicy>(
    1,     // radius
    0, 2   // z_min, z_max
);

// Aerial robot with time filtering
auto policy = std::make_unique<TemporalNeighborhoodPolicy>(
    1,                          // radius
    std::chrono::hours(1)       // max_age
);

// Custom policy
auto policy = std::make_unique<MyCustomPolicy>(...);

// Set policy
rolling_bonsai.setPolicy(std::move(policy));
```

## Requirements

### Functional Requirements

**FR-4.1: Policy Interface**
- The system shall provide an abstract `ChunkLoadingPolicy` interface
- All policies shall implement three methods:
  - `shouldLoad(coord, context) -> bool`: Decide if chunk should be loaded
  - `shouldEvict(coord, context) -> bool`: Decide if chunk should be evicted
  - `getPriority(coord, context) -> float`: Get loading priority (0.0 = lowest, 1.0 = highest)

**FR-4.2: Policy Context**
- Policies shall receive a `PolicyContext` containing:
  - Robot position
  - Current chunk coordinate
  - Current time
  - Chunk metadata (creation time, last access time, access count)
- Context shall be read-only from policy perspective

**FR-4.3: Built-in Policies**
- The system shall provide the following built-in policies:
  - `NeighborhoodPolicy`: Load all chunks within radius R
  - `PlanarNeighborhoodPolicy`: Neighborhood + vertical range restriction
  - `TemporalNeighborhoodPolicy`: Neighborhood + age-based filtering
- Each policy shall be independently usable

**FR-4.4: Policy Selection**
- Users shall be able to set a policy at initialization
- Users shall be able to swap policies at runtime
- Policy change shall take effect on next update cycle
- System shall provide a default policy if none specified (NeighborhoodPolicy with radius=1)

**FR-4.5: Custom Policy Support**
- Users shall be able to implement custom policies by inheriting from `ChunkLoadingPolicy`
- Custom policies shall have access to all information in `PolicyContext`
- No restrictions on custom policy logic (within interface constraints)

**FR-4.6: Policy Validation**
- System shall detect if policy never loads any chunks (warning)
- System shall detect if policy loads excessive chunks (warning, e.g., > 1000)
- Warnings shall be logged but not prevent operation

### Non-Functional Requirements

**NFR-4.1: Policy Evaluation Performance**
- Policy evaluation for 27 neighborhood chunks shall complete in < 100μs
- `shouldLoad()` and `shouldEvict()` shall be const methods (no side effects)
- No heap allocations in policy evaluation hot path

**NFR-4.2: Memory Overhead**
- Policy object size shall be < 1KB
- Policy shall not store large data structures (use context for lookups)

**NFR-4.3: Extensibility**
- Adding a new built-in policy shall not require changes to core system
- Custom policies shall compile without modifying Rolling Bonsai headers

**NFR-4.4: Thread Safety**
- Policies shall be thread-safe for concurrent reads (const methods)
- Policy swap operation shall be thread-safe

## Design

### Core Components

#### 1. Policy Interface

```cpp
namespace RollingBonxai {

// Metadata about each chunk
struct ChunkMetadata {
    std::chrono::steady_clock::time_point creation_time;
    std::chrono::steady_clock::time_point last_access_time;
    uint64_t access_count = 0;
    bool is_dirty = false;
};

// Context provided to policies
struct PolicyContext {
    Position3D robot_pos;
    ChunkCoord current_chunk;
    std::chrono::steady_clock::time_point current_time;
    
    // Reference to metadata map (read-only from policy perspective)
    const std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash>& metadata;
    
    // Helper: get metadata for a chunk (returns nullptr if not found)
    const ChunkMetadata* getMetadata(const ChunkCoord& coord) const {
        auto it = metadata.find(coord);
        return it != metadata.end() ? &it->second : nullptr;
    }
};

// Abstract policy interface
class ChunkLoadingPolicy {
public:
    virtual ~ChunkLoadingPolicy() = default;
    
    // Should this chunk be loaded into memory?
    virtual bool shouldLoad(const ChunkCoord& coord, 
                           const PolicyContext& context) const = 0;
    
    // Should this chunk be evicted from memory?
    virtual bool shouldEvict(const ChunkCoord& coord,
                            const PolicyContext& context) const = 0;
    
    // What priority for loading? (0.0 = lowest, 1.0 = highest)
    // Used when multiple chunks need loading - higher priority loads first
    virtual float getPriority(const ChunkCoord& coord,
                             const PolicyContext& context) const = 0;
    
    // Optional: policy name for debugging/logging
    virtual std::string getName() const { return "ChunkLoadingPolicy"; }
};

} // namespace RollingBonxai
```

#### 2. Built-in Policies

**Neighborhood Policy:**
```cpp
namespace RollingBonxai {

// Load all chunks within Chebyshev distance <= radius
class NeighborhoodPolicy : public ChunkLoadingPolicy {
public:
    explicit NeighborhoodPolicy(int radius = 1) : radius_(radius) {
        if (radius < 0) {
            throw std::invalid_argument("Radius must be non-negative");
        }
    }
    
    bool shouldLoad(const ChunkCoord& coord, 
                   const PolicyContext& context) const override {
        int dx = std::abs(coord.x - context.current_chunk.x);
        int dy = std::abs(coord.y - context.current_chunk.y);
        int dz = std::abs(coord.z - context.current_chunk.z);
        
        // Chebyshev distance (max of absolute differences)
        int dist = std::max({dx, dy, dz});
        return dist <= radius_;
    }
    
    bool shouldEvict(const ChunkCoord& coord,
                    const PolicyContext& context) const override {
        // Evict anything not in load set
        return !shouldLoad(coord, context);
    }
    
    float getPriority(const ChunkCoord& coord,
                     const PolicyContext& context) const override {
        int dx = std::abs(coord.x - context.current_chunk.x);
        int dy = std::abs(coord.y - context.current_chunk.y);
        int dz = std::abs(coord.z - context.current_chunk.z);
        int dist = std::max({dx, dy, dz});
        
        // Closer chunks = higher priority
        // Priority inversely proportional to distance
        return 1.0f / (1.0f + dist);
    }
    
    std::string getName() const override { 
        return "NeighborhoodPolicy(radius=" + std::to_string(radius_) + ")";
    }
    
    int getRadius() const { return radius_; }
    
private:
    int radius_;
};

} // namespace RollingBonxai
```

**Planar Neighborhood Policy:**
```cpp
namespace RollingBonxai {

// Neighborhood policy with vertical restriction (for ground robots)
class PlanarNeighborhoodPolicy : public ChunkLoadingPolicy {
public:
    PlanarNeighborhoodPolicy(int radius, int z_min, int z_max)
        : radius_(radius), z_min_(z_min), z_max_(z_max) {
        if (radius < 0) {
            throw std::invalid_argument("Radius must be non-negative");
        }
        if (z_min > z_max) {
            throw std::invalid_argument("z_min must be <= z_max");
        }
    }
    
    bool shouldLoad(const ChunkCoord& coord,
                   const PolicyContext& context) const override {
        // Check vertical range first (cheap rejection)
        if (coord.z < z_min_ || coord.z > z_max_) {
            return false;
        }
        
        // Check neighborhood
        int dx = std::abs(coord.x - context.current_chunk.x);
        int dy = std::abs(coord.y - context.current_chunk.y);
        int dz = std::abs(coord.z - context.current_chunk.z);
        int dist = std::max({dx, dy, dz});
        
        return dist <= radius_;
    }
    
    bool shouldEvict(const ChunkCoord& coord,
                    const PolicyContext& context) const override {
        return !shouldLoad(coord, context);
    }
    
    float getPriority(const ChunkCoord& coord,
                     const PolicyContext& context) const override {
        if (!shouldLoad(coord, context)) {
            return 0.0f;
        }
        
        int dx = std::abs(coord.x - context.current_chunk.x);
        int dy = std::abs(coord.y - context.current_chunk.y);
        int dz = std::abs(coord.z - context.current_chunk.z);
        int dist = std::max({dx, dy, dz});
        
        return 1.0f / (1.0f + dist);
    }
    
    std::string getName() const override {
        return "PlanarNeighborhoodPolicy(radius=" + std::to_string(radius_) +
               ", z=[" + std::to_string(z_min_) + "," + std::to_string(z_max_) + "])";
    }
    
private:
    int radius_;
    int z_min_, z_max_;
};

} // namespace RollingBonxai
```

**Temporal Neighborhood Policy:**
```cpp
namespace RollingBonxai {

// Neighborhood policy that excludes chunks older than a threshold
class TemporalNeighborhoodPolicy : public ChunkLoadingPolicy {
public:
    TemporalNeighborhoodPolicy(int radius, std::chrono::seconds max_age)
        : radius_(radius), max_age_(max_age) {
        if (radius < 0) {
            throw std::invalid_argument("Radius must be non-negative");
        }
    }
    
    bool shouldLoad(const ChunkCoord& coord,
                   const PolicyContext& context) const override {
        // Check neighborhood
        int dx = std::abs(coord.x - context.current_chunk.x);
        int dy = std::abs(coord.y - context.current_chunk.y);
        int dz = std::abs(coord.z - context.current_chunk.z);
        int dist = std::max({dx, dy, dz});
        
        if (dist > radius_) {
            return false;
        }
        
        // Check age (if chunk exists)
        const ChunkMetadata* meta = context.getMetadata(coord);
        if (meta == nullptr) {
            return true;  // New chunk, no age restriction
        }
        
        auto age = context.current_time - meta->creation_time;
        return age < max_age_;
    }
    
    bool shouldEvict(const ChunkCoord& coord,
                    const PolicyContext& context) const override {
        return !shouldLoad(coord, context);
    }
    
    float getPriority(const ChunkCoord& coord,
                     const PolicyContext& context) const override {
        if (!shouldLoad(coord, context)) {
            return 0.0f;
        }
        
        // Base priority on distance
        int dx = std::abs(coord.x - context.current_chunk.x);
        int dy = std::abs(coord.y - context.current_chunk.y);
        int dz = std::abs(coord.z - context.current_chunk.z);
        int dist = std::max({dx, dy, dz});
        float dist_priority = 1.0f / (1.0f + dist);
        
        // Bonus for newer chunks
        const ChunkMetadata* meta = context.getMetadata(coord);
        if (meta == nullptr) {
            return dist_priority;  // New chunk, full priority
        }
        
        auto age = context.current_time - meta->creation_time;
        float age_seconds = std::chrono::duration<float>(age).count();
        float max_age_seconds = std::chrono::duration<float>(max_age_).count();
        
        // Newer = higher priority
        float age_factor = 1.0f - (age_seconds / max_age_seconds);
        age_factor = std::max(0.0f, age_factor);
        
        // Combine: 70% distance, 30% age
        return 0.7f * dist_priority + 0.3f * age_factor;
    }
    
    std::string getName() const override {
        auto age_seconds = std::chrono::duration_cast<std::chrono::seconds>(max_age_).count();
        return "TemporalNeighborhoodPolicy(radius=" + std::to_string(radius_) +
               ", max_age=" + std::to_string(age_seconds) + "s)";
    }
    
private:
    int radius_;
    std::chrono::seconds max_age_;
};

} // namespace RollingBonxai
```

#### 3. Policy Management in Rolling Bonsai

```cpp
namespace RollingBonxai {

class RollingBonsai {
public:
    // Constructor with optional policy
    RollingBonsai(double chunk_size,
                 std::unique_ptr<ChunkLoadingPolicy> policy = nullptr,
                 /* other params */) 
        : chunk_size_(chunk_size) {
        
        // Default policy if none provided
        if (!policy) {
            policy_ = std::make_unique<NeighborhoodPolicy>(1);
        } else {
            policy_ = std::move(policy);
        }
    }
    
    // Change policy at runtime
    void setPolicy(std::unique_ptr<ChunkLoadingPolicy> new_policy) {
        if (!new_policy) {
            throw std::invalid_argument("Policy cannot be null");
        }
        
        std::lock_guard<std::mutex> lock(policy_mutex_);
        policy_ = std::move(new_policy);
        policy_changed_ = true;
    }
    
    // Get policy name (for debugging)
    std::string getPolicyName() const {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        return policy_->getName();
    }
    
    void update(const Position3D& robot_pos,
               std::chrono::steady_clock::time_point current_time) {
        // ... existing update logic ...
        
        // Build policy context
        PolicyContext context{
            robot_pos,
            current_chunk_,
            current_time,
            chunk_metadata_
        };
        
        // Determine desired chunks
        std::vector<ChunkCoord> desired_chunks;
        {
            std::lock_guard<std::mutex> lock(policy_mutex_);
            
            // Evaluate policy for all potential chunks
            // (current chunk + 26 neighbors for radius=1, more for larger radius)
            for (int dx = -max_radius_; dx <= max_radius_; ++dx) {
                for (int dy = -max_radius_; dy <= max_radius_; ++dy) {
                    for (int dz = -max_radius_; dz <= max_radius_; ++dz) {
                        ChunkCoord coord = current_chunk_ + ChunkCoord{dx, dy, dz};
                        
                        if (policy_->shouldLoad(coord, context)) {
                            desired_chunks.push_back(coord);
                        }
                    }
                }
            }
        }
        
        // Load missing chunks
        loadChunks(desired_chunks, context);
        
        // Evict unwanted chunks
        evictChunks(context);
    }
    
private:
    std::unique_ptr<ChunkLoadingPolicy> policy_;
    mutable std::mutex policy_mutex_;
    bool policy_changed_ = false;
    
    int max_radius_ = 3;  // Maximum radius to check (conservative estimate)
    
    std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash> chunk_metadata_;
    ChunkCoord current_chunk_;
    double chunk_size_;
};

} // namespace RollingBonxai
```

### Design Considerations

#### 1. Why No Composition in V2?

**Complexity:**
- Composition requires AND/OR logic, priority aggregation
- Need to handle contradictory policies
- More testing, more edge cases

**V2 Alternative:**
- Users choose the ONE policy that best fits
- Can create custom policy that combines logic internally
- Example: Custom policy does neighborhood + planar + temporal in one class

**V3 Plan:**
- Add `CompositePolicy` class
- Support AND/OR combination
- Priority aggregation strategies

#### 2. Policy Evaluation Scope

**Question:** Which chunks to evaluate?

**Option 1:** Evaluate only neighborhood (27 chunks for radius=1)
- Pros: Fast, bounded
- Cons: Can't handle policies with larger radius

**Option 2:** Evaluate all loaded chunks + neighborhood
- Pros: Handles eviction well
- Cons: Grows with loaded chunk count

**V2 Decision:** Evaluate within `max_radius` (configurable, default=3)
- Covers most use cases
- Bounded performance (7×7×7 = 343 chunks max)
- Can increase if needed

#### 3. Policy State

**Question:** Should policies be stateful?

**V2 Decision:** Stateless policies only
- All state in `PolicyContext` or `ChunkMetadata`
- Policies are pure functions (const methods)
- Thread-safe by design

**V3 Consideration:** Allow optional state
- Example: Learning policy tracks access patterns
- Requires synchronization

#### 4. Priority Usage

**When is priority used?**
- Multiple chunks need loading simultaneously
- Async I/O queue orders by priority
- Higher priority chunks load first

**Priority range:** [0.0, 1.0]
- 0.0 = lowest priority (load last)
- 1.0 = highest priority (load first)
- Relative ordering matters, not absolute values

#### 5. Custom Policy Example

```cpp
// User's custom policy: only load chunks on planned path
class PathBasedPolicy : public RollingBonxai::ChunkLoadingPolicy {
public:
    PathBasedPolicy(const std::vector<Position3D>& waypoints, double buffer)
        : waypoints_(waypoints), buffer_(buffer) {}
    
    bool shouldLoad(const ChunkCoord& coord,
                   const PolicyContext& context) const override {
        // Check if chunk intersects with path buffer
        for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
            if (chunkIntersectsSegment(coord, waypoints_[i], waypoints_[i+1])) {
                return true;
            }
        }
        return false;
    }
    
    bool shouldEvict(const ChunkCoord& coord,
                    const PolicyContext& context) const override {
        return !shouldLoad(coord, context);
    }
    
    float getPriority(const ChunkCoord& coord,
                     const PolicyContext& context) const override {
        // Priority based on distance along path
        // (implementation details...)
        return 0.5f;
    }
    
    std::string getName() const override { return "PathBasedPolicy"; }
    
private:
    std::vector<Position3D> waypoints_;
    double buffer_;
    
    bool chunkIntersectsSegment(const ChunkCoord& coord,
                               const Position3D& p1,
                               const Position3D& p2) const {
        // Geometry check...
        return false;  // placeholder
    }
};
```

### Edge Cases

#### Case 1: Policy Never Loads Anything

```cpp
class NeverLoadPolicy : public ChunkLoadingPolicy {
    bool shouldLoad(...) const override { return false; }
    // ...
};
```

**Behavior:**
- System would have no chunks loaded
- Robot can't operate

**Mitigation:**
- Validate on first update: if desired_chunks.empty(), log warning
- Consider always loading current chunk regardless of policy

#### Case 2: Policy Loads Everything

```cpp
class LoadAllPolicy : public ChunkLoadingPolicy {
    bool shouldLoad(...) const override { return true; }
    // ...
};
```

**Behavior:**
- Unbounded memory growth
- Performance degradation

**Mitigation:**
- Track loaded chunk count
- Warn if exceeds threshold (e.g., 1000 chunks)
- Up to user to fix policy

#### Case 3: Policy Change During Operation

```cpp
// Runtime policy swap
rolling_bonsai.setPolicy(std::make_unique<PlanarNeighborhoodPolicy>(1, 0, 2));
```

**Behavior:**
- Old policy may have loaded chunks new policy wants to evict
- Or vice versa

**Handling:**
- On next update, re-evaluate all loaded chunks
- Evict chunks no longer desired
- Load chunks newly desired
- Smooth transition over 1-2 update cycles

#### Case 4: Metadata Not Available

```cpp
const ChunkMetadata* meta = context.getMetadata(coord);
if (meta == nullptr) {
    // Chunk doesn't exist yet
}
```

**Handling:**
- Policies should handle nullptr gracefully
- Usually: treat new chunks as "should load"
- Temporal policy: no age restriction for new chunks

### Testing Strategy

#### Unit Tests

**Test 1: Neighborhood Policy Basic**
```cpp
TEST(NeighborhoodPolicy, BasicLoading) {
    NeighborhoodPolicy policy(1);  // radius=1
    
    ChunkCoord current(0, 0, 0);
    PolicyContext context{
        Position3D{0, 0, 0},
        current,
        std::chrono::steady_clock::now(),
        {}  // empty metadata
    };
    
    // Current chunk should load
    EXPECT_TRUE(policy.shouldLoad(current, context));
    
    // Face neighbor should load
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(1, 0, 0), context));
    
    // Edge neighbor should load
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(1, 1, 0), context));
    
    // Corner neighbor should load
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(1, 1, 1), context));
    
    // Outside neighborhood should NOT load
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(2, 0, 0), context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(0, 0, 5), context));
}
```

**Test 2: Planar Policy Vertical Restriction**
```cpp
TEST(PlanarNeighborhoodPolicy, VerticalRestriction) {
    PlanarNeighborhoodPolicy policy(1, 0, 2);  // z=[0,2]
    
    ChunkCoord current(0, 0, 1);  // At z=1
    PolicyContext context{
        Position3D{0, 0, 0},
        current,
        std::chrono::steady_clock::now(),
        {}
    };
    
    // Within neighborhood and z-range
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(1, 0, 1), context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(0, 0, 0), context));
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(0, 0, 2), context));
    
    // Within neighborhood but outside z-range
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(0, 0, 3), context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(0, 0, -1), context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(1, 1, 5), context));
}
```

**Test 3: Temporal Policy Age Filtering**
```cpp
TEST(TemporalNeighborhoodPolicy, AgeFiltering) {
    TemporalNeighborhoodPolicy policy(1, std::chrono::hours(1));
    
    auto now = std::chrono::steady_clock::now();
    auto old_time = now - std::chrono::hours(2);
    auto recent_time = now - std::chrono::minutes(30);
    
    std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash> metadata;
    metadata[ChunkCoord(1, 0, 0)] = ChunkMetadata{old_time, now, 0, false};
    metadata[ChunkCoord(0, 1, 0)] = ChunkMetadata{recent_time, now, 0, false};
    
    ChunkCoord current(0, 0, 0);
    PolicyContext context{
        Position3D{0, 0, 0},
        current,
        now,
        metadata
    };
    
    // Old chunk should NOT load
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(1, 0, 0), context));
    
    // Recent chunk should load
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(0, 1, 0), context));
    
    // New chunk (no metadata) should load
    EXPECT_TRUE(policy.shouldLoad(ChunkCoord(0, 0, 1), context));
}
```

**Test 4: Priority Calculation**
```cpp
TEST(NeighborhoodPolicy, PriorityOrdering) {
    NeighborhoodPolicy policy(2);  // radius=2
    
    ChunkCoord current(0, 0, 0);
    PolicyContext context{
        Position3D{0, 0, 0},
        current,
        std::chrono::steady_clock::now(),
        {}
    };
    
    // Closer chunks should have higher priority
    float priority_current = policy.getPriority(current, context);
    float priority_dist1 = policy.getPriority(ChunkCoord(1, 0, 0), context);
    float priority_dist2 = policy.getPriority(ChunkCoord(2, 0, 0), context);
    
    EXPECT_GT(priority_current, priority_dist1);
    EXPECT_GT(priority_dist1, priority_dist2);
    
    // All priorities in [0, 1]
    EXPECT_GE(priority_current, 0.0f);
    EXPECT_LE(priority_current, 1.0f);
}
```

**Test 5: Policy Swap**
```cpp
TEST(RollingBonsai, PolicySwap) {
    RollingBonsai rb(10.0, std::make_unique<NeighborhoodPolicy>(1));
    
    EXPECT_EQ(rb.getPolicyName(), "NeighborhoodPolicy(radius=1)");
    
    // Swap to planar policy
    rb.setPolicy(std::make_unique<PlanarNeighborhoodPolicy>(1, 0, 2));
    
    EXPECT_EQ(rb.getPolicyName(), "PlanarNeighborhoodPolicy(radius=1, z=[0,2])");
}
```

**Test 6: Custom Policy**
```cpp
// Test that custom policies work
class AlwaysLoadCurrentPolicy : public ChunkLoadingPolicy {
public:
    bool shouldLoad(const ChunkCoord& coord, 
                   const PolicyContext& context) const override {
        return coord == context.current_chunk;
    }
    
    bool shouldEvict(const ChunkCoord& coord,
                    const PolicyContext& context) const override {
        return coord != context.current_chunk;
    }
    
    float getPriority(const ChunkCoord& coord,
                     const PolicyContext& context) const override {
        return coord == context.current_chunk ? 1.0f : 0.0f;
    }
    
    std::string getName() const override { return "AlwaysLoadCurrentPolicy"; }
};

TEST(CustomPolicy, OnlyCurrentChunk) {
    AlwaysLoadCurrentPolicy policy;
    
    ChunkCoord current(5, 5, 5);
    PolicyContext context{
        Position3D{50, 50, 50},
        current,
        std::chrono::steady_clock::now(),
        {}
    };
    
    EXPECT_TRUE(policy.shouldLoad(current, context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(6, 5, 5), context));
    EXPECT_FALSE(policy.shouldLoad(ChunkCoord(5, 6, 5), context));
}
```

#### Performance Tests

**Benchmark 1: Policy Evaluation**
```cpp
BENCHMARK(PolicyEvaluation_Neighborhood) {
    NeighborhoodPolicy policy(1);
    
    ChunkCoord current(0, 0, 0);
    PolicyContext context{
        Position3D{0, 0, 0},
        current,
        std::chrono::steady_clock::now(),
        {}
    };
    
    // Evaluate 27 chunks (3×3×3 neighborhood)
    for (int i = 0; i < 100000; ++i) {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    ChunkCoord coord = current + ChunkCoord{dx, dy, dz};
                    bool should_load = policy.shouldLoad(coord, context);
                    benchmark::DoNotOptimize(should_load);
                }
            }
        }
    }
}
// Target: < 100μs per 27-chunk evaluation
```

### Usage Examples

**Example 1: Ground Robot (Warehouse)**
```cpp
#include "rolling_bonsai.hpp"

int main() {
    double chunk_size = 10.0;
    
    // Ground robot: only load 3 vertical layers, radius=1
    auto policy = std::make_unique<RollingBonxai::PlanarNeighborhoodPolicy>(
        1,    // radius
        0, 2  // z_min, z_max (ground level + 2 layers)
    );
    
    RollingBonxai::RollingBonsai rb(chunk_size, std::move(policy));
    
    // Robot operates...
    while (robot.isRunning()) {
        Position3D pos = robot.getPosition();
        auto now = std::chrono::steady_clock::now();
        
        rb.update(pos, now);
        
        // Use chunks for planning...
    }
    
    return 0;
}
```

**Example 2: Aerial Robot (Dynamic Environment)**
```cpp
int main() {
    double chunk_size = 20.0;
    
    // Aerial robot: full 3D, but ignore old data
    auto policy = std::make_unique<RollingBonxai::TemporalNeighborhoodPolicy>(
        1,                          // radius
        std::chrono::hours(1)       // max_age: 1 hour
    );
    
    RollingBonxai::RollingBonsai rb(chunk_size, std::move(policy));
    
    // Robot operates...
    
    return 0;
}
```

**Example 3: Custom Mission-Based Policy**
```cpp
class MissionPolicy : public RollingBonxai::ChunkLoadingPolicy {
public:
    MissionPolicy(const std::vector<Position3D>& waypoints)
        : waypoints_(waypoints) {}
    
    bool shouldLoad(const ChunkCoord& coord,
                   const PolicyContext& context) const override {
        // Load if within 2 chunks of any waypoint
        for (const auto& wp : waypoints_) {
            ChunkCoord wp_chunk = positionToChunk(wp);
            int dx = std::abs(coord.x - wp_chunk.x);
            int dy = std::abs(coord.y - wp_chunk.y);
            int dz = std::abs(coord.z - wp_chunk.z);
            
            if (std::max({dx, dy, dz}) <= 2) {
                return true;
            }
        }
        return false;
    }
    
    bool shouldEvict(const ChunkCoord& coord,
                    const PolicyContext& context) const override {
        return !shouldLoad(coord, context);
    }
    
    float getPriority(const ChunkCoord& coord,
                     const PolicyContext& context) const override {
        // Higher priority for waypoints closer in mission sequence
        return 0.5f;  // Simplified
    }
    
    std::string getName() const override { return "MissionPolicy"; }
    
private:
    std::vector<Position3D> waypoints_;
    
    ChunkCoord positionToChunk(const Position3D& pos) const {
        // Use chunk coordinate system...
        return ChunkCoord{0, 0, 0};  // placeholder
    }
};

int main() {
    std::vector<Position3D> mission_waypoints = {
        {10, 20, 0}, {50, 30, 0}, {100, 40, 0}
    };
    
    auto policy = std::make_unique<MissionPolicy>(mission_waypoints);
    
    RollingBonxai::RollingBonsai rb(10.0, std::move(policy));
    
    // Robot executes mission...
    
    return 0;
}
```

### Configuration Guidelines

**Choosing a Policy:**

| Robot Type | Environment | Recommended Policy | Parameters |
|------------|-------------|-------------------|------------|
| Ground Robot | Static indoor | `PlanarNeighborhoodPolicy` | radius=1, z=[0,2] |
| Ground Robot | Dynamic outdoor | `TemporalNeighborhoodPolicy` | radius=1, max_age=30min |
| Aerial Drone | Any | `NeighborhoodPolicy` | radius=1 |
| Fast Robot (>5 m/s) | Any | `NeighborhoodPolicy` | radius=2 or 3 |
| Mission-specific | Any | Custom policy | N/A |

**Radius Selection:**
```
radius = ceil(velocity * reaction_time / chunk_size)

Example:
  velocity = 5 m/s
  reaction_time = 2 seconds (time to detect and react)
  chunk_size = 10 m
  
  radius = ceil(5 * 2 / 10) = ceil(1.0) = 1
```

### Integration Considerations

#### With Center-Origin Chunks
- Policy uses `ChunkCoord` directly
- No dependency on coordinate system details

#### With Hysteresis
- Policy evaluated after hysteresis confirms transition
- Avoids policy evaluation thrashing

#### With Async I/O (Next Component)
- Policy provides priority values
- Async I/O uses priorities to order queue
- Higher priority chunks load first

### Documentation Requirements

**API Documentation:**
- Doxygen comments for all policy classes
- `PolicyContext` field explanations
- Custom policy implementation guide

**Design Documentation:**
- Policy selection decision tree
- Performance characteristics of each policy
- Custom policy examples

**User Guide:**
- How to choose a policy
- How to implement a custom policy
- Policy swap best practices

## Acceptance Criteria

- [ ] `ChunkLoadingPolicy` interface defines shouldLoad, shouldEvict, getPriority
- [ ] `NeighborhoodPolicy` correctly loads all chunks within radius
- [ ] `PlanarNeighborhoodPolicy` restricts vertical range correctly
- [ ] `TemporalNeighborhoodPolicy` filters old chunks correctly
- [ ] Policy evaluation for 27 chunks completes in < 100μs
- [ ] Users can set policy at initialization
- [ ] Users can swap policy at runtime
- [ ] Custom policies compile and work correctly
- [ ] All unit tests pass (6 tests minimum)
- [ ] Performance benchmark meets target
- [ ] Documentation complete: API docs + design doc + user guide

---

## Implementation Plan

### Phase 1: Core Interface (1 day)
- Implement `ChunkLoadingPolicy` interface
- Implement `PolicyContext` struct
- Integration hooks in `RollingBonsai`

### Phase 2: Built-in Policies (2 days)
- Implement `NeighborhoodPolicy`
- Implement `PlanarNeighborhoodPolicy`
- Implement `TemporalNeighborhoodPolicy`

### Phase 3: Policy Management (1 day)
- Policy swap mechanism
- Thread safety
- Validation and warnings

### Phase 4: Testing (2 days)
- Unit tests for each policy
- Custom policy test
- Performance benchmarks

### Phase 5: Documentation (1 day)
- API documentation
- Design document
- User guide with examples

**Total Estimated Time: 7 days**

---

## Risks and Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Policy never loads chunks | High | Low | Validate on first update, warn user |
| Policy loads too many chunks | Medium | Medium | Track count, warn if excessive |
| Custom policy performance issues | Medium | Medium | Document performance best practices |
| Policy swap race conditions | Low | Low | Mutex protection on policy access |
| Policy evaluation too slow | Medium | Low | Benchmark early, optimize if needed |

---

**Component Version:** 1.0  
**Component Status:** Ready for Implementation  
**Dependencies:** Center-Origin Chunks (complete), Hysteresis on Transitions (complete)  
**Estimated Effort:** 7 person-days
