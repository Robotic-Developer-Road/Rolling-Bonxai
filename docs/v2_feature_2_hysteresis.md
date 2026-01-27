# Rolling Bonsai V2: Component PRD - Hysteresis on Transitions

## Implementation Tracking

### Components Selected for V2:
1. **Center-Origin Chunks** ✓ Complete
2. **Hysteresis on Transitions** ← Current

### Components Not Yet Selected:
- Predictive Pre-Loading
- Policy Composition
- Async I/O
- LRU Caching
- Lazy Saves with Dirty Tracking

---

## Overview

Hysteresis on Transitions implements a "dead zone" around chunk boundaries to prevent rapid chunk transitions caused by localization noise. This component ensures stable chunk assignment even when the robot's estimated position oscillates near boundaries.

## Motivation

**Problem Without Hysteresis:**

When a robot operates near a chunk boundary:
```
Chunk Boundary at x = 5.0m
Robot actual position: 4.98m (inside chunk A)
Localization noise: ±0.05m

Time 0ms: Position estimate = 4.97m → Chunk A
Time 10ms: Position estimate = 5.02m → Chunk B (transition triggered!)
Time 20ms: Position estimate = 4.96m → Chunk A (transition back!)
Time 30ms: Position estimate = 5.03m → Chunk B (another transition!)
```

Each transition potentially triggers:
- Loading new chunks (I/O operations)
- Evicting old chunks (I/O operations)
- Policy re-evaluation
- Cache reorganization

**Result:** Thrashing, excessive I/O, unstable system behavior

**Solution with Hysteresis:**

Add a threshold distance that must be exceeded before accepting a transition:
```
Chunk Boundary at x = 5.0m
Hysteresis threshold: 2.0m (20% of 10m chunk)

Time 0ms: In Chunk A, position = 4.97m
Time 10ms: Cross boundary to 5.02m → within hysteresis, stay in Chunk A
Time 20ms: Position = 4.96m → still in Chunk A
Time 30ms: Position = 5.03m → within hysteresis, stay in Chunk A
Time 40ms: Position = 7.5m → exceeded hysteresis (> 2m into Chunk B) → transition to B
```

Only genuine movement across chunks triggers transitions.

## Requirements

### Functional Requirements

**FR-2.1: Hysteresis Threshold Configuration**
- The system shall support a configurable hysteresis threshold (default: 20% of chunk_size)
- Threshold shall be specified as a fraction of chunk_size (e.g., 0.2 = 20%)
- Threshold must be in range [0.0, 0.5] (cannot exceed half chunk size)
- Threshold value of 0.0 disables hysteresis (immediate transitions)

**FR-2.2: Transition State Machine**
- The system shall maintain the current chunk assignment
- Chunk transition shall only occur when robot position exceeds hysteresis threshold into new chunk
- Once transitioned, system shall not revert unless robot crosses back beyond hysteresis threshold

**FR-2.3: Boundary Distance Calculation**
- The system shall calculate how far the robot has moved into a neighboring chunk
- Distance measurement shall be perpendicular to the boundary (not Euclidean to chunk center)
- Positive distance = inside new chunk, negative distance = still in current chunk

**FR-2.4: Multi-Axis Hysteresis**
- For face transitions (1 axis changes): Apply hysteresis on that axis
- For edge transitions (2 axes change): Apply hysteresis on both axes (AND condition)
- For corner transitions (3 axes change): Apply hysteresis on all three axes (AND condition)

**FR-2.5: Initialization Behavior**
- On first position update, immediately assign chunk without hysteresis
- System assumes initialization position is valid (no previous chunk to compare against)

**FR-2.6: Stationary Detection (Optional)**
- If robot velocity < threshold for N seconds, may disable hysteresis temporarily
- Allows fine-tuning chunk assignment for stationary robots
- Re-enable when motion detected

### Non-Functional Requirements

**NFR-2.1: Transition Check Performance**
- Hysteresis checking shall add < 1μs overhead per update cycle
- No heap allocations during transition checks
- Suitable for 10-100 Hz update rates

**NFR-2.2: Transition Frequency**
- With properly tuned hysteresis, transitions shall occur < 1 per second for typical robot motion
- For stationary robot (v < 0.1 m/s), transitions shall be zero

**NFR-2.3: Latency**
- Legitimate transitions (robot genuinely moving to new chunk) shall be detected within one update cycle
- No artificial delays beyond hysteresis threshold

**NFR-2.4: Memory**
- State tracking shall require < 100 bytes total
- Only store: current_chunk, previous_position (optional)

## Design

### Core Components

#### 1. Transition State Tracker

```cpp
namespace RollingBonxai {

class TransitionManager {
public:
    TransitionManager(double chunk_size, double hysteresis_ratio = 0.2);
    
    // Main update: check if transition should occur
    // Returns true if chunk changed
    bool update(const Position3D& robot_pos, ChunkCoord& current_chunk);
    
    // Configuration
    void setHysteresisRatio(double ratio);
    double getHysteresisRatio() const { return hysteresis_ratio_; }
    double getHysteresisDistance() const { return hysteresis_distance_; }
    
    // Query state
    ChunkCoord getCurrentChunk() const { return current_chunk_; }
    bool isInitialized() const { return initialized_; }
    
    // Distance into neighboring chunk (for debugging/telemetry)
    double getDistanceIntoNeighbor(const Position3D& robot_pos, 
                                   const ChunkCoord& neighbor) const;
    
private:
    double chunk_size_;
    double hysteresis_ratio_;
    double hysteresis_distance_;  // Cached: hysteresis_ratio * chunk_size
    
    ChunkCoord current_chunk_;
    bool initialized_;
    
    ChunkCoordinateSystem coord_system_;
    
    // Check if position exceeds hysteresis threshold into new chunk
    bool shouldTransition(const Position3D& pos, 
                         const ChunkCoord& from_chunk,
                         const ChunkCoord& to_chunk) const;
};

} // namespace RollingBonxai
```

#### 2. Boundary Distance Calculation

```cpp
namespace RollingBonxai {

// Calculate perpendicular distance from position to chunk boundary
// Positive = inside target chunk, Negative = still in current chunk
double calculateBoundaryPenetration(const Position3D& pos,
                                   const ChunkCoord& from_chunk,
                                   const ChunkCoord& to_chunk,
                                   double chunk_size);

// For each axis, calculate how far position has crossed the boundary
struct AxisPenetration {
    double x_penetration;  // Negative if not crossed
    double y_penetration;
    double z_penetration;
};

AxisPenetration calculateAxisPenetrations(const Position3D& pos,
                                         const ChunkCoord& from_chunk,
                                         const ChunkCoord& to_chunk,
                                         double chunk_size);

} // namespace RollingBonxai
```

### Implementation Strategy

#### Algorithm: Hysteresis Transition Check

```
Input: robot_pos, current_chunk
Output: new_chunk (may equal current_chunk if no transition)

1. Calculate naive chunk assignment: naive_chunk = positionToChunk(robot_pos)

2. If not initialized:
     current_chunk = naive_chunk
     initialized = true
     return current_chunk (no transition)

3. If naive_chunk == current_chunk:
     return current_chunk (still in same chunk, no check needed)

4. Calculate distance into naive_chunk:
     penetration = calculateBoundaryPenetration(robot_pos, current_chunk, naive_chunk)

5. If penetration > hysteresis_distance:
     current_chunk = naive_chunk (transition!)
     return current_chunk

6. Else:
     return current_chunk (within hysteresis zone, stay in current chunk)
```

#### Boundary Penetration Calculation

**For Face Transition (1 axis changes):**

```
Example: current_chunk = (0,0,0), naive_chunk = (1,0,0)
Boundary at x = 5.0m (halfway between chunks)

If robot at x = 6.2m:
  penetration = 6.2 - 5.0 = 1.2m into chunk (1,0,0)
  
If robot at x = 4.8m:
  penetration = 4.8 - 5.0 = -0.2m (still in chunk (0,0,0))
```

**Implementation:**
```cpp
double calculateBoundaryPenetration(const Position3D& pos,
                                   const ChunkCoord& from_chunk,
                                   const ChunkCoord& to_chunk,
                                   double chunk_size) {
    // Determine which axis changed
    int dx = to_chunk.x - from_chunk.x;
    int dy = to_chunk.y - from_chunk.y;
    int dz = to_chunk.z - from_chunk.z;
    
    // Face transition: exactly one axis changes by ±1
    if (std::abs(dx) + std::abs(dy) + std::abs(dz) == 1) {
        double boundary_pos, robot_coord;
        int direction;
        
        if (dx != 0) {
            boundary_pos = from_chunk.x * chunk_size + (dx > 0 ? 0.5 : -0.5) * chunk_size;
            robot_coord = pos.x;
            direction = dx;
        } else if (dy != 0) {
            boundary_pos = from_chunk.y * chunk_size + (dy > 0 ? 0.5 : -0.5) * chunk_size;
            robot_coord = pos.y;
            direction = dy;
        } else { // dz != 0
            boundary_pos = from_chunk.z * chunk_size + (dz > 0 ? 0.5 : -0.5) * chunk_size;
            robot_coord = pos.z;
            direction = dz;
        }
        
        // Positive = moved into new chunk
        return direction * (robot_coord - boundary_pos);
    }
    
    // Edge transition: two axes change
    if (std::abs(dx) + std::abs(dy) + std::abs(dz) == 2) {
        // Calculate penetration for each axis, return minimum
        AxisPenetration pens = calculateAxisPenetrations(pos, from_chunk, to_chunk, chunk_size);
        
        double min_penetration = std::numeric_limits<double>::max();
        if (dx != 0) min_penetration = std::min(min_penetration, pens.x_penetration);
        if (dy != 0) min_penetration = std::min(min_penetration, pens.y_penetration);
        if (dz != 0) min_penetration = std::min(min_penetration, pens.z_penetration);
        
        return min_penetration;
    }
    
    // Corner transition: three axes change
    if (std::abs(dx) + std::abs(dy) + std::abs(dz) == 3) {
        AxisPenetration pens = calculateAxisPenetrations(pos, from_chunk, to_chunk, chunk_size);
        return std::min({pens.x_penetration, pens.y_penetration, pens.z_penetration});
    }
    
    // Should not reach here (non-adjacent chunks)
    return -std::numeric_limits<double>::max();
}
```

**For Edge/Corner Transitions:**

Must exceed hysteresis on **all** changing axes:
```
Example: current = (0,0,0), naive = (1,1,0) (edge transition)
Hysteresis = 2.0m

Robot at (6.5, 6.8, 0):
  x_penetration = 1.5m (6.5 - 5.0)
  y_penetration = 1.8m (6.8 - 5.0)
  min_penetration = 1.5m < 2.0m → Stay in (0,0,0)

Robot at (7.5, 7.8, 0):
  x_penetration = 2.5m
  y_penetration = 2.8m
  min_penetration = 2.5m > 2.0m → Transition to (1,1,0)
```

### Design Considerations

#### 1. Hysteresis Threshold Tuning

**Factors affecting optimal threshold:**
- Localization noise magnitude (σ)
- Robot velocity
- Update frequency
- Chunk size

**Guidelines:**
```
Hysteresis should be > 3σ of localization noise

Example:
  Localization noise: σ = 0.05m
  Minimum hysteresis: 3 × 0.05 = 0.15m
  
For chunk_size = 10m:
  Minimum ratio: 0.15 / 10 = 0.015 (1.5%)
  Recommended: 0.1 - 0.3 (10-30%)
  Default: 0.2 (20%)
```

**Too small (< 5%):**
- Doesn't prevent noise-induced thrashing
- Frequent unnecessary transitions

**Too large (> 40%):**
- Delayed reaction to genuine movement
- Robot may be far into new chunk before transition
- Could cause data staleness issues

**Recommended range: 15-25%**

#### 2. Asymmetric Hysteresis (Future Enhancement)

Current design: symmetric hysteresis (same threshold in all directions)

**Potential enhancement:**
```cpp
struct HysteresisConfig {
    double enter_threshold;  // Distance to enter new chunk
    double exit_threshold;   // Distance to revert to old chunk
};

// Example: Harder to revert than to enter
enter_threshold = 0.15 * chunk_size  // 15% to enter
exit_threshold = 0.30 * chunk_size   // 30% to revert
```

**Use case:** Prevent oscillation when robot hovers exactly at threshold

**For V2:** Keep symmetric for simplicity

#### 3. Per-Axis Hysteresis (Future Enhancement)

Current design: uniform hysteresis on all axes

**Potential enhancement:**
```cpp
struct AxisHysteresis {
    double x_ratio;
    double y_ratio;
    double z_ratio;
};

// Example: Ground robot with different noise characteristics
x_ratio = 0.2  // 20% horizontal
y_ratio = 0.2  // 20% horizontal
z_ratio = 0.5  // 50% vertical (altitude more noisy)
```

**For V2:** Keep uniform for simplicity

#### 4. Velocity-Dependent Hysteresis (Future Enhancement)

**Idea:** Reduce hysteresis for fast-moving robots
```
If velocity > high_speed_threshold:
    hysteresis = base_hysteresis * 0.5
    // React faster to transitions
Else:
    hysteresis = base_hysteresis
```

**Rationale:** Fast-moving robot needs quicker chunk transitions

**For V2:** Keep static hysteresis

#### 5. Integration with Predictive Preloading

**Important:** Hysteresis and preloading work at different boundaries:

```
|-------- Current Chunk --------|-------- Next Chunk --------|
                          ^                ^           ^
                          |                |           |
                    Preload          Hysteresis    Firm in next
                    Boundary         Boundary      chunk
```

- **Preload boundary:** Trigger loading (e.g., 20% from edge)
- **Chunk boundary:** Actual geometric boundary
- **Hysteresis boundary:** Confirm transition (e.g., 20% into next chunk)

Timeline:
1. Robot approaches boundary
2. At preload distance: Start loading next chunk's neighbors (async)
3. Cross geometric boundary: No transition yet (within hysteresis)
4. Exceed hysteresis: Transition confirmed, chunks already loaded!

### Edge Cases

#### Case 1: Teleportation

**Scenario:** Robot position jumps discontinuously (e.g., relocalization)
```
Time T: position = (0, 0, 0), chunk = (0,0,0)
Time T+1: position = (50, 0, 0), chunk = ???
```

**Behavior:**
- Naive chunk = (5, 0, 0) (assuming 10m chunks)
- Distance from (0,0,0) to (5,0,0) = 50m
- Far exceeds hysteresis → Immediate transition

**Correct:** Large jumps bypass hysteresis (as intended)

#### Case 2: Very Slow Movement

**Scenario:** Robot creeps across boundary at 0.01 m/s
```
Hysteresis = 2.0m
Time to cross hysteresis = 2.0 / 0.01 = 200 seconds
```

**Behavior:**
- Remains in old chunk for 200 seconds
- Eventually crosses hysteresis and transitions

**Is this a problem?**
- Robot is geometrically in new chunk but system thinks it's in old chunk
- Depends on application: Is position data from old chunk still valid?

**Mitigation:**
- Ensure working set includes neighbors (predictive preloading handles this)
- Robot can access both old and new chunk data during transition

#### Case 3: Multi-Chunk Jump

**Scenario:** Robot moves multiple chunks at once
```
Current chunk = (0,0,0)
New position → naive chunk = (3,0,0)
```

**Behavior:**
- Not an adjacent chunk (distance > 1 in any axis)
- `calculateBoundaryPenetration` returns -infinity
- But naive chunk ≠ current chunk
- Should transition immediately

**Fix in algorithm:**
```cpp
bool shouldTransition(const Position3D& pos, 
                     const ChunkCoord& from_chunk,
                     const ChunkCoord& to_chunk) const {
    // Check if adjacent
    int dx = std::abs(to_chunk.x - from_chunk.x);
    int dy = std::abs(to_chunk.y - from_chunk.y);
    int dz = std::abs(to_chunk.z - from_chunk.z);
    
    // Non-adjacent: immediate transition (teleport/large jump)
    if (dx > 1 || dy > 1 || dz > 1) {
        return true;
    }
    
    // Adjacent: apply hysteresis
    double penetration = calculateBoundaryPenetration(pos, from_chunk, to_chunk, chunk_size_);
    return penetration > hysteresis_distance_;
}
```

#### Case 4: Initialization Near Boundary

**Scenario:** Robot initializes at position (4.9, 0, 0) with chunk_size = 10m
```
Chunk boundary at x = 5.0
Robot position: 4.9 (only 0.1m from boundary)
```

**Behavior:**
- First update: assign chunk = (0,0,0) without hysteresis
- If localization noise pushes to 5.05m: naive chunk = (1,0,0)
- Penetration = 0.05m < 2.0m hysteresis → Stay in (0,0,0)

**Correct:** Hysteresis prevents immediate thrashing even during initialization

### Testing Strategy

#### Unit Tests

**Test 1: Basic Transition with Hysteresis**
```cpp
TEST(TransitionManager, BasicTransitionWithHysteresis) {
    TransitionManager tm(10.0, 0.2);  // 10m chunks, 20% hysteresis = 2m
    ChunkCoord chunk;
    
    // Initialize in chunk (0,0,0)
    bool changed = tm.update(Position3D(0, 0, 0), chunk);
    EXPECT_TRUE(changed);  // First update always transitions
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Move to boundary
    changed = tm.update(Position3D(5.0, 0, 0), chunk);
    EXPECT_FALSE(changed);  // On boundary, but within hysteresis
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Move 1m into next chunk (within hysteresis)
    changed = tm.update(Position3D(6.0, 0, 0), chunk);
    EXPECT_FALSE(changed);  // Within 2m hysteresis
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Move 2.5m into next chunk (exceeds hysteresis)
    changed = tm.update(Position3D(7.5, 0, 0), chunk);
    EXPECT_TRUE(changed);  // Exceeds hysteresis, transition!
    EXPECT_EQ(chunk, ChunkCoord(1, 0, 0));
}
```

**Test 2: Oscillation Prevention**
```cpp
TEST(TransitionManager, OscillationPrevention) {
    TransitionManager tm(10.0, 0.2);
    ChunkCoord chunk;
    
    // Initialize
    tm.update(Position3D(0, 0, 0), chunk);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Simulate noisy position oscillating around boundary
    std::vector<Position3D> noisy_positions = {
        {4.98, 0, 0},  // -0.02m from boundary
        {5.03, 0, 0},  // +0.03m into next chunk
        {4.96, 0, 0},  // -0.04m from boundary
        {5.05, 0, 0},  // +0.05m into next chunk
        {4.97, 0, 0},  // -0.03m from boundary
    };
    
    int transition_count = 0;
    for (const auto& pos : noisy_positions) {
        bool changed = tm.update(pos, chunk);
        if (changed) transition_count++;
    }
    
    // Should have ZERO transitions (all within hysteresis)
    EXPECT_EQ(transition_count, 0);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
}
```

**Test 3: Edge Transition Hysteresis**
```cpp
TEST(TransitionManager, EdgeTransitionHysteresis) {
    TransitionManager tm(10.0, 0.2);  // 2m hysteresis
    ChunkCoord chunk;
    
    tm.update(Position3D(0, 0, 0), chunk);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Move diagonally toward (1,1,0) edge
    // Position (6.0, 6.0, 0):
    //   x_penetration = 1.0m, y_penetration = 1.0m
    //   min = 1.0m < 2.0m → no transition
    bool changed = tm.update(Position3D(6.0, 6.0, 0), chunk);
    EXPECT_FALSE(changed);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Position (7.5, 7.5, 0):
    //   x_penetration = 2.5m, y_penetration = 2.5m
    //   min = 2.5m > 2.0m → transition!
    changed = tm.update(Position3D(7.5, 7.5, 0), chunk);
    EXPECT_TRUE(changed);
    EXPECT_EQ(chunk, ChunkCoord(1, 1, 0));
}
```

**Test 4: Teleportation (Large Jump)**
```cpp
TEST(TransitionManager, LargeJumpBypassesHysteresis) {
    TransitionManager tm(10.0, 0.2);
    ChunkCoord chunk;
    
    tm.update(Position3D(0, 0, 0), chunk);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Teleport to distant location
    bool changed = tm.update(Position3D(50, 0, 0), chunk);
    EXPECT_TRUE(changed);  // Should transition immediately
    EXPECT_EQ(chunk, ChunkCoord(5, 0, 0));
}
```

**Test 5: Hysteresis Ratio Configuration**
```cpp
TEST(TransitionManager, HysteresisRatioConfiguration) {
    TransitionManager tm(10.0, 0.3);  // 30% = 3m hysteresis
    ChunkCoord chunk;
    
    tm.update(Position3D(0, 0, 0), chunk);
    
    // 2m into next chunk: within 3m hysteresis
    bool changed = tm.update(Position3D(7.0, 0, 0), chunk);
    EXPECT_FALSE(changed);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // 3.5m into next chunk: exceeds 3m hysteresis
    changed = tm.update(Position3D(8.5, 0, 0), chunk);
    EXPECT_TRUE(changed);
    EXPECT_EQ(chunk, ChunkCoord(1, 0, 0));
}
```

**Test 6: Zero Hysteresis (Disabled)**
```cpp
TEST(TransitionManager, ZeroHysteresisDisablesFeature) {
    TransitionManager tm(10.0, 0.0);  // 0% = no hysteresis
    ChunkCoord chunk;
    
    tm.update(Position3D(4.9, 0, 0), chunk);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Cross boundary by 0.1m: should transition immediately
    bool changed = tm.update(Position3D(5.1, 0, 0), chunk);
    EXPECT_TRUE(changed);
    EXPECT_EQ(chunk, ChunkCoord(1, 0, 0));
}
```

**Test 7: Boundary Penetration Calculation**
```cpp
TEST(TransitionManager, BoundaryPenetrationCalculation) {
    TransitionManager tm(10.0, 0.2);
    
    ChunkCoord from(0, 0, 0);
    ChunkCoord to(1, 0, 0);
    
    // Position in old chunk
    double pen1 = tm.getDistanceIntoNeighbor(Position3D(4.5, 0, 0), to);
    EXPECT_DOUBLE_EQ(pen1, -0.5);  // -0.5m (still in old chunk)
    
    // Position on boundary
    double pen2 = tm.getDistanceIntoNeighbor(Position3D(5.0, 0, 0), to);
    EXPECT_DOUBLE_EQ(pen2, 0.0);
    
    // Position in new chunk
    double pen3 = tm.getDistanceIntoNeighbor(Position3D(6.5, 0, 0), to);
    EXPECT_DOUBLE_EQ(pen3, 1.5);  // 1.5m into new chunk
}
```

**Test 8: Corner Transition**
```cpp
TEST(TransitionManager, CornerTransition) {
    TransitionManager tm(10.0, 0.2);  // 2m hysteresis
    ChunkCoord chunk;
    
    tm.update(Position3D(0, 0, 0), chunk);
    
    // Move toward corner (1,1,1)
    // Position (6.5, 6.5, 6.5):
    //   All penetrations = 1.5m < 2.0m → no transition
    bool changed = tm.update(Position3D(6.5, 6.5, 6.5), chunk);
    EXPECT_FALSE(changed);
    
    // Position (7.5, 7.5, 7.5):
    //   All penetrations = 2.5m > 2.0m → transition!
    changed = tm.update(Position3D(7.5, 7.5, 7.5), chunk);
    EXPECT_TRUE(changed);
    EXPECT_EQ(chunk, ChunkCoord(1, 1, 1));
}
```

#### Integration Tests

**Test 9: With Center-Origin Chunks**
```cpp
TEST(Integration, HysteresisWithCenterOriginChunks) {
    ChunkCoordinateSystem coord_sys(10.0);
    TransitionManager tm(10.0, 0.2);
    ChunkCoord chunk;
    
    // Robot spawns at origin (center of chunk)
    Position3D spawn(0, 0, 0);
    tm.update(spawn, chunk);
    EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    
    // Add noise to spawn position (±10cm)
    std::vector<Position3D> noisy_spawn = {
        {0.08, 0.05, -0.03},
        {-0.06, 0.09, 0.02},
        {0.04, -0.07, 0.08},
    };
    
    for (const auto& pos : noisy_spawn) {
        bool changed = tm.update(pos, chunk);
        EXPECT_FALSE(changed);  // All within same chunk due to center-origin
        EXPECT_EQ(chunk, ChunkCoord(0, 0, 0));
    }
}
```

#### Performance Tests

**Benchmark 1: Transition Check Performance**
```cpp
BENCHMARK(TransitionCheckPerformance) {
    TransitionManager tm(10.0, 0.2);
    ChunkCoord chunk;
    
    tm.update(Position3D(0, 0, 0), chunk);
    
    for (int i = 0; i < 1000000; ++i) {
        Position3D pos(i * 0.001, 0, 0);  // Slowly moving
        benchmark::DoNotOptimize(tm.update(pos, chunk));
    }
}
// Target: < 1μs per update
```

**Benchmark 2: Boundary Penetration Calculation**
```cpp
BENCHMARK(BoundaryPenetrationCalculation) {
    TransitionManager tm(10.0, 0.2);
    ChunkCoord to(1, 0, 0);
    
    for (int i = 0; i < 1000000; ++i) {
        Position3D pos(i * 0.001, 0, 0);
        benchmark::DoNotOptimize(tm.getDistanceIntoNeighbor(pos, to));
    }
}
// Target: < 100ns per calculation
```

### Usage Example

```cpp
#include "transition_manager.hpp"
#include "chunk_coordinate_system.hpp"

int main() {
    // Initialize systems
    double chunk_size = 10.0;  // 10m chunks
    ChunkCoordinateSystem coord_sys(chunk_size);
    TransitionManager transition_mgr(chunk_size, 0.2);  // 20% hysteresis
    
    ChunkCoord current_chunk;
    
    // Main robot loop
    while (robot.isRunning()) {
        // Get robot state
        Position3D robot_pos = robot.getPosition();
        
        // Check for chunk transition
        bool chunk_changed = transition_mgr.update(robot_pos, current_chunk);
        
        if (chunk_changed) {
            std::cout << "Transitioned to chunk: " 
                      << current_chunk.x << ", " 
                      << current_chunk.y << ", " 
                      << current_chunk.z << std::endl;
            
            // Trigger chunk loading/eviction
            chunkManager.onChunkTransition(current_chunk);
        }
        
        // Continue with robot tasks...
        robot.doPathPlanning(current_chunk);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}
```

### Configuration Guidelines

**Recommended Hysteresis Ratios by Use Case:**

| Robot Type | Speed | Localization σ | Chunk Size | Recommended Ratio |
|------------|-------|----------------|------------|-------------------|
| Ground Robot | Slow (0.5 m/s) | 0.03m | 10m | 15% (1.5m) |
| Ground Robot | Medium (2 m/s) | 0.05m | 10m | 20% (2.0m) |
| Ground Robot | Fast (5 m/s) | 0.10m | 10m | 25% (2.5m) |
| Aerial Robot | Medium (3 m/s) | 0.10m | 20m | 15% (3.0m) |
| Aerial Robot | Fast (10 m/s) | 0.20m | 20m | 20% (4.0m) |

**Formula:**
```
hysteresis_ratio = max(0.15, 5 × σ / chunk_size)
```

Where σ is the standard deviation of localization noise.

### Integration Considerations

#### With Center-Origin Chunks
- Hysteresis works seamlessly with center-origin coordinate system
- No special handling needed
- Both components independent but complementary

#### With Predictive Preloading (Future)
- Hysteresis boundary ≠ Preload boundary
- Preloading can start before hysteresis is exceeded
- Timeline: Preload → Cross boundary → Exceed hysteresis → Confirm transition

#### With Policy System (Future)
- Policy evaluation happens after transition confirmation
- Don't re-evaluate policies during hysteresis phase (wasteful)
- Only when `chunk_changed == true`

### Documentation Requirements

**API Documentation:**
- Doxygen comments for TransitionManager class
- Explanation of hysteresis concept
- Configuration guidelines
- Performance characteristics

**Design Documentation:**
- State machine diagram
- Boundary penetration calculation algorithm
- Edge case handling
- Integration with other components

**Tuning Guide:**
- How to measure localization noise
- How to calculate optimal hysteresis
- What happens if too small/large
- Robot-specific recommendations

## Acceptance Criteria

- [ ] `TransitionManager` correctly prevents transitions within hysteresis threshold
- [ ] Oscillating position (±5cm around boundary) causes zero transitions
- [ ] Legitimate movement exceeding hysteresis triggers exactly one transition
- [ ] Edge and corner transitions apply hysteresis correctly (AND logic)
- [ ] Large jumps (non-adjacent chunks) bypass hysteresis
- [ ] First position update initializes chunk without hysteresis
- [ ] Hysteresis ratio configurable in range [0.0, 0.5]
- [ ] Ratio of 0.0 disables hysteresis (immediate transitions)
- [ ] Transition check completes in < 1μs (99th percentile)
- [ ] All unit tests pass (8 tests minimum)
- [ ] Integration test with center-origin chunks passes
- [ ] Documentation complete: API docs + design doc + tuning guide
- [ ] Performance benchmarks show < 1μs update latency

---

## Implementation Plan

### Phase 1: Core Algorithm (2 days)
- Implement `TransitionManager` class
- Implement `shouldTransition()` logic
- Implement boundary penetration calculation
- Handle face/edge/corner transitions

### Phase 2: State Management (1 day)
- Track current chunk
- Handle initialization
- Configuration management (hysteresis ratio)

### Phase 3: Testing (2 days)
- Unit tests (8 tests)
- Edge case tests (teleportation, multi-chunk jumps)
- Integration test with center-origin chunks
- Performance benchmarks

### Phase 4: Documentation (1 day)
- API documentation
- Design document
- Tuning guide
- Usage examples

**Total Estimated Time: 6 days**

---

## Risks and Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Hysteresis too conservative | Medium | Medium | Provide tuning guide, make ratio configurable |
| Slow robots stuck in wrong chunk | Low | Low | Document expected behavior, ensure working set includes neighbors |
| Performance overhead | Low | Low | Benchmark early, optimize hot paths |
| Edge/corner transition logic bugs | Medium | Medium | Comprehensive unit tests for all transition types |
| Integration complexity with preloading | Medium | Low | Clear separation of concerns, well-defined interfaces |

---

**Component Version:** 1.0  
**Component Status:** Ready for Implementation  
**Dependencies:** Center-Origin Chunks (complete)  
**Estimated Effort:** 6 person-days
