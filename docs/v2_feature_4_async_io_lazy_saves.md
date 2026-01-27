# Rolling Bonsai V2: Component PRD - Async I/O with Lazy Saves

## Implementation Tracking

### Components Selected for V2:
1. **Center-Origin Chunks** ✓ Complete
2. **Hysteresis on Transitions** ✓ Complete
3. **Chunk Loading Policy** ✓ Complete
4. **Async I/O with Lazy Saves** ← Current

### Components Slated for V3:
- Predictive Pre-Loading
- Policy Composition
- LRU Caching

---

## Overview

Async I/O with Lazy Saves provides non-blocking chunk loading and saving operations using a thread pool. The system ensures the robot's main control loop never blocks on disk I/O while maintaining data persistence. Lazy saves with dirty tracking minimize write I/O by only saving chunks that have been modified.

## Motivation

**Problem: Blocking I/O Kills Real-Time Performance**

Synchronous I/O blocks the robot's control loop:
```
Time 0ms: Trigger load chunk (1,0,0)
Time 0ms: READ from disk (blocking!)
  - Main loop frozen
  - No sensor processing
  - No control updates
  - No planning
Time 200ms: I/O completes
Time 200ms: Resume main loop

Robot missed 20 control cycles (at 100 Hz)!
```

**Problem: Unnecessary Writes Waste Resources**

Without dirty tracking:
```
Load chunk from disk (pre-built map)
Never modify chunk (read-only)
Evict chunk → WRITE to disk (unnecessary!)

Result:
- Wasted disk bandwidth
- Wasted SSD write cycles
- Wasted battery power
```

**Solution: Async I/O + Dirty Tracking**

```
Async I/O:
  Time 0ms: Queue load request → Return immediately
  Time 0-200ms: Worker thread handles I/O (main loop continues!)
  Time 200ms: Chunk ready, available on next update

Lazy Saves:
  Chunk loaded (dirty=false)
  Chunk never modified → dirty=false
  Evict chunk → Check dirty flag
    If dirty=false → Simply delete from memory (no I/O!)
    If dirty=true → Save to disk
```

**Benefits:**
- Main loop never blocks on I/O
- Real-time performance maintained
- 80-90% reduction in write I/O for read-heavy workloads
- Longer SSD lifespan
- Lower power consumption

## Requirements

### Functional Requirements

**FR-5.1: Thread Pool for I/O**
- The system shall use a thread pool for asynchronous I/O operations
- Thread pool size shall be configurable (default: 4 threads)
- Thread pool shall handle both load and save operations
- Workers shall process requests from priority queues

**FR-5.2: Async Load Operations**
- `loadAsync(coord, priority)` shall return immediately (non-blocking)
- Load operations shall return a future/callback when complete
- Multiple concurrent loads shall be supported
- Load requests shall be prioritized (higher priority loads first)

**FR-5.3: Async Save Operations**
- `saveAsync(coord, data)` shall return immediately (non-blocking)
- Save operations shall be lower priority than loads
- Saves shall be queued and processed in background
- Save completion shall be trackable (for graceful shutdown)

**FR-5.4: Dirty Tracking**
- Each chunk shall have a dirty flag (boolean)
- Dirty flag shall be false on load from disk
- Dirty flag shall be true on creation of new chunk
- Any modification to chunk shall set dirty flag to true
- On eviction, clean chunks (dirty=false) shall not be saved

**FR-5.5: Priority Queue**
- Load operations shall be ordered by priority (float value)
- Higher priority = load first
- Priority provided by loading policy
- FIFO ordering for same-priority requests

**FR-5.6: Duplicate Load Prevention**
- The system shall track pending load requests
- Duplicate loads for same chunk shall reuse existing request
- Shall not queue multiple loads for same chunk

**FR-5.7: Graceful Shutdown**
- On shutdown, all pending saves shall complete
- System shall wait for save queue to drain
- Timeout mechanism for hung operations

**FR-5.8: Error Handling**
- Load failures shall be reported (chunk not available)
- Save failures shall be logged and retried (configurable retry count)
- Storage full errors shall be handled gracefully

### Non-Functional Requirements

**NFR-5.1: Load Request Latency**
- `loadAsync()` call shall complete in < 10μs
- No blocking operations in main thread
- Immediate return with future/callback

**NFR-5.2: I/O Throughput**
- Thread pool shall achieve > 80% of disk sequential read bandwidth
- Worker threads shall minimize idle time
- Queue management overhead < 5% of total I/O time

**NFR-5.3: Memory Overhead**
- Pending load tracking shall use < 1KB per pending request
- Thread pool overhead shall be < 100KB total
- No memory leaks over extended operation

**NFR-5.4: Write I/O Reduction**
- Dirty tracking shall reduce write I/O by > 80% for read-heavy workloads
- Clean chunks shall never be written to disk

**NFR-5.5: Thread Safety**
- All operations shall be thread-safe
- No data races between main thread and workers
- Proper synchronization on shared data structures

## Design

### Core Components

#### 1. Thread Pool

```cpp
namespace RollingBonxai {

// Simple thread pool for I/O operations
class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads = 4);
    ~ThreadPool();
    
    // Non-copyable, non-movable
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    
    // Enqueue a task, returns future
    template<typename F>
    auto enqueue(F&& task) -> std::future<decltype(task())>;
    
    // Get number of pending tasks
    size_t pendingTasks() const;
    
    // Shutdown (wait for all tasks to complete)
    void shutdown();
    
private:
    void workerLoop();
    
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    
    mutable std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_;
};

// Template implementation
template<typename F>
auto ThreadPool::enqueue(F&& task) -> std::future<decltype(task())> {
    using return_type = decltype(task());
    
    auto packaged_task = std::make_shared<std::packaged_task<return_type()>>(
        std::forward<F>(task)
    );
    
    std::future<return_type> result = packaged_task->get_future();
    
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        if (stop_) {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }
        
        tasks_.emplace([packaged_task]() {
            (*packaged_task)();
        });
    }
    
    condition_.notify_one();
    return result;
}

} // namespace RollingBonxai
```

#### 2. Storage Backend Interface

```cpp
namespace RollingBonxai {

// Timestamp metadata stored alongside chunk data
struct ChunkTimestamp {
    int64_t creation_time_ns;      // When chunk first created
    int64_t last_modified_ns;      // When chunk last saved
    int64_t last_accessed_ns;      // When chunk last loaded
    uint64_t access_count;         // How many times accessed
    
    // Conversion helpers
    std::chrono::system_clock::time_point getCreationTime() const;
    std::chrono::system_clock::time_point getLastModified() const;
    std::chrono::system_clock::time_point getLastAccessed() const;
    
    // Age calculations
    std::chrono::seconds getAge() const;
    std::chrono::seconds getTimeSinceModified() const;
    std::chrono::seconds getTimeSinceAccessed() const;
};

// Abstract storage backend (user implements)
class ChunkStorageBackend {
public:
    virtual ~ChunkStorageBackend() = default;
    
    // Save chunk data to storage
    // Returns true on success, false on failure
    virtual bool save(const ChunkCoord& coord, 
                     const Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>& grid) = 0;
    
    // Load chunk data from storage
    // Returns unique_ptr to grid if found, nullptr if not found
    virtual std::unique_ptr<Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>> 
        load(const ChunkCoord& coord) = 0;
    
    // Check if chunk exists in storage
    virtual bool exists(const ChunkCoord& coord) const = 0;
    
    // Timestamp operations
    virtual std::optional<ChunkTimestamp> loadTimestamp(const ChunkCoord& coord) const = 0;
    virtual bool updateModifiedTime(const ChunkCoord& coord) = 0;
    virtual bool updateAccessTime(const ChunkCoord& coord) = 0;
};

// File-based backend implementation
// Files stored as: x_y_z.chunk (data) and x_y_z.stamp (timestamp metadata)
class FileStorageBackend : public ChunkStorageBackend {
public:
    explicit FileStorageBackend(const std::filesystem::path& base_dir);
    
    bool save(const ChunkCoord& coord, 
             const Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>& grid) override;
    
    std::unique_ptr<Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>> 
        load(const ChunkCoord& coord) override;
    
    bool exists(const ChunkCoord& coord) const override;
    
    std::optional<ChunkTimestamp> loadTimestamp(const ChunkCoord& coord) const override;
    bool updateModifiedTime(const ChunkCoord& coord) override;
    bool updateAccessTime(const ChunkCoord& coord) override;
    
private:
    // File paths: x_y_z.chunk and x_y_z.stamp
    std::filesystem::path getChunkPath(const ChunkCoord& coord) const;
    std::filesystem::path getStampPath(const ChunkCoord& coord) const;
    
    enum class TimestampField { ACCESSED, MODIFIED };
    bool updateTimestamp(const ChunkCoord& coord, TimestampField field);
    
    std::filesystem::path base_dir_;
};

} // namespace RollingBonxai
```

#### 3. Async Chunk Manager

```cpp
namespace RollingBonxai {

// Manages async loading and saving of chunks
class AsyncChunkManager {
public:
    AsyncChunkManager(std::unique_ptr<ChunkStorageBackend> backend,
                     size_t num_io_threads = 4);
    
    ~AsyncChunkManager();
    
    // Async load: returns future to chunk data
    // grid_factory: function to create empty grid if load fails
    std::future<std::shared_ptr<OccupancyGrid>> loadAsync(
        const ChunkCoord& coord,
        float priority,
        std::function<std::shared_ptr<OccupancyGrid>()> grid_factory
    );
    
    // Async save: returns future<bool> (true = success)
    std::future<bool> saveAsync(
        const ChunkCoord& coord,
        std::shared_ptr<OccupancyGrid> grid
    );
    
    // Check if chunk is currently loading
    bool isLoading(const ChunkCoord& coord) const;
    
    // Wait for all pending saves to complete (for shutdown)
    void waitForPendingSaves();
    
    // Get statistics
    struct Stats {
        uint64_t loads_requested;
        uint64_t loads_completed;
        uint64_t loads_failed;
        uint64_t saves_requested;
        uint64_t saves_completed;
        uint64_t saves_failed;
        size_t pending_loads;
        size_t pending_saves;
    };
    
    Stats getStats() const;
    
private:
    struct LoadRequest {
        ChunkCoord coord;
        float priority;
        std::function<std::shared_ptr<OccupancyGrid>()> grid_factory;
        std::shared_ptr<std::promise<std::shared_ptr<OccupancyGrid>>> promise;
        std::chrono::steady_clock::time_point enqueue_time;
        
        bool operator<(const LoadRequest& other) const {
            // Higher priority first, then FIFO
            if (priority != other.priority) {
                return priority < other.priority;  // Reverse for max-heap
            }
            return enqueue_time > other.enqueue_time;
        }
    };
    
    struct SaveRequest {
        ChunkCoord coord;
        std::shared_ptr<OccupancyGrid> grid;
        std::shared_ptr<std::promise<bool>> promise;
    };
    
    void loadWorkerLoop();
    void saveWorkerLoop();
    
    std::shared_ptr<OccupancyGrid> loadFromBackend(
        const ChunkCoord& coord,
        std::function<std::shared_ptr<OccupancyGrid>()> grid_factory
    );
    
    bool saveToBackend(const ChunkCoord& coord, 
                      std::shared_ptr<OccupancyGrid> grid);
    
    std::unique_ptr<ChunkStorageBackend> backend_;
    
    // Load queue (priority queue)
    std::priority_queue<LoadRequest> load_queue_;
    std::mutex load_queue_mutex_;
    std::condition_variable load_condition_;
    
    // Save queue (FIFO)
    std::queue<SaveRequest> save_queue_;
    std::mutex save_queue_mutex_;
    std::condition_variable save_condition_;
    
    // Track pending loads (prevent duplicates)
    std::unordered_map<
        ChunkCoord,
        std::shared_ptr<std::promise<std::shared_ptr<OccupancyGrid>>>,
        ChunkCoordHash
    > pending_loads_;
    std::mutex pending_loads_mutex_;
    
    // Worker threads
    std::vector<std::thread> load_workers_;
    std::vector<std::thread> save_workers_;
    
    // Shutdown flag
    std::atomic<bool> stop_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    Stats stats_;
};

} // namespace RollingBonxai
```

#### 4. Dirty Tracking

```cpp
namespace RollingBonxai {

// Wrapper around OccupancyGrid that tracks dirty state
class ManagedChunk {
public:
    ManagedChunk(std::shared_ptr<OccupancyGrid> grid, bool dirty = false)
        : grid_(grid), dirty_(dirty) {}
    
    // Get the underlying grid (read-only access)
    const OccupancyGrid* getGrid() const { return grid_.get(); }
    
    // Get the underlying grid (mutable access - marks as dirty)
    OccupancyGrid* getGridMutable() {
        markDirty();
        return grid_.get();
    }
    
    // Check if chunk has been modified
    bool isDirty() const { return dirty_.load(); }
    
    // Mark chunk as dirty (modified)
    void markDirty() { dirty_.store(true); }
    
    // Clear dirty flag (after save)
    void clearDirty() { dirty_.store(false); }
    
    // Get shared pointer to grid
    std::shared_ptr<OccupancyGrid> getSharedGrid() const { return grid_; }
    
private:
    std::shared_ptr<OccupancyGrid> grid_;
    std::atomic<bool> dirty_;
};

} // namespace RollingBonxai
```

### Implementation Strategy

#### Algorithm: Async Load

```
loadAsync(coord, priority):
  1. Check if already loading:
       if pending_loads contains coord:
           return existing future (duplicate request)
  
  2. Create promise and future:
       promise = new promise<OccupancyGrid*>
       future = promise.get_future()
  
  3. Add to pending loads map:
       pending_loads[coord] = promise
  
  4. Queue load request:
       request = LoadRequest{coord, priority, grid_factory, promise, now()}
       load_queue.push(request)  // priority queue
       load_condition.notify_one()
  
  5. Return future (non-blocking!)
       return future

Worker thread (load):
  while not stop:
    1. Wait for request:
         request = load_queue.pop()  // blocks if empty
    
    2. Load from backend:
         data = backend.load(coord)
    
    3. Deserialize or create new:
         if data:
             grid = grid_factory()
             grid.deserialize(data)
             dirty = false
         else:
             grid = grid_factory()  // New chunk
             dirty = true
    
    4. Complete promise:
         request.promise.set_value(grid)
         remove from pending_loads
```

#### Algorithm: Async Save (Lazy)

```
saveAsync(coord, grid):
  1. Check dirty flag:
       if not grid.isDirty():
           return immediate success (no save needed!)
  
  2. Create promise and future:
       promise = new promise<bool>
       future = promise.get_future()
  
  3. Queue save request:
       request = SaveRequest{coord, grid, promise}
       save_queue.push(request)  // FIFO
       save_condition.notify_one()
  
  4. Return future
       return future

Worker thread (save):
  while not stop:
    1. Wait for request:
         request = save_queue.pop()  // blocks if empty
    
    2. Serialize:
         data = request.grid.serialize()
    
    3. Save to backend:
         success = backend.save(coord, data)
    
    4. Complete promise:
         request.promise.set_value(success)
    
    5. Clear dirty flag if successful:
         if success:
             request.grid.clearDirty()
```

### Design Considerations

#### 1. Thread Pool Sizing

**How many threads?**

**For SSD storage:**
- 2-4 load threads
- 1-2 save threads
- More threads don't help (bandwidth saturated)

**For HDD storage:**
- 1 load thread
- 1 save thread
- Seeking is serial, more threads = worse performance

**For network storage:**
- 8-16 threads
- Latency-bound, more threads hide latency

**V2 Default:** 4 total threads (3 load, 1 save)

#### 2. Load vs Save Priority

**Question:** How to prioritize loads vs saves?

**Option 1:** Separate queues and threads (V2 choice)
- Dedicated load threads
- Dedicated save threads
- Loads never blocked by saves

**Option 2:** Single queue with priority
- Loads priority = 0.5 - 1.0
- Saves priority = 0.0 - 0.5
- Simpler but saves can block loads

**V2 Decision:** Option 1 (separate queues/threads)

#### 3. Dirty Flag Semantics

**When is chunk dirty?**

```cpp
// Load from disk
auto grid = load(coord);
grid.isDirty() == false  ✓

// Create new chunk
auto grid = factory();
grid.isDirty() == true  ✓

// Modify chunk
grid.setVoxel(x, y, z, value);
grid.isDirty() == true  ✓

// Read chunk
auto value = grid.getVoxel(x, y, z);
grid.isDirty() == unchanged  ✓
```

**Implementation:** `ManagedChunk` wrapper
- `getGrid()` - read-only, doesn't mark dirty
- `getGridMutable()` - mutable, marks dirty
- User must use mutable getter for modifications

**Alternative:** Automatic dirty detection
- Override all mutating methods in OccupancyGrid
- Automatically set dirty flag
- More invasive, requires grid modification

**V2 Choice:** Manual dirty marking via `ManagedChunk`

#### 4. Graceful Shutdown

**Problem:** Process terminating, unsaved data in memory

**Solution:** Wait for saves to complete
```cpp
~RollingBonsai() {
    // Queue saves for all dirty chunks
    for (auto& [coord, chunk] : active_chunks_) {
        if (chunk.isDirty()) {
            async_manager_->saveAsync(coord, chunk.getSharedGrid());
        }
    }
    
    // Wait for save queue to drain (with timeout)
    async_manager_->waitForPendingSaves();
}
```

**Timeout:** If saves take > 30 seconds, abort
- Log warning
- Accept data loss
- Better than hanging indefinitely

#### 5. Error Handling

**Load Failure:**
```cpp
auto future = loadAsync(coord, priority, factory);
try {
    auto grid = future.get();
    // Use grid...
} catch (const std::exception& e) {
    // Load failed, grid is empty/default
    // Log warning, continue with empty grid
}
```

**Save Failure:**
- Retry save (configurable retry count)
- If all retries fail, log error
- Keep chunk in memory (don't evict)
- User can check save stats for failures

#### 6. Memory Management

**Chunk Lifetime:**
```
1. Load initiated → Future created
2. Load completes → shared_ptr<Grid> in future
3. Main thread gets shared_ptr → ref count++
4. Store in active_chunks → ref count++
5. Evict from active_chunks → ref count--
6. Async save completes → ref count--
7. ref count == 0 → Grid destroyed
```

**No memory leaks:** shared_ptr ensures cleanup

### Edge Cases

#### Case 1: Load Request While Loading

```cpp
Time 0ms: loadAsync(coord) → Future1 (queued)
Time 10ms: loadAsync(coord) → Future2 (duplicate!)

Behavior:
  - Check pending_loads map
  - coord already present
  - Return same Future1 (reuse)
  - No duplicate load
```

#### Case 2: Save Clean Chunk

```cpp
auto grid = load(coord);  // dirty=false
// Never modify grid
evict(coord);

saveAsync(coord, grid):
  if not grid.isDirty():
      return immediate success (no I/O!)
```

**Result:** Zero write I/O for read-only chunks

#### Case 3: Modify During Save

```cpp
Time 0ms: saveAsync(coord, grid) → Queued
Time 10ms: Modify grid (dirty=true again)
Time 50ms: Save completes, clears dirty flag
Time 100ms: Evict chunk

Behavior:
  - Time 10ms modification sets dirty=true
  - Time 50ms save clears dirty flag (incorrect!)
  - Time 100ms eviction sees dirty=false (BUG!)
  - Modifications lost!

Mitigation:
  - Don't modify chunks that are being saved
  - Or: Use versioning/copy-on-write
  - V2: Document as limitation
```

#### Case 4: Shutdown with Pending Loads

```cpp
Shutdown initiated
Load queue has 10 pending requests

Behavior:
  - Complete pending loads (may take 2-3 seconds)
  - Or: Abort pending loads, empty queue
  - V2 choice: Abort loads (loads not critical at shutdown)
```

#### Case 5: Storage Full

```cpp
saveAsync(coord, grid):
  backend.save() → throws "disk full" exception

Behavior:
  - Catch exception in worker
  - Set promise exception
  - Main thread can check future
  - Log error, continue operation
  - Chunk remains in memory
```

### Testing Strategy

#### Unit Tests

**Test 1: Async Load Basic**
```cpp
TEST(AsyncChunkManager, BasicAsyncLoad) {
    auto backend = std::make_unique<MockStorageBackend>();
    AsyncChunkManager manager(std::move(backend), 2);
    
    auto factory = []() { return std::make_shared<TestGrid>(); };
    
    auto future = manager.loadAsync(ChunkCoord{0, 0, 0}, 1.0f, factory);
    
    // Should return immediately (non-blocking)
    EXPECT_TRUE(future.valid());
    
    // Wait for completion
    auto grid = future.get();
    EXPECT_NE(grid, nullptr);
}
```

**Test 2: Duplicate Load Prevention**
```cpp
TEST(AsyncChunkManager, DuplicateLoadPrevention) {
    auto backend = std::make_unique<MockStorageBackend>();
    AsyncChunkManager manager(std::move(backend), 2);
    
    auto factory = []() { return std::make_shared<TestGrid>(); };
    
    ChunkCoord coord{5, 5, 5};
    
    // First load
    auto future1 = manager.loadAsync(coord, 1.0f, factory);
    
    // Second load (duplicate)
    auto future2 = manager.loadAsync(coord, 1.0f, factory);
    
    // Should get same grid
    auto grid1 = future1.get();
    auto grid2 = future2.get();
    
    EXPECT_EQ(grid1, grid2);  // Same pointer
}
```

**Test 3: Dirty Tracking - Clean Chunk**
```cpp
TEST(ManagedChunk, CleanChunkNotSaved) {
    auto grid = std::make_shared<TestGrid>();
    ManagedChunk chunk(grid, false);  // Clean (from disk)
    
    EXPECT_FALSE(chunk.isDirty());
    
    // Read-only access doesn't mark dirty
    const OccupancyGrid* read = chunk.getGrid();
    EXPECT_FALSE(chunk.isDirty());
    
    // Should not be saved on eviction (tested by not calling save)
}
```

**Test 4: Dirty Tracking - Modified Chunk**
```cpp
TEST(ManagedChunk, ModifiedChunkMarkedDirty) {
    auto grid = std::make_shared<TestGrid>();
    ManagedChunk chunk(grid, false);  // Start clean
    
    EXPECT_FALSE(chunk.isDirty());
    
    // Mutable access marks dirty
    OccupancyGrid* mutable_grid = chunk.getGridMutable();
    mutable_grid->setVoxel(0, 0, 0, 1);
    
    EXPECT_TRUE(chunk.isDirty());
}
```

**Test 5: Priority Ordering**
```cpp
TEST(AsyncChunkManager, PriorityOrdering) {
    auto backend = std::make_unique<SlowMockBackend>(100);  // 100ms delay
    AsyncChunkManager manager(std::move(backend), 1);  // Single thread
    
    auto factory = []() { return std::make_shared<TestGrid>(); };
    
    // Queue multiple loads with different priorities
    auto future_low = manager.loadAsync(ChunkCoord{0, 0, 0}, 0.1f, factory);
    auto future_high = manager.loadAsync(ChunkCoord{1, 0, 0}, 0.9f, factory);
    auto future_med = manager.loadAsync(ChunkCoord{2, 0, 0}, 0.5f, factory);
    
    // High priority should complete first
    // (Test by timing or mock tracking load order)
}
```

**Test 6: Graceful Shutdown**
```cpp
TEST(AsyncChunkManager, GracefulShutdown) {
    auto backend = std::make_unique<MockStorageBackend>();
    auto backend_ptr = backend.get();
    
    {
        AsyncChunkManager manager(std::move(backend), 2);
        
        auto grid = std::make_shared<TestGrid>();
        grid->setVoxel(0, 0, 0, 1);
        
        auto future = manager.saveAsync(ChunkCoord{0, 0, 0}, grid);
        
        // Destructor should wait for save
    }
    
    // Check that save completed
    EXPECT_TRUE(backend_ptr->wasChunkSaved(ChunkCoord{0, 0, 0}));
}
```

**Test 7: Load Failure Handling**
```cpp
TEST(AsyncChunkManager, LoadFailureHandling) {
    auto backend = std::make_unique<FailingMockBackend>();
    AsyncChunkManager manager(std::move(backend), 2);
    
    auto factory = []() { return std::make_shared<TestGrid>(); };
    
    auto future = manager.loadAsync(ChunkCoord{0, 0, 0}, 1.0f, factory);
    
    // Should return empty grid on failure (not throw)
    auto grid = future.get();
    EXPECT_NE(grid, nullptr);  // Factory grid returned
}
```

#### Performance Tests

**Benchmark 1: Load Request Latency**
```cpp
BENCHMARK(LoadRequestLatency) {
    auto backend = std::make_unique<MockStorageBackend>();
    AsyncChunkManager manager(std::move(backend), 4);
    
    auto factory = []() { return std::make_shared<TestGrid>(); };
    
    for (int i = 0; i < 10000; ++i) {
        ChunkCoord coord{i, 0, 0};
        auto future = manager.loadAsync(coord, 1.0f, factory);
        benchmark::DoNotOptimize(future);
    }
}
// Target: < 10μs per request
```

**Benchmark 2: I/O Throughput**
```cpp
BENCHMARK(IOThroughput) {
    auto backend = std::make_unique<FileStorageBackend>("/tmp/benchmark");
    AsyncChunkManager manager(std::move(backend), 4);
    
    auto factory = []() { return std::make_shared<TestGrid>(); };
    
    // Load 100 chunks
    std::vector<std::future<std::shared_ptr<OccupancyGrid>>> futures;
    for (int i = 0; i < 100; ++i) {
        futures.push_back(manager.loadAsync(ChunkCoord{i, 0, 0}, 1.0f, factory));
    }
    
    // Wait for all
    for (auto& f : futures) {
        f.get();
    }
}
// Measure: MB/s throughput
```

### Usage Examples

**Example 1: Basic Async Load/Save**
```cpp
#include "async_chunk_manager.hpp"

int main() {
    // Create storage backend
    auto backend = std::make_unique<RollingBonxai::FileStorageBackend>(
        "/var/rolling_bonsai/chunks"
    );
    
    // Create async manager
    RollingBonxai::AsyncChunkManager async_mgr(std::move(backend), 4);
    
    // Grid factory
    auto factory = []() {
        return std::make_shared<MyOccupancyGrid>();
    };
    
    // Async load
    ChunkCoord coord{5, 10, 2};
    auto future = async_mgr.loadAsync(coord, 0.8f, factory);
    
    // Do other work while loading...
    robot.doPlanning();
    robot.processImage();
    
    // Get result (blocks if not ready)
    auto grid = future.get();
    
    // Use grid...
    grid->setVoxel(0, 0, 0, 1);
    
    // Async save (if dirty)
    auto save_future = async_mgr.saveAsync(coord, grid);
    
    // Don't need to wait for save...
    
    return 0;
}
```

**Example 2: Integration with RollingBonsai**
```cpp
class RollingBonsai {
public:
    RollingBonsai(/* ... */) {
        auto backend = std::make_unique<FileStorageBackend>(storage_path_);
        async_manager_ = std::make_unique<AsyncChunkManager>(
            std::move(backend),
            4  // threads
        );
    }
    
    void update(const Position3D& robot_pos, auto current_time) {
        // Determine desired chunks from policy
        auto desired = policy_->getDesiredChunks(/* ... */);
        
        // Load missing chunks
        for (const auto& coord : desired) {
            if (!isLoaded(coord) && !async_manager_->isLoading(coord)) {
                float priority = policy_->getPriority(coord, context);
                
                auto future = async_manager_->loadAsync(coord, priority, grid_factory_);
                pending_loads_[coord] = std::move(future);
            }
        }
        
        // Check completed loads
        for (auto it = pending_loads_.begin(); it != pending_loads_.end();) {
            if (it->second.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                auto grid = it->second.get();
                active_chunks_[it->first] = ManagedChunk(grid, false);
                it = pending_loads_.erase(it);
            } else {
                ++it;
            }
        }
        
        // Evict unwanted chunks
        for (auto it = active_chunks_.begin(); it != active_chunks_.end();) {
            if (!desired.contains(it->first)) {
                // Save if dirty
                if (it->second.isDirty()) {
                    async_manager_->saveAsync(it->first, it->second.getSharedGrid());
                }
                it = active_chunks_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    ~RollingBonsai() {
        // Graceful shutdown: save all dirty chunks
        for (auto& [coord, chunk] : active_chunks_) {
            if (chunk.isDirty()) {
                async_manager_->saveAsync(coord, chunk.getSharedGrid());
            }
        }
        
        async_manager_->waitForPendingSaves();
    }
    
private:
    std::unique_ptr<AsyncChunkManager> async_manager_;
    std::unordered_map<ChunkCoord, std::future<std::shared_ptr<OccupancyGrid>>, ChunkCoordHash> pending_loads_;
    std::unordered_map<ChunkCoord, ManagedChunk, ChunkCoordHash> active_chunks_;
};
```

---

## File Structure and Storage

### Directory Layout

```
/var/rolling_bonsai/chunks/
  0_0_0.chunk       ← Chunk data (Bonxai serialized VoxelGrid)
  0_0_0.stamp       ← Timestamp metadata (text format)
  0_0_1.chunk
  0_0_1.stamp
  5_10_2.chunk
  5_10_2.stamp
  -3_7_-2.chunk     ← Negative coordinates supported
  -3_7_-2.stamp
  ...
```

### File Naming Convention

**Chunk Files:** `{x}_{y}_{z}.chunk`
- Contains Bonxai-serialized VoxelGrid data
- Binary format (uses existing `Bonxai::Serialize()`)
- Size: 1-50 MB depending on exploration

**Timestamp Files:** `{x}_{y}_{z}.stamp`
- Contains metadata about chunk lifecycle
- Text format for easy debugging
- Size: ~60 bytes per stamp

**Example Stamp File Content:**
```
creation: 1738077135123456789
modified: 1738077522987654321
accessed: 1738079445123789456
count: 42
```
Timestamps are nanoseconds since epoch (int64).

### Why Flat Directory Structure?

**Advantages:**
- **Simplicity:** Path construction is just string concatenation
- **Debugging:** `ls *.chunk` shows all chunks at once
- **Easy queries:** `ls *_0_*.chunk` finds all z=0 chunks
- **Performance:** Modern filesystems (ext4, NTFS, APFS) handle 100k+ files easily

**When to Upgrade:**
- If chunk count exceeds 100k, consider nested structure (`x/y/z.chunk`)
- Migration is straightforward (just move files)
- V3 can add sharding if measurements show need

### Timestamp Use Cases (Future)

**V3 Features Enabled by Timestamps:**
1. **Temporal Policies:** Filter chunks by age
2. **LRU Eviction:** Evict least-recently-accessed chunks
3. **Map Analytics:** Track exploration patterns
4. **Incremental Backup:** Only backup modified chunks
5. **Map Pruning:** Delete chunks not accessed in N days

---

### Configuration Guidelines

**Thread Count Recommendations:**

| Storage Type | Load Threads | Save Threads | Total | Rationale |
|--------------|--------------|--------------|-------|-----------|
| SSD | 3 | 1 | 4 | Balanced, most common |
| HDD | 1 | 1 | 2 | Minimize seeking |
| NVMe | 4-6 | 2 | 6-8 | High bandwidth device |
| Network | 8 | 4 | 12 | Hide latency |
| SD Card | 1 | 1 | 2 | Slow, fragile |

**Memory Budgets:**
- Each pending load: ~100 bytes overhead
- Thread pool: ~10KB per thread
- For 100 concurrent loads, 4 threads: ~50KB total overhead

### Integration Considerations

#### With Center-Origin Chunks
- No special considerations
- Async manager operates on ChunkCoord directly

#### With Hysteresis
- Load requests only after hysteresis confirms transition
- Avoids loading chunks during oscillation

#### With Policy
- Policy provides priority values
- Async manager uses priorities to order queue

### Documentation Requirements

**API Documentation:**
- Doxygen comments for all async classes
- Thread safety guarantees
- Error handling behavior

**Design Documentation:**
- Thread pool architecture
- Priority queue ordering
- Dirty tracking semantics

**Performance Guide:**
- Thread count tuning
- Expected throughput
- Profiling techniques

## Acceptance Criteria

- [ ] `loadAsync()` returns immediately (< 10μs)
- [ ] Load requests processed by priority (higher first)
- [ ] Duplicate loads reuse existing request
- [ ] Dirty chunks saved on eviction
- [ ] Clean chunks NOT saved on eviction (0 write I/O)
- [ ] Graceful shutdown waits for pending saves
- [ ] Thread pool achieves > 80% of disk bandwidth
- [ ] No memory leaks over 1 hour operation
- [ ] All unit tests pass (7 tests minimum)
- [ ] Performance benchmarks meet targets
- [ ] Documentation complete: API docs + design doc + performance guide

---

## Implementation Plan

### Phase 1: Thread Pool (2 days)
- Implement ThreadPool class
- Worker loop with condition variables
- Task queue management
- Shutdown mechanism

### Phase 2: Storage Backend (1 day)
- Implement ChunkStorageBackend interface
- Implement FileStorageBackend
- File path management
- Error handling

### Phase 3: Async Manager (3 days)
- Implement AsyncChunkManager
- Load queue with priority
- Save queue (FIFO)
- Duplicate load prevention
- Worker threads for load/save

### Phase 4: Dirty Tracking (1 day)
- Implement ManagedChunk wrapper
- Dirty flag semantics
- Integration with save logic

### Phase 5: Testing (2 days)
- Unit tests (7 tests)
- Performance benchmarks
- Load failure testing
- Shutdown testing

### Phase 6: Documentation (1 day)
- API documentation
- Design document
- Performance guide
- Usage examples

**Total Estimated Time: 10 days**

---

## Risks and Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Thread synchronization bugs | High | Medium | Extensive testing, thread sanitizer |
| I/O throughput lower than expected | Medium | Low | Benchmark early, tune thread count |
| Dirty flag not set correctly | High | Medium | Clear API, document usage patterns |
| Memory leaks in async operations | High | Low | Use shared_ptr, valgrind testing |
| Shutdown hangs on pending saves | Medium | Low | Implement timeout mechanism |

---

**Component Version:** 1.0  
**Component Status:** Ready for Implementation  
**Dependencies:** Center-Origin Chunks, Chunk Loading Policy  
**Estimated Effort:** 10 person-days
