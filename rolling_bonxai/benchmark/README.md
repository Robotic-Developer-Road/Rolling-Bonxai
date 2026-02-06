# Rolling Bonxai Chunk Storage Benchmarks

Comprehensive performance benchmarks for the three ChunkStorage implementations (Vector, HashMap, Deque) using [nanobench](https://nanobench.ankerl.com/).

## Overview

These benchmarks measure performance across different neighborhood radii (1-5), corresponding to typical robotics usage patterns:

| Radius | Chunk Count | Use Case |
|--------|-------------|----------|
| 1 | 27 | Minimal neighborhood (26 neighbors + source) |
| 2 | 125 | Standard neighborhood |
| 3 | 343 | Extended neighborhood |
| 4 | 729 | Large neighborhood |
| 5 | 1331 | Very large neighborhood |

## Benchmark Categories

### 1. **Insert Operations**
- Measures time to insert chunks into empty storage
- Tests all chunks in a neighborhood

### 2. **Find Operations (Hit)**
- Tests finding existing chunks
- Measures best case (first), average case (middle), worst case (last)

### 3. **Find Operations (Miss)**
- Tests finding non-existent chunks
- Measures worst-case search performance

### 4. **Contains Operations**
- Similar to find but only returns boolean
- Tests both hit and miss cases

### 5. **Erase Operations**
- Measures time to remove chunks
- Tests erasing first, middle, and last elements

### 6. **Iteration (forEachChunk)**
- Measures time to iterate over all chunks
- Tests the traversal performance

### 7. **Mixed Workload**
- Realistic usage pattern:
  - Insert 80% of chunks
  - Query all chunks (mix of hits/misses)
  - Evict 30% of chunks
- Simulates robot movement through environment

### 8. **Memory Growth**
- Measures capacity after inserting all chunks
- Useful for understanding memory overhead

## Building

### Option 1: Build with Colcon (Recommended for ROS2 workspace)

```bash
# In your workspace root
colcon build --packages-select rolling_bonxai --cmake-args -DBUILD_BENCHMARKS=ON

# Run benchmarks
./install/rolling_bonxai/lib/rolling_bonxai/benchmarks/chunk_storage_benchmark
```

### Option 2: Build Standalone (CMake only)

```bash
cd rolling_bonxai
mkdir build && cd build
cmake .. -DBUILD_BENCHMARKS=ON
make -j$(nproc)

# Run benchmarks
./benchmarks/chunk_storage_benchmark
```

## Running Benchmarks

### Basic Usage

```bash
./chunk_storage_benchmark
```

### Redirecting Output

```bash
# Save results to file
./chunk_storage_benchmark > results.txt 2>&1

# View with pager
./chunk_storage_benchmark | less
```

### Expected Runtime

- Full benchmark suite: ~2-5 minutes (depends on CPU)
- Per-radius benchmarks: ~30-60 seconds

## Interpreting Results

nanobench provides the following metrics:

- **ns/op**: Nanoseconds per operation (lower is better)
- **op/s**: Operations per second (higher is better)
- **err%**: Relative error percentage (lower means more stable)
- **total**: Total benchmark time

### Example Output

```
|               ns/op |                op/s |    err% |     total | benchmark
|--------------------:|--------------------:|--------:|----------:|:----------
|              123.45 |        8,100,000.00 |    0.5% |      0.01 | Vector insert (r=1, n=27)
|               45.67 |       21,900,000.00 |    0.3% |      0.01 | HashMap insert (r=1, n=27)
|              156.78 |        6,380,000.00 |    0.7% |      0.01 | Deque insert (r=1, n=27)
```

### Performance Expectations

Based on design:

**Vector** (Linear search):
- Insert: O(n) - slower for duplicate check
- Find: O(n) - degrades with size
- Best for: n < 100

**HashMap**:
- Insert: O(1) avg - fastest
- Find: O(1) avg - fastest
- Best for: n > 100 or when speed is critical

**Deque**:
- Insert: O(n) - similar to Vector
- Find: O(n) - similar to Vector
- Best for: stable pointers needed (rare)

## Customizing Benchmarks

### Adding New Radii

Edit `chunk_storage_benchmark.cpp`:

```cpp
// Line ~370
std::vector<int> radii = {1, 2, 3, 4, 5, 6};  // Add radius 6
```

### Adjusting Warmup/Epochs

Edit benchmark configuration:

```cpp
nanobench::Bench bench;
bench.warmup(100)      // Change warmup iterations
     .epochs(1000);    // Change measurement iterations
```

### Adding Custom Benchmarks

Add new benchmark function:

```cpp
template <typename StorageT>
void benchmarkMyCustomTest(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    // Your benchmark code here
    bench.run(storage_name + " my_test", [&] {
        // Measured code
    });
}

// Call in runAllBenchmarks()
benchmarkMyCustomTest<VectorChunkStorage>(bench, "Vector", radius);
```

## Troubleshooting

### nanobench Not Found

Ensure CMake can fetch from GitHub:

```bash
# Test connectivity
git clone https://github.com/martinus/nanobench.git /tmp/nanobench-test
rm -rf /tmp/nanobench-test
```

If offline, manually download nanobench:

```bash
cd rolling_bonxai/benchmark/include
git clone https://github.com/martinus/nanobench.git
```

Then modify CMakeLists.txt to use local copy.

### Compilation Errors

Check C++ standard:

```bash
# CMake should show C++17
cmake .. -DBUILD_BENCHMARKS=ON | grep "CXX_STANDARD"
```

### Inconsistent Results

Ensure system is idle during benchmarks:

```bash
# Check CPU frequency governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Set to performance mode (Linux)
sudo cpupower frequency-set -g performance

# Disable CPU turbo (for more stable results)
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo
```

## Benchmarking Best Practices

1. **Close other applications** before running
2. **Run multiple times** to verify consistency
3. **Use performance CPU governor** on Linux
4. **Disable CPU frequency scaling** for stable results
5. **Check thermal throttling** (CPU temperature)

## Analyzing Results

### Which Storage to Use?

Based on benchmark results:

- **n ≤ 50**: All three perform similarly, use Vector (simplest)
- **50 < n ≤ 150**: Vector still competitive, HashMap starts to win
- **n > 150**: HashMap clearly faster
- **Stable pointers needed**: Deque (rare requirement)

### Generating Comparison Charts

Export results and use Python/gnuplot:

```bash
./chunk_storage_benchmark > results.txt

# Parse results with Python (example)
python3 parse_results.py results.txt --output chart.png
```

## References

- [nanobench documentation](https://nanobench.ankerl.com/)
- [nanobench tutorial](https://nanobench.ankerl.com/tutorial.html)

## License

Same as parent project (see main LICENSE file).