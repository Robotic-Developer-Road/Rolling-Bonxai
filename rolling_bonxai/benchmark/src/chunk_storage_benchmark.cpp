/**
 * @file chunk_storage_benchmark.cpp
 * @brief Comprehensive benchmarks for ChunkStorage implementations
 * 
 * Benchmarks all three storage implementations (Vector, HashMap, Deque) across
 * different neighborhood sizes (radius 1-5) to measure:
 * - Insert performance
 * - Find performance (hit and miss)
 * - Contains performance
 * - Erase performance
 * - Iteration performance (forEachChunk)
 * - Mixed workload patterns
 */

#include <iostream>
#include <memory>
#include <vector>
#include <random>
#include <algorithm>

// nanobench for high-precision benchmarking
#define ANKERL_NANOBENCH_IMPLEMENT
#include "nanobench.h"

// Rolling Bonxai includes
#include "rolling_bonxai/chunk_storage.hpp"
#include "rolling_bonxai/common.hpp"
#include "rolling_bonxai/coordinate_system.hpp"
#include "bonxai_map/occupancy_map.hpp"

using namespace RollingBonxai;
using namespace ankerl;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Helper to create storage with capacity hint (handles different constructors)
 */
template <typename StorageT>
struct StorageFactory {
    static StorageT create(size_t capacity_hint) {
        return StorageT(capacity_hint);
    }
};

// Specialization for DequeChunkStorage (no capacity hint needed)
template <>
struct StorageFactory<DequeChunkStorage> {
    static DequeChunkStorage create(size_t /*capacity_hint*/) {
        return DequeChunkStorage();
    }
};

/**
 * @brief Create a dummy ManagedChunk for benchmarking
 * @param coord Chunk coordinate
 * @return ManagedChunk with minimal OccupancyMap
 */
ManagedChunk createDummyChunk(const ChunkCoord& coord) {
    auto map = std::make_unique<Bonxai::OccupancyMap>(0.1); // 10cm resolution
    return ManagedChunk(std::move(map), const_cast<ChunkCoord&>(coord), false);
}

/**
 * @brief Generate chunk coordinates for a given neighborhood radius
 * @param radius Neighborhood radius (1 = 27 chunks, 2 = 125 chunks, etc.)
 * @return Vector of chunk coordinates
 */
std::vector<ChunkCoord> generateNeighborhood(int radius) {
    std::vector<ChunkCoord> coords;
    ChunkCoord center{0, 0, 0};
    
    // Calculate expected size: (2*radius + 1)^3
    int expected_size = (2 * radius + 1) * (2 * radius + 1) * (2 * radius + 1);
    coords.reserve(expected_size);
    
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dz = -radius; dz <= radius; ++dz) {
                coords.emplace_back(center.x + dx, center.y + dy, center.z + dz);
            }
        }
    }
    
    return coords;
}

/**
 * @brief Generate random chunk coordinates (for miss testing)
 * @param count Number of coordinates to generate
 * @param seed Random seed
 * @return Vector of random chunk coordinates
 */
std::vector<ChunkCoord> generateRandomCoords(size_t count, unsigned seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int32_t> dist(-1000, 1000);
    
    std::vector<ChunkCoord> coords;
    coords.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        coords.emplace_back(dist(rng), dist(rng), dist(rng));
    }
    
    return coords;
}

/**
 * @brief Pre-populate storage with chunks
 * @tparam StorageT Storage type (VectorChunkStorage, HashMapChunkStorage, DequeChunkStorage)
 * @param storage Storage to populate
 * @param coords Coordinates to insert
 */
template <typename StorageT>
void populateStorage(StorageT& storage, const std::vector<ChunkCoord>& coords) {
    for (const auto& coord : coords) {
        storage.insert(coord, createDummyChunk(coord));
    }
}

// ============================================================================
// Benchmark: Insert Operations
// ============================================================================

template <typename StorageT>
void benchmarkInsert(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    bench.run(storage_name + " insert (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
        for (const auto& coord : coords) {
            storage.insert(coord, createDummyChunk(coord));
            nanobench::doNotOptimizeAway(storage.size());
        }
    });
}

// ============================================================================
// Benchmark: Find Operations (Hit)
// ============================================================================

template <typename StorageT>
void benchmarkFindHit(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    // Pre-populate storage
    StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
    populateStorage(storage, coords);
    
    // Benchmark finding existing chunks (best, average, worst case)
    ChunkCoord first = coords.front();
    ChunkCoord middle = coords[coords.size() / 2];
    ChunkCoord last = coords.back();
    
    bench.run(storage_name + " find_hit_first (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        auto* chunk = storage.find(first);
        nanobench::doNotOptimizeAway(chunk);
    });
    
    bench.run(storage_name + " find_hit_middle (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        auto* chunk = storage.find(middle);
        nanobench::doNotOptimizeAway(chunk);
    });
    
    bench.run(storage_name + " find_hit_last (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        auto* chunk = storage.find(last);
        nanobench::doNotOptimizeAway(chunk);
    });
}

// ============================================================================
// Benchmark: Find Operations (Miss)
// ============================================================================

template <typename StorageT>
void benchmarkFindMiss(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    // Pre-populate storage
    StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
    populateStorage(storage, coords);
    
    // Generate coordinate that definitely doesn't exist
    ChunkCoord missing{9999, 9999, 9999};
    
    bench.run(storage_name + " find_miss (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        auto* chunk = storage.find(missing);
        nanobench::doNotOptimizeAway(chunk);
    });
}

// ============================================================================
// Benchmark: Contains Operations
// ============================================================================

template <typename StorageT>
void benchmarkContains(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    // Pre-populate storage
    StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
    populateStorage(storage, coords);
    
    ChunkCoord existing = coords[coords.size() / 2];
    ChunkCoord missing{9999, 9999, 9999};
    
    bench.run(storage_name + " contains_hit (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        bool result = storage.contains(existing);
        nanobench::doNotOptimizeAway(result);
    });
    
    bench.run(storage_name + " contains_miss (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        bool result = storage.contains(missing);
        nanobench::doNotOptimizeAway(result);
    });
}

// ============================================================================
// Benchmark: Erase Operations
// ============================================================================

template <typename StorageT>
void benchmarkErase(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    // Test erasing first, middle, last elements
    bench.run(storage_name + " erase_first (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
        populateStorage(storage, coords);
        storage.erase(coords.front());
        nanobench::doNotOptimizeAway(storage.size());
    });
    
    bench.run(storage_name + " erase_middle (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
        populateStorage(storage, coords);
        storage.erase(coords[coords.size() / 2]);
        nanobench::doNotOptimizeAway(storage.size());
    });
    
    bench.run(storage_name + " erase_last (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
        populateStorage(storage, coords);
        storage.erase(coords.back());
        nanobench::doNotOptimizeAway(storage.size());
    });
}

// ============================================================================
// Benchmark: Iteration (forEachChunk)
// ============================================================================

template <typename StorageT>
void benchmarkIteration(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    // Pre-populate storage
    StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
    populateStorage(storage, coords);
    
    bench.run(storage_name + " iterate (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        size_t count = 0;
        storage.forEachChunk([&count](const ChunkCoord&, const ManagedChunk&) {
            ++count;
        });
        nanobench::doNotOptimizeAway(count);
    });
}

// ============================================================================
// Benchmark: Mixed Workload (Realistic Usage Pattern)
// ============================================================================

template <typename StorageT>
void benchmarkMixedWorkload(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    bench.run(storage_name + " mixed_workload (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
        
        // Simulate robot movement: load chunks, query, evict
        // Pattern: Insert 80% of chunks
        size_t load_count = static_cast<size_t>(coords.size() * 0.8);
        for (size_t i = 0; i < load_count; ++i) {
            storage.insert(coords[i], createDummyChunk(coords[i]));
        }
        
        // Query chunks (mix of hits and misses)
        for (size_t i = 0; i < coords.size(); ++i) {
            auto* chunk = storage.find(coords[i]);
            nanobench::doNotOptimizeAway(chunk);
        }
        
        // Evict 30% of chunks
        size_t evict_count = static_cast<size_t>(load_count * 0.3);
        for (size_t i = 0; i < evict_count; ++i) {
            storage.erase(coords[i]);
        }
        
        nanobench::doNotOptimizeAway(storage.size());
    });
}

// ============================================================================
// Benchmark: Memory Usage (Capacity Growth)
// ============================================================================

template <typename StorageT>
void benchmarkMemoryGrowth(nanobench::Bench& bench, const std::string& storage_name, int radius) {
    auto coords = generateNeighborhood(radius);
    int num_chunks = static_cast<int>(coords.size());
    
    bench.run(storage_name + " capacity_growth (r=" + std::to_string(radius) + ", n=" + std::to_string(num_chunks) + ")", [&] {
        StorageT storage = StorageFactory<StorageT>::create(static_cast<size_t>(num_chunks));
        for (const auto& coord : coords) {
            storage.insert(coord, createDummyChunk(coord));
        }
        // Check final capacity
        size_t cap = storage.capacity();
        nanobench::doNotOptimizeAway(cap);
    });
}

// ============================================================================
// Main Benchmark Suite
// ============================================================================

void runAllBenchmarks() {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  Chunk Storage Benchmarks\n";
    std::cout << "========================================\n";
    std::cout << "\n";
    
    // Radii to test: 1, 2, 3, 4, 5
    // Corresponding chunk counts: 27, 125, 343, 729, 1331
    std::vector<int> radii = {1, 2, 3, 4, 5};
    
    for (int radius : radii) {
        int num_chunks = (2 * radius + 1) * (2 * radius + 1) * (2 * radius + 1);
        
        std::cout << "\n";
        std::cout << "========================================\n";
        std::cout << "  Radius: " << radius << " (n=" << num_chunks << " chunks)\n";
        std::cout << "========================================\n";
        
        // Configure benchmark
        nanobench::Bench bench;
        bench.title("ChunkStorage Benchmarks")
             .unit("op")
             .warmup(100)
             .minEpochIterations(1000);
        
        // Run all benchmark categories for each storage type
        std::cout << "\n--- Insert Benchmarks ---\n";
        benchmarkInsert<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkInsert<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkInsert<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Find (Hit) Benchmarks ---\n";
        benchmarkFindHit<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkFindHit<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkFindHit<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Find (Miss) Benchmarks ---\n";
        benchmarkFindMiss<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkFindMiss<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkFindMiss<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Contains Benchmarks ---\n";
        benchmarkContains<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkContains<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkContains<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Erase Benchmarks ---\n";
        benchmarkErase<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkErase<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkErase<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Iteration Benchmarks ---\n";
        benchmarkIteration<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkIteration<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkIteration<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Mixed Workload Benchmarks ---\n";
        benchmarkMixedWorkload<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkMixedWorkload<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkMixedWorkload<DequeChunkStorage>(bench, "Deque", radius);
        
        std::cout << "\n--- Memory Growth Benchmarks ---\n";
        benchmarkMemoryGrowth<VectorChunkStorage>(bench, "Vector", radius);
        benchmarkMemoryGrowth<HashMapChunkStorage>(bench, "HashMap", radius);
        benchmarkMemoryGrowth<DequeChunkStorage>(bench, "Deque", radius);
    }
    
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  Benchmarks Complete!\n";
    std::cout << "========================================\n";
    std::cout << "\n";
}

int main() {
    std::cout << "Starting Chunk Storage Benchmarks...\n";
    std::cout << "Using nanobench v4.1.0\n";
    
    try {
        runAllBenchmarks();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error during benchmarking: " << e.what() << "\n";
        return 1;
    }
}