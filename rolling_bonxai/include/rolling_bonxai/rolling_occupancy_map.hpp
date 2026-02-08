#pragma once

#include "bonxai_core/bonxai.hpp"
#include "bonxai_map/occupancy_map.hpp"
#include "rolling_bonxai/common.hpp"
#include "rolling_bonxai/coordinate_system.hpp"
#include "rolling_bonxai/chunk_storage.hpp"
#include "rolling_bonxai/chunk_loading_policies.hpp"
#include "rolling_bonxai/transition_manager.hpp"
#include "rolling_bonxai/file_storage_backend.hpp"
#include "rolling_bonxai/task_queue.hpp"
#include "rolling_bonxai/async_chunk_manager.hpp"
#include "rolling_bonxai/logger.hpp"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <atomic>
#include <optional>
#include <chrono>
#include <future>
#include <functional>
#include <cmath>

namespace RollingBonxai
{

/**
 * @class RollingOccupancyMap
 * @brief Orchestrator for chunked, rolling-window occupancy mapping
 * 
 * Manages a set of spatially-partitioned OccupancyMap chunks that are
 * loaded/evicted as the robot moves through the environment. Point clouds
 * are dispatched to the correct chunks, with rays that span chunk boundaries
 * handled via upfront raytracing and per-voxel dispatch.
 * 
 * Owns and coordinates:
 * - ChunkCoordinateSystem (spatial indexing)
 * - TransitionManager (hysteresis-based chunk transitions)
 * - ChunkStorage (active chunks in memory)
 * - AsyncChunkManager (async disk I/O)
 * - ChunkLoadingPolicy (load/evict decisions)
 * 
 * @thread_safety
 * - insertPointCloud must be called from a single thread (main update thread)
 * - AsyncChunkManager handles its own thread safety for I/O
 * - Query methods (isOccupied, etc.) are safe for concurrent reads if no
 *   insertPointCloud is in progress
 */
class RollingOccupancyMap
{
    using Vector3D = Eigen::Vector3d;
    using CoordT = Bonxai::CoordT;

public:

    /// Factory function types for creating OccupancyMaps and OccupancyOptions
    using OccupancyMapFactory = std::function<std::unique_ptr<Bonxai::OccupancyMap>()>;
    using OccupancyOptionsFactory = std::function<Bonxai::Occupancy::OccupancyOptions()>;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief All parameters needed to initialise the RollingOccupancyMap
     */
    struct AllParameters 
    {
        /// Map parameters for each chunk (resolution, sensor model, etc.)
        MapParams map_params;

        /// Chunk size in meters for the coordinate system
        double chunk_size{5.0};

        /// Hysteresis ratio for the TransitionManager (fraction of chunk size)
        double hysteresis_ratio{0.20};

        /// Loading policy selection and parameters
        std::string loading_policy_name{"neighbourhood"};
        int nb_radius{2};
        int planar_z_min{1};
        int planar_z_max{3};
        bool planar_use_relative{false};
        double temporal_age_weightage{0.3};
        int temporal_max_age_s{300};

        /// Full path to folder for chunk file storage
        std::string full_save_folder_path;

        /// Async I/O thread pool configuration
        AsyncChunkManagerConfig asyncio;
    };

    // ========================================================================
    // Construction & Destruction
    // ========================================================================

    /**
     * @brief Construct and initialise the rolling occupancy map
     * 
     * Initialises all subsystems: coordinate system, transition manager,
     * loading policy, file storage backend, and async I/O manager.
     * 
     * @param all_params Configuration parameters
     * @param logger Shared pointer to a Logger implementation (e.g., RosLogger)
     */
    RollingOccupancyMap(const AllParameters& all_params, std::shared_ptr<Logger> logger);

    /// Destructor
    ~RollingOccupancyMap();

    /// Non-copyable
    RollingOccupancyMap(const RollingOccupancyMap&) = delete;
    RollingOccupancyMap& operator=(const RollingOccupancyMap&) = delete;

    /// Non-movable (owns references via TransitionManager -> ChunkCoordinateSystem&)
    RollingOccupancyMap(RollingOccupancyMap&&) = delete;
    RollingOccupancyMap& operator=(RollingOccupancyMap&&) = delete;

    // ========================================================================
    // Primary Update Interface
    // ========================================================================

    /**
     * @brief Insert a point cloud observation (primary update function)
     * 
     * Converts inputs to Eigen::Vector3d, then performs occupancy update
     * (dispatching rays across chunks) followed by rolling update
     * (transition management, chunk loading/eviction).
     * 
     * @tparam PointT Point type with x,y,z fields (e.g., pcl::PointXYZ, Eigen::Vector3d)
     * @tparam Allocator Allocator type for vector
     * 
     * @param points Point cloud in map (global) frame
     * @param source Sensor position in map frame
     * @param source_lin_vel Sensor linear velocity in map frame
     * @param max_range Maximum valid sensor range (meters)
     */
    template <typename PointT, typename Allocator = std::allocator<PointT>>
    void insertPointCloud(const std::vector<PointT, Allocator>& points,
                          const PointT& source,
                          const PointT& source_lin_vel,
                          double max_range)
    {
        // Convert source and velocity to Eigen::Vector3d
        const Vector3D source_eigen = Bonxai::ConvertPoint<Vector3D>(source);
        const Vector3D velocity_eigen = Bonxai::ConvertPoint<Vector3D>(source_lin_vel);

        // Convert all points to Eigen::Vector3d
        std::vector<Vector3D> points_eigen;
        points_eigen.reserve(points.size());
        for (const auto& pt : points) {
            points_eigen.emplace_back(Bonxai::ConvertPoint<Vector3D>(pt));
        }

        // Perform occupancy update (dispatch rays to chunks)
        updateOccupancy(points_eigen, source_eigen, max_range);

        // Perform rolling update (transitions, load/evict)
        updateRolling(source_eigen, velocity_eigen);
    }

    // ========================================================================
    // Statistics (aggregated across all active chunks)
    // ========================================================================

    /**
     * @brief Get total number of active (allocated) cells across all chunks
     * @return Sum of active cell counts from all active chunks
     */
    [[nodiscard]] size_t getActiveCellCount() const noexcept;

    /**
     * @brief Get the number of active chunks
     * @return size_t number of active chunks where ManagedChunk.isMapValid() == true
     */
    [[nodiscard]] size_t getActiveChunkCount() const;

    /**
     * @brief Get the number of dirty chunks
     * @return size_t number of dirty chunks that are active AND isDirty
     */
    [[nodiscard]] size_t getDirtyChunkCount() const;

    /**
     * @brief Get the number of clean chunks
     * @return size_t number of clean chunks that are active AND isDirty
     */
    [[nodiscard]] size_t getCleanChunkCount() const;

    /**
     * @brief Get a std::vector<pair<Vec3d,bool>> containing the center
     * position of a chunk and whether it is clean or dirty
     */
    [[nodiscard]] std::vector<std::pair<Vector3D,bool>> getChunkStates() const;

    /**
     * @brief Get total memory usage in bytes across all chunks
     * @return Sum of memory usage from all active chunks
     */
    [[nodiscard]] size_t getMemoryUsage() const noexcept;

    /**
     * @brief Get comprehensive statistics aggregated across all active chunks
     * 
     * Fields are summed (cell counts, memory, counters) or computed as
     * global min/max/mean across all chunks.
     * 
     * @param compute_expensive If true, iterates all cells in all chunks
     *        for bounding box, probability distributions, etc.
     * @return Aggregated OccupancyMapStats
     */
    [[nodiscard]] Bonxai::Occupancy::OccupancyMapStats getStats(bool compute_expensive = false) const;

    /**
     * @brief Get quick statistics aggregated across all active chunks (no cell iteration)
     * @return Aggregated basic statistics
     */
    [[nodiscard]] Bonxai::Occupancy::OccupancyMapStats getQuickStats() const noexcept;

    // ========================================================================
    // Memory Management
    // ========================================================================

    /**
     * @brief Release unused memory from all active chunks
     * 
     * Iterates all active chunks and calls releaseUnusedMemory() on each.
     * Frees leaf grids where all cells are OFF.
     * 
     * @warning Invalidates all existing accessors in all chunks
     */
    void releaseUnusedMemory();

    /**
     * @brief Shrink internal coordinate buffers in all active chunks
     * 
     * Less aggressive than releaseUnusedMemory().
     * 
     */
    void shrinkToFit();

    // ========================================================================
    // Query Interface
    // ========================================================================

    /**
     * @brief Check if a world position is occupied
     * 
     * @param position World position to query
     * @return true if the voxel is occupied in the relevant chunk
     * @return false if free, unknown, or chunk not loaded
     */
    [[nodiscard]] bool isOccupied(const Vector3D& position) const;

    /**
     * @brief Check if a voxel coordinate is occupied
     * 
     * Determines the chunk from the voxel coordinate and forwards the query.
     * 
     * @param coord Global voxel coordinate
     * @return true if probability_log > occupancy_threshold_log
     * @return false if probability_log <= threshold, unknown, or chunk not loaded
     */
    [[nodiscard]] bool isOccupied(const CoordT& coord) const;

    /**
     * @brief Check if a world position is free
     * 
     * @param position World position to query
     * @return true if the voxel is marked free
     * @return false if occupied, unknown, or chunk not loaded
     */
    [[nodiscard]] bool isFree(const Vector3D& position) const;

    /**
     * @brief Check if a voxel coordinate is free
     * 
     * @param coord Global voxel coordinate
     * @return true if probability_log < occupancy_threshold_log
     * @return false if probability_log >= threshold, unknown, or chunk not loaded
     */
    [[nodiscard]] bool isFree(const CoordT& coord) const;

    /**
     * @brief Check if a world position is unknown
     * 
     * @param position World position to query
     * @return true if the voxel is unknown or chunk not loaded
     */
    [[nodiscard]] bool isUnknown(const Vector3D& position) const;

    /**
     * @brief Check if a voxel coordinate is unknown
     * 
     * @param coord Global voxel coordinate
     * @return true if voxel never observed, probability == threshold, or chunk not loaded
     */
    [[nodiscard]] bool isUnknown(const CoordT& coord) const;

    /**
     * @brief Extract all occupied voxel coordinates across all active chunks
     * @param coords Output vector (cleared before filling)
     * @complexity O(N) where N = total active cells across all chunks
     */
    void getOccupiedVoxels(std::vector<CoordT>& coords) const;

    /**
     * @brief Extract all free voxel coordinates across all active chunks
     * @param coords Output vector (cleared before filling)
     * @complexity O(N)
     */
    void getFreeVoxels(std::vector<CoordT>& coords) const;

    /**
     * @brief Extract occupied voxels as point cloud across all active chunks
     * @tparam PointT Point type with x,y,z fields (e.g., Eigen::Vector3d, pcl::PointXYZ)
     * @param points Output vector (cleared before filling)
     */
    template <typename PointT>
    void getOccupiedVoxels(std::vector<PointT>& points) const {
        // Extract the coords
        std::vector<CoordT> coords;
        getOccupiedVoxels(coords);
        
        // Clear and reserve space for the points
        points.clear();
        points.reserve(coords.size());
        
        // For every coord - > Bonxai::CoordT, convert to positionCoordinate
        for (const auto& coord : coords) {
            // this world coord point is a Vec3D
            auto world_coord_point = c_sys_->voxelToPositionCoordinate(coord);
            // Convert to whatever PointT the user is asking for
            auto world_point_to_return = Bonxai::ConvertPoint<PointT>(world_coord_point);
            // Push back into the points
            points.push_back(world_point_to_return);
        }
    }

    /**
     * @brief Extract free voxels as point cloud across all active chunks
     * @tparam PointT Point type with x,y,z fields
     * @param points Output vector (cleared before filling)
     * @complexity O(N)
     */
    template <typename PointT>
    void getFreeVoxels(std::vector<PointT>& points) const {
        // Grab the free voxels
        std::vector<CoordT> coords;
        getFreeVoxels(coords);

        // Clear and reserve space
        points.clear();
        points.reserve(coords.size());

        
        for (const auto& coord : coords) {
            // this world coord point is a Vec3D
            auto world_coord_point = c_sys_->voxelToPositionCoordinate(coord);
            // Convert to whatever PointT the user is asking for
            auto world_point_to_return = Bonxai::ConvertPoint<PointT>(world_coord_point);
            // Push back into the points
            points.push_back(world_point_to_return);
        }
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /**
     * @brief Get const reference to the coordinate system
     */
    [[nodiscard]] const ChunkCoordinateSystem& getCoordinateSystem() const;

    /**
     * @brief Get const reference to active chunk storage
     */
    [[nodiscard]] const HashMapChunkStorage& getActiveChunks() const;

    /**
     * @brief Get the current reference chunk coordinate (from transition manager)
     */
    [[nodiscard]] ChunkCoord getCurrentChunk() const;

    /**
     * @brief Get number of points dropped due to unloaded chunks
     */
    [[nodiscard]] uint64_t getDroppedPointCount() const noexcept;

    /**
     * @brief Get the parameters
     */
    [[nodiscard]] const AllParameters& getParams() const noexcept;

    /**
     * @brief Get the OccupancyMap factory function
     */
    [[nodiscard]] const OccupancyMapFactory& getMapFactory() const noexcept;

    /**
     * @brief Get the OccupancyOptions factory function
     */
    [[nodiscard]] const OccupancyOptionsFactory& getOptionsFactory() const noexcept;

private:

    // ========================================================================
    // Internal Update Methods
    // ========================================================================

    /**
     * @brief Perform occupancy update by dispatching rays across active chunks
     * 
     * For each point:
     *   1. Classify as hit or miss based on max_range
     *   2. Dispatch endpoint to correct chunk (addHitPoint/addMissPoint)
     *   3. Raytrace source→endpoint, dispatching intermediates to chunks
     *      with boundary-check optimization (6 integer comparisons per voxel,
     *      recompute chunk only on boundary crossing)
     *   4. After all points: call updateFreeCellsPreRayTrace() on each touched chunk
     * 
     * Points whose endpoint or intermediate voxels land in unloaded chunks are
     * silently skipped (counted in dropped_points_).
     * 
     * @param points Point cloud in map frame (Eigen::Vector3d)
     * @param source Sensor position in map frame
     * @param max_range Maximum valid sensor range (meters)
     */
    void updateOccupancy(const std::vector<Vector3D>& points,
                         const Vector3D& source,
                         double max_range);

    /**
     * @brief Perform rolling update (transition management, chunk loading/eviction)
     * 
     * @param source Current sensor position in map frame
     * @param source_lin_vel Current sensor velocity in map frame
     * 
     * @note Stub for now — will be implemented in part 2
     */
    void updateRolling(const Vector3D& source,
                       const Vector3D& source_lin_vel);

    // ========================================================================
    // Internal Helpers
    // ========================================================================
    
    /**
     * @brief Build the OccupancyMapFactory from current MapParams
     * 
     * Creates a lambda that constructs OccupancyMap(resolution, options)
     */
    void buildFactories();

    // ========================================================================
    // Member Variables
    // ========================================================================

    /// Configuration
    AllParameters params_;

    /// Coordinate system for chunk spatial indexing
    std::unique_ptr<ChunkCoordinateSystem> c_sys_;

    /// Transition manager for hysteresis-based chunk transitions
    std::unique_ptr<TransitionManager> transition_manager_;

    /// Active chunks currently loaded in memory
    HashMapChunkStorage active_chunks_;

    /// Metadata tracking for policy decisions
    std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash> chunk_metadata_;

    /// Chunk loading/eviction policy
    std::unique_ptr<ChunkLoadingPolicy> loading_policy_;

    /// File storage backend (ownership transferred to asyncio_manager_ after construction)
    /// This pointer becomes null after AsyncChunkManager takes ownership
    std::unique_ptr<FileStorageBackend> storage_backend_;

    /// Async I/O manager for non-blocking chunk load/save
    std::unique_ptr<AsyncChunkManager> asyncio_manager_;

    /// Logger
    Logger::SharedPtr logger_;

    /// Factory for creating new OccupancyMap instances
    OccupancyMapFactory map_factory_;

    /// Factory for creating OccupancyOptions from MapParams
    OccupancyOptionsFactory options_factory_;

    /// Cached inverse resolution for voxel coordinate math
    double inv_resolution_{0.0};

    /// Counter for points dropped due to dispatch to unloaded chunks
    uint64_t dropped_points_{0};
};

} // namespace RollingBonxai