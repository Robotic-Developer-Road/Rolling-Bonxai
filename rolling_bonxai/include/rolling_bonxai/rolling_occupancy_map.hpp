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

#include <vector>
#include <unordered_map>
#include <memory>
#include <atomic>
#include <optional>
#include <chrono>
#include <future>

/*
*/
namespace RollingBonxai
{
class RollingOccupancyMap
{
public:
    /// Factory function type for creating new OccupancyMaps when a chunk has no data on disk
    using OccupancyMapFactory = std::function<std::unique_ptr<Bonxai::OccupancyMap>()>;
    using OccupancyOptionsFactory = std::function<Bonxai::Occupancy::OccupancyOptions()>;
    
    struct AllParameters 
    {
        // map params containing parameters pertaining to each chunk aka OccupancyGrid
        MapParams map_params;

        // Coordinate System
        double chunk_size_;

        // Transition Manaher
        double hysteresis_ratio_;

        // Loading policy
        std::string loading_policy_name{"neighbourhood"};
        int nb_radius{2};
        int planar_z_min{1};
        int planar_z_max{3};
        bool planar_use_relative{false};
        double temporal_age_weightage{0.3};
        int temporal_max_age_s{300.0};
        
        // File storage
        std::string full_save_folder_path;

        // Aysyncio params
        AsyncChunkManagerConfig asyncio;
    };

    /**
     * @brief
     */
    RollingOccupancyMap(const AllParameters& all_params_);

private:
    AllParameters params_;

    // Declare the Coordinate System
    std::unique_ptr<ChunkCoordinateSystem> c_sys_;

    // Declare the Transition Manager
    std::unique_ptr<TransitionManager> transition_manager_;

    // Declare the active chunks
    HashMapChunkStorage active_chunks_;

    // Declare the loading policy
    std::unique_ptr<ChunkLoadingPolicy> loading_policy_;

    // Declare the filesystem
    std::unique_ptr<FileStorageBackend> storage_backend_;

    // Declare the Asyncio
    std::unique_ptr<AsyncChunkManager> asyncio_manager_;

    // Pending Futures
    std::unordered_map<
        ChunkCoord,
        std::shared_ptr<std::promise<std::unique_ptr<Bonxai::OccupancyMap>>>,
        ChunkCoordHash
    > pending_loads_;

    // Factory methods for OccupancyMap and Occupancy Options
    OccupancyMapFactory main_occupancy_map_factory_;
    OccupancyOptionsFactory main_occupancy_options_factory_;



};
}