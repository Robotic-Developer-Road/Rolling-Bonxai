#include "rolling_bonxai/rolling_occupancy_map.hpp"



namespace RollingBonxai
{
    RollingOccupancyMap::RollingOccupancyMap(const AllParameters& params)
    :
    params_(params)
    {
        // Initialize coordinate system
        c_sys_ = std::make_unique<ChunkCoordinateSystem>(params_.chunk_size_);

        // Initialise transition manager
        transition_manager_ = std::make_unique<TransitionManager>(params_.hysteresis_ratio_,*c_sys_);

        // Initialise the loading policy
        if (params_.loading_policy_name == "neighbourhood") {
            loading_policy_ = std::make_unique<NeighborhoodPolicy>(params_.nb_radius);
        }
        else if (params_.loading_policy_name == "planar") {
            loading_policy_ = std::make_unique<PlanarNeighborhoodPolicy>(params_.nb_radius,
                                                                        params_.planar_z_min,
                                                                        params_.planar_z_max,
                                                                        params_.planar_use_relative);
        }
        else if (params_.loading_policy_name == "temporal") {
            loading_policy_ = std::make_unique<TemporalNeighborhoodPolicy>(params_.nb_radius,params_.temporal_age_weightage,
                                                                           std::chrono::seconds(params_.temporal_max_age_s));
        }
        else {
            // silent default to neighbourhood
            loading_policy_ = std::make_unique<NeighborhoodPolicy>(params_.nb_radius);
        }

        // Initial the file storage
        std::filesystem::path storage_path(params_.full_save_folder_path);
        storage_backend_ = std::make_unique<FileStorageBackend>(storage_path);
        storage_backend_->initStorageBackend();

        // Create the asyncio
        asyncio_manager_ = std::make_unique<AsyncChunkManager>(std::move(storage_backend_),params_.asyncio);
    }
}