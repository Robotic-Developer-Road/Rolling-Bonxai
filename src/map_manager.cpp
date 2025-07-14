#include "rolling_map/map_manager.h"


namespace RM
{
    MapManager::MapManager()
    {
 
        return;
    }

    MapManager::MapManager(SensorParams& sensor_params,MapParams& map_params,ChunkParams& chunk_params)
    :
    sp_{sensor_params},
    mp_{map_params},
    cp_{chunk_params},
    inv_resolution_{1.0 / mp_.resolution}
    {
        //Create Chunk manaer
        chunk_manager_ = std::make_unique<ChunkManager>(sp_,mp_,cp_);
    }
    
    bool MapManager::updateMap(PCLPointCloud& points,PCLPoint& origin)
    {
        //The voxel coordinate where the origin is found in
        auto voxel_coord_sensor = utils::mapPointToVoxelCoord(origin,mp_.resolution);

        //The chunk coordinate where the origin voxel is found in
        auto chunk_coord_sensor = utils::voxelCoordToChunkCoord(voxel_coord_sensor,cp_.chunk_dim);

        if (first_update_)
        {
            chunk_manager_->initFirstChunks(chunk_coord_sensor);
            first_update_ = false;
            return false;
        }

        else
        {
            bool able_to_update = chunk_manager_->updateChunks(points,origin);
            return able_to_update;
        }
    }
    
    void MapManager::getFreeVoxels(PCLPointCloud& points)
    {
        chunk_manager_->getAllFreeVoxels(points);
    }

    void MapManager::getOccupiedVoxels(PCLPointCloud& points)
    {
        chunk_manager_->getAllOccupiedVoxels(points);
    }

}// namespace RM