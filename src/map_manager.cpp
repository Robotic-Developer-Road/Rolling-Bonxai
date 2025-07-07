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
    
    void MapManager::updateMap(PCLPointCloud& points,PCLPoint& origin)
    {
        //The voxel coordinate where the origin is found in
        auto voxel_id_sensor = this->mapPointToVoxelCoord(origin);

        //The chunk coordinate where the origin voxel is found in
        auto chunk_coord_sensor = this->voxelCoordToChunkCoord(voxel_id_sensor);

        //How many neibors and how many chunks are in play
        constexpr size_t neibors = 26;
        constexpr size_t num_chunks_in_play = neibors + 1; //neibor chunks and me

        //Create a vector containing all the chunks in play. Should be a maximum of 27
        std::vector<Bonxai::CoordT> chunks_in_play;

        //Reserve memory for faster pushbacks
        chunks_in_play.reserve(num_chunks_in_play);

        //Push in the chunk containing the origin
        chunks_in_play.push_back(chunk_coord_sensor);

        //Get the neibor coordinates
        auto face_neibors = getFaceNeibors(chunk_coord_sensor);
        auto edge_neibors = getEdgeNeibors(chunk_coord_sensor);
        auto corner_neibors = getCornerNeibors(chunk_coord_sensor);

        for (const auto &face : face_neibors) {chunks_in_play.push_back(face);}
        for (const auto &edge : edge_neibors) {chunks_in_play.push_back(edge);}
        for (const auto &corner : corner_neibors) {chunks_in_play.push_back(corner);}
        

        if (first_update_)
        {
            chunk_manager_->initFirstChunks(chunk_coord_sensor,chunks_in_play);
            first_update_ = false;
        }

        else
        {
            chunk_manager_->updateChunks(points,origin,chunks_in_play);
        }
    }

    std::array<Bonxai::CoordT,6> MapManager::getFaceNeibors(const Bonxai::CoordT& coord)
    {
        //Follow the right-handle rule system X(front),Y(Left), Z(Up)
        return {{
            {coord.x + 1, coord.y, coord.z}, //front
            {coord.x - 1, coord.y, coord.z}, //back
            {coord.x, coord.y + 1, coord.z}, //left
            {coord.x, coord.y - 1, coord.z}, //right
            {coord.x, coord.y, coord.z + 1}, //up
            {coord.x, coord.y, coord.z - 1}  //down
        }};
    }

    std::array<Bonxai::CoordT,12> MapManager::getEdgeNeibors(const Bonxai::CoordT& coord)
    {
        //Follow the right-handle rule system X(front),Y(Left), Z(Up)
        return {{
            // xy-plane edges
            {coord.x - 1, coord.y - 1, coord.z}, //back-right
            {coord.x - 1, coord.y + 1, coord.z}, //back-left
            {coord.x + 1, coord.y - 1, coord.z}, //front-right
            {coord.x + 1, coord.y + 1, coord.z}, //front-left

            // xz-plane edges
            {coord.x - 1, coord.y, coord.z - 1}, //back-down
            {coord.x - 1, coord.y, coord.z + 1}, //back-up
            {coord.x + 1, coord.y, coord.z - 1}, //front-down
            {coord.x + 1, coord.y, coord.z + 1}, //front-up

            // yz-plane edges
            {coord.x, coord.y - 1, coord.z - 1}, //right-down
            {coord.x, coord.y - 1, coord.z + 1}, //right-up
            {coord.x, coord.y + 1, coord.z - 1}, //left-down
            {coord.x, coord.y + 1, coord.z + 1}  //left-up
        }};
    }

    std::array<Bonxai::CoordT,8> MapManager::getCornerNeibors(const Bonxai::CoordT& coord) 
    {
        //Follow the right-handle rule system X(front),Y(Left), Z(Up)
        return {{
            {coord.x - 1, coord.y - 1, coord.z - 1}, //back  - right  -  down
            {coord.x - 1, coord.y - 1, coord.z + 1}, //back  - right  -  up
            {coord.x - 1, coord.y + 1, coord.z - 1}, //back  - left   -  down
            {coord.x - 1, coord.y + 1, coord.z + 1}, //back  - left  -  up
            {coord.x + 1, coord.y - 1, coord.z - 1}, //front - right  -  down
            {coord.x + 1, coord.y - 1, coord.z + 1}, //front - right  -  up
            {coord.x + 1, coord.y + 1, coord.z - 1}, //front - left   -  down
            {coord.x + 1, coord.y + 1, coord.z + 1}  //front - left   -  up
        }};
    }

    Bonxai::CoordT MapManager::mapPointToVoxelCoord(const PCLPoint& point)
    {
        return {
            static_cast<int32_t>(std::floor(point.x * inv_resolution_)),
            static_cast<int32_t>(std::floor(point.y * inv_resolution_)),
            static_cast<int32_t>(std::floor(point.z * inv_resolution_))};
    }

    MapManager::PCLPoint MapManager::voxelCoordToMapPoint(const Bonxai::CoordT& coord)
    {
        return {
            (static_cast<double>(coord.x)) * mp_.resolution,
            (static_cast<double>(coord.y)) * mp_.resolution,
            (static_cast<double>(coord.z)) * mp_.resolution};
    }

    MapManager::PCLPoint MapManager::voxelCoordToMapCenterPoint(const Bonxai::CoordT& coord)
    {
        double half = 0.5 * mp_.resolution;
        return {
            (static_cast<double>(coord.x)) * mp_.resolution + half,
            (static_cast<double>(coord.y)) * mp_.resolution + half,
            (static_cast<double>(coord.z)) * mp_.resolution + half};
    }

    Bonxai::CoordT MapManager::voxelCoordToChunkCoord(const Bonxai::CoordT& vc)
    {
        const int32_t chunk_size = cp_.chunk_dim;

        return {
        vc.x >= 0 ? vc.x / chunk_size : (vc.x - chunk_size + 1) / chunk_size,
        vc.y >= 0 ? vc.y / chunk_size : (vc.y - chunk_size + 1) / chunk_size,
        vc.z >= 0 ? vc.z / chunk_size : (vc.z - chunk_size + 1) / chunk_size
        };
    }

    Bonxai::CoordT MapManager::chunkCoordToVoxelCoord(const Bonxai::CoordT& cc)
    {
        const int32_t chunk_size = cp_.chunk_dim;
        return {
            cc.x * chunk_size,
            cc.y * chunk_size,
            cc.z * chunk_size
        };
    }

    Bonxai::CoordT MapManager::mapFramePointToChunkFrameCoord(const PCLPoint& mP)
    {
        using Coord = Bonxai::CoordT;

        // Convert the Map Frame Point mP to Map Frame Voxel Coordinate mVC
        Coord mVC = this->mapPointToVoxelCoord(mP);

        // Find the Coordinate of the Chunk mCC
        Coord mCC = this->voxelCoordToChunkCoord(mVC);

        // Get the Origin Voxel in the Chunk as a Voxel Coordinate in map frame
        Coord mOriginVC = this->chunkCoordToVoxelCoord(mCC);

        // And Finally, make the mVC we calculated earlier relative to mOriginVC
        Coord cVC = mVC - mOriginVC;

        return cVC;

    }


}// namespace RM