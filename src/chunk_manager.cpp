#include "rolling_map/chunk_manager.h"

namespace RM
{
    ChunkManager::ChunkManager(SensorParams& sensor_params,MapParams& map_params,ChunkParams& chunk_params)
    :
    sensor_params_(sensor_params),
    map_params_(map_params),
    chunk_params_(chunk_params)
    {
        folder_pth_ = chunk_params_.chunk_folder_path;

        //init with null pointers
        chunks_.fill(nullptr);
        chunks_metadata_.fill(Chunkmd());
        map_live_.fill(false);

        //Set the options
        moption_.prob_miss_log = Bonxai::MapUtils::logods(sensor_params_.probability_miss);
        moption_.prob_hit_log = Bonxai::MapUtils::logods(sensor_params_.probability_hit);
        moption_.clamp_min_log = Bonxai::MapUtils::logods(map_params_.occupancy_min_threshold);
        moption_.clamp_max_log = Bonxai::MapUtils::logods(map_params_.occupancy_max_threshold);

        //Initialise the threads
        read_thread_ = std::thread(&ChunkManager::readWorker,this);
        write_thread_ = std::thread(&ChunkManager::writeWorker,this);

    }

    ChunkManager::ChunkManager()
    {
        chunks_.fill(nullptr);
        chunks_metadata_.fill(Chunkmd());
        map_live_.fill(false);
    }

    ChunkManager::~ChunkManager()
    {
        io_running_ = false;
        read_cv_.notify_all();
        write_cv_.notify_all();
        
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }

        if (write_thread_.joinable())
        {
            write_thread_.join();
        }
        return;
    }

    void ChunkManager::setSourceCoord(Bonxai::CoordT& source)
    {
        current_source_coord_ = source;
    }

    void ChunkManager::initFirstChunks(Bonxai::CoordT &source_chunk)
    {
        setSourceCoord(source_chunk);
        for (size_t i = 0; i < 27; ++i)
        {
            //Make a new map
            MapPtr mptr = std::make_shared<Bonxai::OccupancyMap>(map_params_.resolution,moption_);
            chunks_[i] = mptr;

            auto ctype = indexToChunkType(i);
            auto coord = getChunkCoord(ctype,source_chunk);

            chunks_metadata_[i].coord = coord;
            chunks_metadata_[i].key = chunkCoordToChunkKey(coord);
            chunks_metadata_[i].state = ChunkState::CLEAN;
            chunks_metadata_[i].touched_once = false;
        }
        //We are ready to go.
        map_live_.fill(true);
    }

    bool ChunkManager::updateChunks(PCLPointCloud &points,PCLPoint& origin)
    {
        auto source_voxel = utils::mapPointToVoxelCoord(origin,map_params_.resolution);
        auto source_chunk = utils::voxelCoordToChunkCoord(source_voxel,chunk_params_.chunk_dim);
        
        // isAnyBad();

        //Check if there are any read operations in progress
        if (hasPendingIO())
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rolling-map-node"),"IO operation in progress. Skipping frame.");
            //The entire cache is in a transient state so we return right away
            return false;
        }
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rolling-map-node"),"Can Update");
        //Check the current center coord
        bool is_same_source = (current_source_coord_ == source_chunk);

        //If the source has changed, this means that 
        if (!is_same_source)
        {
            //Update the occupancy
            updateAllOccupancy(points,origin,source_chunk);
            //Update the cache
            updateCache(source_chunk);
        }

        else
        {
            //Just update all the occupancies
            updateAllOccupancy(points,origin,source_chunk);
        }
        return true;
    }

    void ChunkManager::updateAllOccupancy(PCLPointCloud &points,PCLPoint& origin,
                                          Bonxai::CoordT& source_chunk)
    {
        using V3D = Bonxai::OccupancyMap::Vector3D;
        //Vectors to store hit or miss voxel coords
        std::vector<Bonxai::CoordT> end_point_voxels_map;
        Bonxai::CoordT sensor_voxel = utils::mapPointToVoxelCoord(origin,map_params_.resolution);
        //Origin as V3D
        V3D origin_map_v3d = Bonxai::ConvertPoint<V3D>(origin);
        //Iterate through the pointcloud
        for (const auto& p_map : points)
        {
            //Convert p_world to V3D
            V3D p_world_v3d = Bonxai::ConvertPoint<V3D>(p_map);
            V3D from_to = p_world_v3d - origin_map_v3d;
            double distance_squared = from_to.squaredNorm();
            double range_squared = sensor_params_.max_range * sensor_params_.max_range;
            bool is_hit = distance_squared < range_squared;

            if (is_hit)
            {
                //Convert to Voxel, update the relevant map then add to the hit voxels collection
                Bonxai::CoordT hit_voxel = utils::mapPointToVoxelCoord(p_map,map_params_.resolution);
                updateAllHitPoints(hit_voxel,source_chunk);
                end_point_voxels_map.push_back(hit_voxel);
            }
            else
            {
                //Distance between the sensor and point is greater than or equal to the range
                //So we have no idea if it actually hit anything at the range so we take it as a miss.
                V3D from_to_unit = from_to / std::sqrt(distance_squared);
                V3D from_to_range = from_to_unit * sensor_params_.max_range;
                
                //Compute a point along the vector that is equal to the range
                V3D new_to_point_v3d = origin_map_v3d + from_to_range;
                PCLPoint new_to_point = Bonxai::ConvertPoint<PCLPoint>(new_to_point_v3d);

                //Convert the point to a voxel coordinate in world frame
                Bonxai::CoordT missed_voxel = utils::mapPointToVoxelCoord(new_to_point,map_params_.resolution);
                updateAllMissPoints(missed_voxel,source_chunk);
                end_point_voxels_map.push_back(missed_voxel);
            }
        }
        //Update the free cells
        updateAllFreeCells(end_point_voxels_map,source_chunk,sensor_voxel);
        //Finally, increment the update count
        incrementUpdateCount();
    }
    void ChunkManager::updateAllHitPoints(Bonxai::CoordT& hit_voxel,Bonxai::CoordT& source_chunk)
    {
        //Get the parent chunk
        Bonxai::CoordT parent_chunk = utils::voxelCoordToChunkCoord(hit_voxel,chunk_params_.chunk_dim);
        //
        if (is26neibor(source_chunk,parent_chunk))
        {
            //Voxel Coordinate of parent chunk
            Bonxai::CoordT parent_chunk_voxel_origin = utils::chunkCoordToVoxelCoord(parent_chunk,chunk_params_.chunk_dim);
            Bonxai::CoordT relative_voxel = hit_voxel - parent_chunk_voxel_origin;
            //This relative voxel coordinate is the voxel coordinate in the chunk frame
            ChunkType ctype = getChunkType(source_chunk,parent_chunk);
            size_t target_index = chunkTypeToIndex(ctype);

            if (chunks_.at(target_index)->isAccessorBound())
            {
                chunks_.at(target_index)->addHitPoint(relative_voxel);
            }
            else
            {
                auto accessor = chunks_.at(target_index)->getGrid().createAccessor();
                chunks_.at(target_index)->addHitPoint(relative_voxel,accessor);
            }

            //mark parent chunk as dirty if its not already
            if (getChunkState(target_index) != ChunkState::DIRTY) {markDirty(target_index);}
            //This chunk was touched during the update
            this->chunks_metadata_[target_index].touched_once = true;
        }
    }

    void ChunkManager::updateAllMissPoints(Bonxai::CoordT& miss_voxel,Bonxai::CoordT& source_chunk)
    {
        //Get the parent chunk
        Bonxai::CoordT parent_chunk = utils::voxelCoordToChunkCoord(miss_voxel,chunk_params_.chunk_dim);
        
        if (is26neibor(source_chunk,parent_chunk))
        {
            //Voxel Coordinate of parent chunk
            Bonxai::CoordT parent_chunk_voxel_origin = utils::chunkCoordToVoxelCoord(parent_chunk,chunk_params_.chunk_dim);
            Bonxai::CoordT relative_voxel = miss_voxel - parent_chunk_voxel_origin;
            //This relative voxel coordinate is the voxel coordinate in the chunk frame
            ChunkType ctype = getChunkType(source_chunk,parent_chunk);
            size_t target_index = chunkTypeToIndex(ctype);

            if (chunks_.at(target_index)->isAccessorBound())
            {
                chunks_.at(target_index)->addMissPoint(relative_voxel);
            }
            else
            {
                auto accessor = chunks_.at(target_index)->getGrid().createAccessor();
                chunks_.at(target_index)->addMissPoint(relative_voxel,accessor);
            }
            //mark the chunk as dirty if its not already dirty
            if (getChunkState(target_index) != ChunkState::DIRTY) {markDirty(target_index);}
            //This chunk was touched during the update
            this->chunks_metadata_[target_index].touched_once = true;
        }
    }

    void ChunkManager::updateAllHitOrMissPoints(Bonxai::CoordT& target_voxel,Bonxai::CoordT& source_chunk,bool is_hit)
    {
        if (is_hit)
        {
            updateAllHitPoints(target_voxel,source_chunk);
        }
        else
        {
            updateAllMissPoints(target_voxel,source_chunk);
        }
    }

    void ChunkManager::updateAllFreeCells(std::vector<Bonxai::CoordT> &end_voxels, Bonxai::CoordT& source_chunk,Bonxai::CoordT &sensor_voxel)
    {
        auto clearPointLambda = [this,&source_chunk](const Bonxai::CoordT& coord) 
        {
            //This gonna be the exact same as updateMissPoint
            Bonxai::CoordT parent_chunk = utils::voxelCoordToChunkCoord(coord,chunk_params_.chunk_dim);
            if (is26neibor(source_chunk,parent_chunk))
            {
                //Voxel Coordinate of parent chunk
                Bonxai::CoordT parent_chunk_voxel_origin = utils::chunkCoordToVoxelCoord(parent_chunk,chunk_params_.chunk_dim);
                Bonxai::CoordT relative_voxel = coord - parent_chunk_voxel_origin;
                //This relative voxel coordinate is the voxel coordinate in the chunk frame
                ChunkType ctype = getChunkType(source_chunk,parent_chunk);
                size_t target_index = chunkTypeToIndex(ctype);

                if (this->chunks_.at(target_index)->isAccessorBound())
                {
                    this->chunks_.at(target_index)->addMissPoint(relative_voxel);
                }
                else
                {
                    auto accessor = chunks_.at(target_index)->getGrid().createAccessor();
                    this->chunks_.at(target_index)->addMissPoint(relative_voxel,accessor);
                }
                //mark the chunk as dirty if its not already dirty
                if (getChunkState(target_index) != ChunkState::DIRTY) {markDirty(target_index);}
                //This chunk was touched during the update
                this->chunks_metadata_[target_index].touched_once = true;
            }
            return true;
        }; // clearPointLambda
        
        //Iterate through all the end voxels and raycast update
        for (const auto& end_voxel : end_voxels)
        {
            Bonxai::MapUtils::RayIterator(sensor_voxel,end_voxel,clearPointLambda);
        }
    }

    void ChunkManager::incrementUpdateCount()
    {
        for (size_t i = 0 ; i < CACHE_SIZE ; ++i)
        {
            if (chunks_metadata_.at(i).touched_once)
            {
                this->chunks_[i]->incrementUpdateCount();
            }
            this->chunks_metadata_[i].touched_once = false;
        }
    }

    void ChunkManager::getAllOccupiedVoxels(PCLPointCloud& points)
    {
        if (hasPendingIO())
        {
            return;
        }
        points.clear();

        for (int i = 0 ; i < chunks_.size() ; ++i)
        {
            Bonxai::CoordT cc_origin_voxel_map = utils::chunkCoordToVoxelCoord(chunks_metadata_[i].coord,chunk_params_.chunk_dim);

            auto visitor = [this,&cc_origin_voxel_map,&points](Bonxai::MapUtils::CellOcc &cell,const Bonxai::CoordT& coord)
            {
                if (cell.probability_log > moption_.occupancy_threshold_log)
                {
                    Bonxai::CoordT map_voxel = coord + cc_origin_voxel_map;
                    auto pcpt = utils::voxelCoordToMapPoint(map_voxel,map_params_.resolution);
                    points.push_back(pcpt);
                }

            };

            chunks_[i] -> getGrid().forEachCell(visitor); 
        }
    }

    void ChunkManager::getAllFreeVoxels(PCLPointCloud& points)
    {
        if (hasPendingIO())
        {
            return;
        }

        points.clear();

        for (int i = 0 ; i < chunks_.size() ; ++i)
        {
            Bonxai::CoordT cc_origin_voxel_map = utils::chunkCoordToVoxelCoord(chunks_metadata_[i].coord,chunk_params_.chunk_dim);

            auto visitor = [this,&cc_origin_voxel_map,&points](Bonxai::MapUtils::CellOcc &cell,const Bonxai::CoordT& coord)
            {
                if (cell.probability_log < moption_.occupancy_threshold_log)
                {
                    Bonxai::CoordT map_voxel = coord + cc_origin_voxel_map;
                    auto pcpt = utils::voxelCoordToMapPoint(map_voxel,map_params_.resolution);
                    points.push_back(pcpt);
                }
            };
            chunks_[i] -> getGrid().forEachCell(visitor); 
        }
        
    }

    void ChunkManager::updateCache(Bonxai::CoordT& new_source_chunk)
    {   
        /*
        There are 3 map types when it comes to updates: Persist,Evict,Load
        Persist Maps: Maps that are currently in the cache that are still neibors of the new source chunk.
        Evict Maps: Maps that are currently in the cache that are no longer neibors of the new source chunk.
        Load Maps: Maps that need to be loaded because they are not in the current cache but are neibors of the new source chunk

        There are 2 modes of eviction.
        1. Just kill the chunk if its clean (has not been updated)
        2. Save the chunk to disc if its dirty (has been updated)

        There are 2 modes of load.
        1. Create a new map if the map doesnt exist in the disc
        2. Deserialize and load the map from disc

        Eviction Requires a ChunkKey and MapPtr
        Loading just requires a ChunKey

        Workflow: Persist common chunks to the previous and current source & evict chunks that are not required --> Read/create new chunks
         */

        RCLCPP_WARN_STREAM(rclcpp::get_logger("rolling-map-node"), "Updating Cache!");
        
        //Disable all the maps
        map_live_.fill(false);
        //Save a copy of the current source chunks
        Bonxai::CoordT current_source_chunk = this->current_source_coord_;
        //Update the source chunk store with the new source chunk
        setSourceCoord(new_source_chunk);       
        //Create a new state array
        std::array<Chunkmd,27> new_chunks_md;
        //Create the new chunks array
        std::array<MapPtr,27> new_chunks;

        //Populate the new chunk states array with the appropriate key and UNSET state based on the new neibor
        for (size_t i = 0 ; i < CACHE_SIZE ; ++i)
        {
            //Create a new metadata object
            Chunkmd new_chunk_md;
            //Get the chunk type enumeration
            auto chunktype = indexToChunkType(i);
            //Get the corresponding coordinate that matches the enumeration and the new source coord
            new_chunk_md.coord = getChunkCoord(chunktype,new_source_chunk);
            //Convert this to the corresponding Key
            new_chunk_md.key = chunkCoordToChunkKey(new_chunk_md.coord);
            //Add the state
            new_chunk_md.state = ChunkState::UNSET;
            //reset the touch
            new_chunk_md.touched_once = false;
            //Add the new key at that position
            new_chunks_md[i] = new_chunk_md;
            //Create an empty map pointer at that position in the new chunks
            MapPtr empty_mptr;
            new_chunks[i] = empty_mptr;
        }
        
        //Beyond this point, we will begin mutating the chunks.
        for (size_t cache_idx = 0 ; cache_idx < CACHE_SIZE; ++cache_idx)
        {
            const Chunkmd curr_md = chunks_metadata_.at(cache_idx);
            //Check for persistence fist. Persistence will happen if current_chunk_coordinate is either the new_source_coord or a neibor of the new_source_coord
            if (curr_md.coord == new_source_chunk || is26neibor(new_source_chunk,curr_md.coord))
            {
                //Find the new chunk type of this coordinate in the new cache
                ChunkType updated_chunk_type = getChunkType(new_source_chunk,curr_md.coord);
                //Get the target index
                size_t updated_cache_idx = chunkTypeToIndex(updated_chunk_type);                
                //Swapping the allocated map pointer in the cache from its original cache_idx to an updated_cache_idx
                std::swap(chunks_[cache_idx],new_chunks[updated_cache_idx]);
                //Update the state of this chunk in the new chunk metadata
                new_chunks_md[updated_cache_idx].state = curr_md.state;
                //Update the map liveliness
                map_live_[updated_cache_idx] = true;
            }
            else //Eviction block because the current chunk is neither the source nor a neibor. So it should not be there
            {
                if (curr_md.state == ChunkState::CLEAN)
                {
                    //There was no update. Can safely let the map go
                    chunks_[cache_idx].reset();
                }
                else
                {
                    ChunkKey currkey = curr_md.key;
                    //There was an update. We need to give it to the Write Queue
                    wqPush(currkey,chunks_[cache_idx]);
                    chunks_[cache_idx].reset();
                }
            }
        }

        //Replace the new chunks array with the existing chunks.
        chunks_ = std::move(new_chunks);
        //Replace the new states array with the existing states
        chunks_metadata_ = std::move(new_chunks_md);
        //Create a vector for the read tasks
        std::vector<std::pair<ChunkKey,size_t>> read_tasks;
        //Loop through the updated states and chunks
        for (size_t i = 0 ; i < 27 ; ++i)
        {
            const Chunkmd updated_md = chunks_metadata_.at(i);
            //Read it if its not a neibor of the current source chunk and its not the current source chunk
            if (!is26neibor(current_source_chunk,updated_md.coord) && updated_md.coord != current_source_chunk)
            {
                if (updated_md.state != ChunkState::UNSET)
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rolling-map-node"),"This chunk update needs to be UNSET!");
                }
                //Read it
                read_tasks.push_back(std::make_pair(updated_md.key,i));
            }
        }

        //Push the read tasks to the read queue
        for (auto task : read_tasks)
        {
            rqPush(task.first,task.second);
        }

    }

    ChunkManager::ChunkType ChunkManager::getChunkType(const Bonxai::CoordT& source,const Bonxai::CoordT& c)
    {
        //this way is pretty gross
        auto delta = c - source;

        using C = Bonxai::CoordT;

        if (delta == C{0, 0, 0}) return ChunkType::SOURCE;

        // Faces
        if (delta == C{1, 0, 0})    return ChunkType::FACE_F;
        if (delta == C{-1, 0, 0})   return ChunkType::FACE_B;
        if (delta == C{0, 1, 0})    return ChunkType::FACE_L;
        if (delta == C{0, -1, 0})   return ChunkType::FACE_R;
        if (delta == C{0, 0, 1})    return ChunkType::FACE_T;
        if (delta == C{0, 0, -1})   return ChunkType::FACE_D;

        // Edges
        if (delta == C{-1, -1, 0})  return ChunkType::EDGE_B_R;
        if (delta == C{-1,  1, 0})  return ChunkType::EDGE_B_L;
        if (delta == C{ 1, -1, 0})  return ChunkType::EDGE_F_R;
        if (delta == C{ 1,  1, 0})  return ChunkType::EDGE_F_L;
        if (delta == C{-1,  0, -1}) return ChunkType::EDGE_B_D;
        if (delta == C{-1,  0,  1}) return ChunkType::EDGE_B_T;
        if (delta == C{ 1,  0, -1}) return ChunkType::EDGE_F_D;
        if (delta == C{ 1,  0,  1}) return ChunkType::EDGE_F_T;
        if (delta == C{ 0, -1, -1}) return ChunkType::EDGE_R_D;
        if (delta == C{ 0, -1,  1}) return ChunkType::EDGE_R_T;
        if (delta == C{ 0,  1, -1}) return ChunkType::EDGE_L_D;
        if (delta == C{ 0,  1,  1}) return ChunkType::EDGE_L_T;

        // Corners
        if (delta == C{-1, -1, -1}) return ChunkType::CORNER_B_R_D;
        if (delta == C{-1, -1,  1}) return ChunkType::CORNER_B_R_T;
        if (delta == C{-1,  1, -1}) return ChunkType::CORNER_B_L_D;
        if (delta == C{-1,  1,  1}) return ChunkType::CORNER_B_L_T;
        if (delta == C{ 1, -1, -1}) return ChunkType::CORNER_F_R_D;
        if (delta == C{ 1, -1,  1}) return ChunkType::CORNER_F_R_T;
        if (delta == C{ 1,  1, -1}) return ChunkType::CORNER_F_L_D;
        if (delta == C{ 1,  1,  1}) return ChunkType::CORNER_F_L_T;

        return ChunkType::INVALID;
    }

    size_t ChunkManager::chunkTypeToIndex(ChunkType type)
    {
        return static_cast<size_t>(type);
    }

    ChunkManager::ChunkType ChunkManager::indexToChunkType(size_t index)
    {
        
        if (index == 0) {return ChunkManager::ChunkType::SOURCE;} 
        if (index == 1) {return ChunkManager::ChunkType::FACE_F;}       
        if (index == 2) {return ChunkManager::ChunkType::FACE_B;}       
        if (index == 3) {return ChunkManager::ChunkType::FACE_L;}      
        if (index == 4) {return ChunkManager::ChunkType::FACE_R;}        
        if (index == 5) {return ChunkManager::ChunkType::FACE_T;}        
        if (index == 6) {return ChunkManager::ChunkType::FACE_D;}       
        if (index == 7 ) {return ChunkManager::ChunkType::EDGE_B_R    ;}     
        if (index == 8 ) {return ChunkManager::ChunkType::EDGE_B_L    ;}     
        if (index == 9 ) {return ChunkManager::ChunkType::EDGE_F_R    ;}      
        if (index == 10) {return ChunkManager::ChunkType::EDGE_F_L    ;}    
        if (index == 11) {return ChunkManager::ChunkType::EDGE_B_D    ;}  
        if (index == 12) {return ChunkManager::ChunkType::EDGE_B_T    ;}    
        if (index == 13) {return ChunkManager::ChunkType::EDGE_F_D    ;}    
        if (index == 14) {return ChunkManager::ChunkType::EDGE_F_T    ;}    
        if (index == 15) {return ChunkManager::ChunkType::EDGE_R_D    ;}   
        if (index == 16) {return ChunkManager::ChunkType::EDGE_R_T    ;}    
        if (index == 17) {return ChunkManager::ChunkType::EDGE_L_D    ;}   
        if (index == 18) {return ChunkManager::ChunkType::EDGE_L_T    ;}    
        if (index == 19) {return ChunkManager::ChunkType::CORNER_B_R_D;} 
        if (index == 20) {return ChunkManager::ChunkType::CORNER_B_R_T;} 
        if (index == 21) {return ChunkManager::ChunkType::CORNER_B_L_D;} 
        if (index == 22) {return ChunkManager::ChunkType::CORNER_B_L_T;} 
        if (index == 23) {return ChunkManager::ChunkType::CORNER_F_R_D;} 
        if (index == 24) {return ChunkManager::ChunkType::CORNER_F_R_T;} 
        if (index == 25) {return ChunkManager::ChunkType::CORNER_F_L_D;} 
        if (index == 26) {return ChunkManager::ChunkType::CORNER_F_L_T;} 
        
        return ChunkManager::ChunkType::INVALID;
    }

    Bonxai::CoordT ChunkManager::getChunkCoord(ChunkManager::ChunkType ctype,const Bonxai::CoordT& source)
    {
        using C = Bonxai::CoordT;

        if (ctype == ChunkType::SOURCE){return source + C{0,0,0};}    

        // Faces
        if (ctype == ChunkType::FACE_F) {return source + C{1, 0, 0} ;}    
        if (ctype == ChunkType::FACE_B) {return source + C{-1, 0, 0};}  
        if (ctype == ChunkType::FACE_L) {return source + C{0, 1, 0} ;}    
        if (ctype == ChunkType::FACE_R) {return source + C{0, -1, 0};}  
        if (ctype == ChunkType::FACE_T) {return source + C{0, 0, 1} ;} 
        if (ctype == ChunkType::FACE_D) {return source + C{0, 0, -1};} 

        // Edges
        if (ctype == ChunkType::EDGE_B_R) {return source + C{-1, -1, 0} ;} 
        if (ctype == ChunkType::EDGE_B_L) {return source + C{-1,  1, 0} ;} 
        if (ctype == ChunkType::EDGE_F_R) {return source + C{ 1, -1, 0} ;} 
        if (ctype == ChunkType::EDGE_F_L) {return source + C{ 1,  1, 0} ;} 
        if (ctype == ChunkType::EDGE_B_D) {return source + C{-1,  0, -1};} 
        if (ctype == ChunkType::EDGE_B_T) {return source + C{-1,  0,  1};} 
        if (ctype == ChunkType::EDGE_F_D) {return source + C{ 1,  0, -1};} 
        if (ctype == ChunkType::EDGE_F_T) {return source + C{ 1,  0,  1};} 
        if (ctype == ChunkType::EDGE_R_D) {return source + C{ 0, -1, -1};} 
        if (ctype == ChunkType::EDGE_R_T) {return source + C{ 0, -1,  1};} 
        if (ctype == ChunkType::EDGE_L_D) {return source + C{ 0,  1, -1};} 
        if (ctype == ChunkType::EDGE_L_T) {return source + C{ 0,  1,  1};} 

        // Corners
        if(ctype == ChunkType::CORNER_B_R_D){return source + C{-1, -1, -1};}
        if(ctype == ChunkType::CORNER_B_R_T){return source + C{-1, -1,  1};}
        if(ctype == ChunkType::CORNER_B_L_D){return source + C{-1,  1, -1};}
        if(ctype == ChunkType::CORNER_B_L_T){return source + C{-1,  1,  1};}
        if(ctype == ChunkType::CORNER_F_R_D){return source + C{ 1, -1, -1};}
        if(ctype == ChunkType::CORNER_F_R_T){return source + C{ 1, -1,  1};}
        if(ctype == ChunkType::CORNER_F_L_D){return source + C{ 1,  1, -1};}
        if(ctype == ChunkType::CORNER_F_L_T){return source + C{ 1,  1,  1};}

        throw(std::runtime_error("Invalid chunk type"));
    }

    ChunkManager::ChunkKey ChunkManager::chunkCoordToChunkKey(const Bonxai::CoordT& cc)
    {
        std::string key = std::to_string(cc.x) + "_" + std::to_string(cc.y) + "_" + std::to_string(cc.z);
        return key;
    }

    Bonxai::CoordT ChunkManager::chunkKeyToChunkCoord(ChunkKey& key)
    {
        std::istringstream ss(key);
        std::string token;
        Bonxai::CoordT coord;

        if (std::getline(ss, token, '_')) coord.x = std::stoi(token);
        if (std::getline(ss, token, '_')) coord.y = std::stoi(token);
        if (std::getline(ss, token, '_')) coord.z = std::stoi(token);

        return coord;
    }

    std::string ChunkManager::chunkKeyToChunkPath(ChunkKey& key)
    {
        std::filesystem::path path = folder_pth_;
        std::string key_file_path = key + ".chunk";
        path /= key_file_path;
        return path.string();
    }

    bool ChunkManager::chunkFileExists(ChunkKey& key)
    {
        std::string path_str = chunkKeyToChunkPath(key);
        std::filesystem::path pathtocheck = path_str;
        return std::filesystem::exists(pathtocheck);
    }

    void ChunkManager::removeChunkFile(ChunkKey& key)
    {
        std::string path_str = chunkKeyToChunkPath(key);
        std::filesystem::remove(path_str);
    }

    void ChunkManager::markClean(size_t idx)
    {
        if (idx >= CACHE_SIZE)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rolling-map-node"),"Invalid Mark Clean ID: " 
            << idx << " CACHE_SIZE: " << CACHE_SIZE);
            return;
        }
        chunks_metadata_[idx].state = ChunkState::CLEAN;
    }

    void ChunkManager::markDirty(size_t idx)
    {
        if (idx >= CACHE_SIZE)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rolling-map-node"),"Invalid Mark Clean ID: " 
            << idx << " CACHE_SIZE: " << CACHE_SIZE);
            return;
        }
        chunks_metadata_[idx].state = ChunkState::DIRTY;
    }

    void ChunkManager::markUnset(size_t idx)
    {
        if (idx >= CACHE_SIZE)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rolling-map-node"),"Invalid Mark Clean ID: " 
            << idx << " CACHE_SIZE: " << CACHE_SIZE);
            return;
        }
        chunks_metadata_[idx].state = ChunkState::UNSET;
    }

    ChunkManager::ChunkState& ChunkManager::getChunkState(size_t idx)
    {
        if (idx >= CACHE_SIZE)
        {
            std::cout << "Invalid Mark Unset ID" << std::endl;
            throw(std::runtime_error("Invalid Mark Unset ID"));
        }
        return chunks_metadata_.at(idx).state;
    }

    void ChunkManager::rqPush(ChunkKey &key, size_t cache_index)
    {
        {
            std::lock_guard<std::mutex> lock(read_mutex_);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rolling-map-node"),"Push Request to read key: " << key << " CacheIdx: "<< cache_index);
            read_queue_.emplace(key,cache_index);
            rq_size_ ++;
        }
        read_cv_.notify_one();
    }

    void ChunkManager::wqPush(ChunkKey &key, MapPtr mptr)
    {
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_queue_.emplace(key,mptr);
            wq_size_ ++;
        }
        write_cv_.notify_one();
    }

    void ChunkManager::readWorker()
    {        
        while (true)
        {
            std::pair<ChunkKey,size_t> pair2read;
            {
                //lock and pop from the read queue
                std::unique_lock lock(read_mutex_);
                read_cv_.wait(lock, [&]() { return !read_queue_.empty() || !io_running_; });
                if (!io_running_) break;
                pair2read = read_queue_.front();
            }
            //Extract the key and the target index where this new map will need to be inserted into
            auto& [key2read,target_idx] = pair2read;
            
            //Get the chunk path
            auto chunk_path = this->chunkKeyToChunkPath(key2read);
            //Check if the chunk path exists
            bool path_exists = chunkFileExists(key2read);
            
            //Initialise new map
            MapPtr new_map = nullptr;
            //Deserialize chunk
            if (std::ifstream infile{chunk_path,std::ios::binary};path_exists && infile.is_open())
            {
                //deserialize and load grid
                char header[256];
                infile.getline(header, 256);
                Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
                auto loaded_grid = Bonxai::Deserialize<Bonxai::MapUtils::CellOcc>(infile,info);
                
                //Create new map using the constructor that a
                new_map = std::make_shared<Bonxai::OccupancyMap>(moption_,std::move(loaded_grid));
                
                infile.close();
            }
            else
            {
                //just create a fresh new map using resolutions
                new_map = std::make_shared<Bonxai::OccupancyMap>(map_params_.resolution,moption_);
            }

            if (chunks_metadata_[target_idx].state == ChunkState::UNSET)
            {
                chunks_[target_idx] = new_map;
                chunks_metadata_[target_idx].state = ChunkState::CLEAN;
            }

            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rolling-map-node"),"Something is screwed up with read task for key: " << key2read);
            }

            this->read_queue_.pop();
            this->rq_size_ --;
            map_live_[target_idx] = true;
        }
        return;
    }

    void ChunkManager::writeWorker()
    {
        while (true)
        {
            std::pair<ChunkKey,MapPtr> pair2write;
            {
                std::unique_lock ulock(write_mutex_);
                write_cv_.wait(ulock, [&]() { return !write_queue_.empty() || !io_running_; });
                if (!io_running_) break;
                pair2write = write_queue_.front();
            }

            //Get the key to write and the mapptr
            auto &[key2write,map2write] = pair2write;
            auto chunk_path = this->chunkKeyToChunkPath(key2write);
            bool path_exists = chunkFileExists(key2write);

            //Nuke any copy of the chunk file if it currently exists
            if (path_exists) {removeChunkFile(key2write);}
            std::ofstream outfile(chunk_path,std::ios::binary);
            //Serialize
            Bonxai::Serialize(outfile,map2write->getGrid());

            //Destroy the map. It will go out of scope
            map2write.reset();
            this->write_queue_.pop();
            this->wq_size_ --;
        }
        return;
    }

    bool ChunkManager::hasPendingIO()
    {
        bool all_maps_live = false;
        {
            std::lock_guard<std::mutex> l_lock(liveliness_mutex_);
            
            //If the liveliness array is not all true, this means that IO is going on
            all_maps_live = std::all_of(map_live_.begin(),
                                             map_live_.end(),
                                             [](bool b){return b;});
          
        }
        return !all_maps_live;  

    }

    bool ChunkManager::is26neibor(const Bonxai::CoordT& chunk_origin,const Bonxai::CoordT& chunk_coordinate)
    {
        auto delta = chunk_coordinate - chunk_origin;

        return (std::abs(delta.x)<=1 &&
                std::abs(delta.y)<=1 &&
                std::abs(delta.z)<=1 && !(delta.x==0 && delta.y==0 && delta.z==0));
    }
}
