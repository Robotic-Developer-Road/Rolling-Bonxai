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

        std::pair<ChunkKey,ChunkState> def = std::make_pair("_",ChunkState::UNSET);
        chunk_states_.fill(def);

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

        std::pair<ChunkKey,ChunkState> def = std::make_pair("_",ChunkState::UNSET);
        chunk_states_.fill(def);
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

        std::cout << "ChunkManager destructor called" << std::endl;
        return;
    }

    void ChunkManager::setSourceCoord(Bonxai::CoordT& source)
    {
        current_source_coord_ = source;
    }

    void ChunkManager::initFirstChunks(Bonxai::CoordT &source_chunk,
                                       std::vector<Bonxai::CoordT> &nb_chunks)
    {
        setSourceCoord(source_chunk);
        //First one is always the source. Get the key
        ChunkKey source_key = chunkCoordToChunkKey(source_chunk);
        //Allocate the map
        MapPtr source_map = std::make_shared<Bonxai::OccupancyMap>(map_params_.resolution,moption_);
        //Add the new map to the chunks
        chunks_[0] = source_map;
        //Update the chunk states
        chunk_states_[0] = std::make_pair(source_key,ChunkState::CLEAN);

        for (auto &c : nb_chunks)
        {
            // Build a fresh new map
            MapPtr new_map = std::make_shared<Bonxai::OccupancyMap>(map_params_.resolution,moption_);
            
            //Grab the chunk type
            ChunkType chunk_type = getChunkType(source_chunk,c);
            //Convert to idx
            size_t chunk_idx = chunkTypeToIndex(chunk_type);
            //Get the chunk key
            ChunkKey k = chunkCoordToChunkKey(c);

            chunks_[chunk_idx] = new_map;
            chunk_states_[chunk_idx] = std::make_pair(k,ChunkState::CLEAN);
        }
        //Set the touched once array to all false
        touched_once_.fill(false);
        //We are ready to go.
        is_init_ = true;
    }

    void ChunkManager::updateChunks(PCLPointCloud &points,PCLPoint& origin,
                          std::vector<Bonxai::CoordT> &nb_chunks)
    {
        auto source_voxel = mapPointToVoxelCoord(origin);
        auto source_chunk = voxelCoordToChunkCoord(source_voxel);

        //Check if there are any read operations in progress
        if (isReadIORunning() || isWriteIORunning())
        {
            //The entire cache is in a transient state so we return right away
            return;
        }

        //Check the current center coord
        bool is_same_source = (current_source_coord_ == source_chunk);

        //If the source has changed, this means that 
        if (!is_same_source)
        {
            //Update the occupancy
            updateAllOccupancy(points,origin,source_chunk);
            //Update the cache
            updateCache(source_chunk,nb_chunks);
        }

        else
        {
            //Just update all the occupancies
            updateAllOccupancy(points,origin,source_chunk);
        }
    }

    void ChunkManager::updateAllOccupancy(PCLPointCloud &points,PCLPoint& origin,
                                          Bonxai::CoordT& source_chunk)
    {
        using V3D = Bonxai::OccupancyMap::Vector3D;
        //Vectors to store hit or miss voxel coords
        std::vector<Bonxai::CoordT> end_point_voxels_map;
        Bonxai::CoordT sensor_voxel = mapPointToVoxelCoord(origin);
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
                Bonxai::CoordT hit_voxel = mapPointToVoxelCoord(p_map);
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
                Bonxai::CoordT missed_voxel = mapPointToVoxelCoord(new_to_point);
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
        Bonxai::CoordT parent_chunk = voxelCoordToChunkCoord(hit_voxel);
        //
        if (is26neibor(source_chunk,parent_chunk))
        {
            //Voxel Coordinate of parent chunk
            Bonxai::CoordT parent_chunk_voxel_origin = chunkCoordToVoxelCoord(parent_chunk);
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
            this->touched_once_[target_index] = true;
        }
    }

    void ChunkManager::updateAllMissPoints(Bonxai::CoordT& miss_voxel,Bonxai::CoordT& source_chunk)
    {
        //Get the parent chunk
        Bonxai::CoordT parent_chunk = voxelCoordToChunkCoord(miss_voxel);
        
        if (is26neibor(source_chunk,parent_chunk))
        {
            //Voxel Coordinate of parent chunk
            Bonxai::CoordT parent_chunk_voxel_origin = chunkCoordToVoxelCoord(parent_chunk);
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
            this->touched_once_[target_index] = true;
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
            Bonxai::CoordT parent_chunk = voxelCoordToChunkCoord(coord);
            if (is26neibor(source_chunk,parent_chunk))
            {
                //Voxel Coordinate of parent chunk
                Bonxai::CoordT parent_chunk_voxel_origin = chunkCoordToVoxelCoord(parent_chunk);
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
                this->touched_once_[target_index] = true;
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
        for (size_t i = 0 ; i < this->touched_once_.size() ; ++i)
        {
            if (touched_once_.at(i))
            {
                this->chunks_[i]->incrementUpdateCount();
            }
        }

        touched_once_.fill(false);
    }

    void ChunkManager::getAllOccupiedVoxels(PCLPointCloud& points)
    {
        if (isReadIORunning() || isWriteIORunning())
        {
            return;
        }
        points.clear();

        for (int i = 0 ; i < chunks_.size() ; ++i)
        {
            auto key = chunk_states_[i].first;
            Bonxai::CoordT cc = this->chunkKeyToChunkCoord(key);
            Bonxai::CoordT cc_origin_voxel_map = chunkCoordToVoxelCoord(cc);

            auto visitor = [this,&cc_origin_voxel_map,&points](Bonxai::MapUtils::CellOcc &cell,const Bonxai::CoordT& coord)
            {
                if (cell.probability_log > moption_.occupancy_threshold_log)
                {
                    Bonxai::CoordT map_voxel = coord + cc_origin_voxel_map;
                    auto pcpt = this->voxelCoordToMapPoint(map_voxel);
                    points.push_back(pcpt);
                }

            };

            chunks_[i] -> getGrid().forEachCell(visitor); 
        }
    }

    void ChunkManager::getAllFreeVoxels(PCLPointCloud& points)
    {
        if (isReadIORunning() || isWriteIORunning())
        {
            return;
        }

        points.clear();

        for (int i = 0 ; i < chunks_.size() ; ++i)
        {
            auto key = chunk_states_[i].first;
            Bonxai::CoordT cc = this->chunkKeyToChunkCoord(key);
            Bonxai::CoordT cc_origin_voxel_map = chunkCoordToVoxelCoord(cc);

            auto visitor = [this,&cc_origin_voxel_map,&points](Bonxai::MapUtils::CellOcc &cell,const Bonxai::CoordT& coord)
            {
                if (cell.probability_log < moption_.occupancy_threshold_log)
                {
                    Bonxai::CoordT map_voxel = coord + cc_origin_voxel_map;
                    auto pcpt = this->voxelCoordToMapPoint(map_voxel);
                    points.push_back(pcpt);
                }
            };
            chunks_[i] -> getGrid().forEachCell(visitor); 
        }
        
    }

    void ChunkManager::updateCache(Bonxai::CoordT& source_chunk,std::vector<Bonxai::CoordT> &nb_chunks)
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

        Workflow: Take out chunks that need to be evicted, replacing them will nullptrs
         */

        //A lambda to zero out a chunk idx
        auto makeEmptyLambda = [this](size_t i)
        {
            this->chunks_[i] = nullptr;
            chunk_states_[i] = std::make_pair("_",ChunkState::UNSET);
        }; //makeEmptyLambda

        //Set the new source chunk
        setSourceCoord(source_chunk);

        std::vector<std::tuple<ChunkKey,ChunkState,MapPtr>> persist_tasks;

        for (size_t i = 0 ; i < chunks_.size() ; ++i)
        {
            ChunkKey key = chunk_states_[i].first;
            ChunkState state = chunk_states_[i].second;

            Bonxai::CoordT coord = chunkKeyToChunkCoord(key);

            //If the coordinate is also the source chunk, we persist
            if (coord == source_chunk)
            {
                auto mptr = chunks_[i];
                persist_tasks.emplace_back(key,state,mptr);
                makeEmptyLambda(i);
                continue;
            }
            
            //A chunk coordinate in the cache is not a neibor of the source chunk, so we must evict
            if (!is26neibor(source_chunk,coord))
            {
                //We need to save it if its dirty
                if (state == ChunkState::DIRTY)
                {
                    //Copy of the mptr
                    auto mptr = chunks_[i];
                    //zero out the chunk idx
                    makeEmptyLambda(i);
                    //push his chunk into the write queue
                    wqPush(key,mptr);
                }
                else
                {
                    //its not dirty so nuke it
                    makeEmptyLambda(i);
                }

            }
            else //if it is a 26 nb, we need to save it
            {
                //If a chunk in the cache is a neibor of the new source chunk
                auto mptr = chunks_[i];
                persist_tasks.emplace_back(key,state,mptr);
                makeEmptyLambda(i);
            }
        }

        //Put the persists back into the cache
        for (auto [key,state,mptr] : persist_tasks)
        {
            Bonxai::CoordT persist_coord = chunkKeyToChunkCoord(key);
            ChunkType ctype = getChunkType(source_chunk,persist_coord);
            size_t idx = chunkTypeToIndex(ctype);

            chunks_[idx] = mptr;
            chunk_states_[idx] = std::make_pair(key,state);
        }

        for (size_t i = 0 ; i < chunks_.size() ; i++)
        {
            if (!chunks_[i])
            {
                auto coord = nb_chunks[i];
                auto key = chunkCoordToChunkKey(coord);
                rqPush(key,coord);
            }
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
        if (idx >= chunk_states_.max_size())
        {
            std::cout << "Invalid Mark Clean ID" << std::endl;
            return;
        }

        chunk_states_[idx].second = ChunkState::CLEAN;
    }

    void ChunkManager::markDirty(size_t idx)
    {
        if (idx >= chunk_states_.max_size())
        {
            std::cout << "Invalid Mark Dirty ID" << std::endl;
            return;
        }

        chunk_states_[idx].second = ChunkState::DIRTY;
    }

    void ChunkManager::markUnset(size_t idx)
    {
        if (idx >= chunk_states_.max_size())
        {
            std::cout << "Invalid Mark Unset ID" << std::endl;
            return;
        }

        chunk_states_[idx].first = "_";
        chunk_states_[idx].second = ChunkState::UNSET;
    }

    ChunkManager::ChunkState& ChunkManager::getChunkState(size_t idx)
    {
        if (idx >= chunk_states_.max_size())
        {
            std::cout << "Invalid Mark Unset ID" << std::endl;
            throw(std::runtime_error("Invalid Mark Unset ID"));
        }
        return chunk_states_.at(idx).second;
    }

    void ChunkManager::rqPush(ChunkKey &key, Bonxai::CoordT &coord)
    {
        {
            std::lock_guard<std::mutex> lock(read_mutex_);
            read_queue_.emplace(key,coord);
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
        //Set the read io running flag
        this->read_io_running_ = true;
        
        while (true)
        {
            std::pair<ChunkKey,Bonxai::CoordT> pair2read;
            {
                //lock and pop from the read queue
                std::unique_lock lock(read_mutex_);
                read_cv_.wait(lock, [&]() { return !read_queue_.empty() || !io_running_; });
                if (!io_running_) break;
                auto pair2read = read_queue_.front();
                read_queue_.pop();
                rq_size_ --;
            }
            auto [key2read,coord2read] = pair2read;
            ChunkType new_chunk_type = getChunkType(current_source_coord_,coord2read);
            size_t target_idx = chunkTypeToIndex(new_chunk_type);

            auto chunk_path = this->chunkKeyToChunkPath(key2read);
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

            if (!chunks_[target_idx] && chunk_states_[target_idx].second == ChunkState::UNSET)
            {
                chunks_[target_idx] = new_map;
                ChunkKey k = chunkCoordToChunkKey(coord2read);
                chunk_states_[target_idx]= std::make_pair(k,ChunkState::CLEAN);
            }

            else
            {
                std::cout << "Tring to read into chunk that already exists" << std::endl;
            }
        }
        //Kill read io
        read_io_running_ = false;
        return;
    }

    void ChunkManager::writeWorker()
    {
        write_io_running_ = true;

        while (true)
        {
            std::pair<ChunkKey,MapPtr> pair2write;
            {
                std::unique_lock ulock(write_mutex_);
                write_cv_.wait(ulock, [&]() { return !write_queue_.empty() || !io_running_; });
                if (!io_running_) break;
                pair2write = write_queue_.front();
                write_queue_.pop();
                wq_size_ --;
            }

            //Get the key to write and the mapptr
            auto [key2write,map2write] = pair2write;
            auto chunk_path = this->chunkKeyToChunkPath(key2write);
            bool path_exists = chunkFileExists(key2write);

            //Nuke any copy of the chunk file if it currently exists
            if (path_exists) {removeChunkFile(key2write);}
            std::ofstream outfile(chunk_path,std::ios::binary);
            //Serialize
            Bonxai::Serialize(outfile,map2write->getGrid());

            //Destroy the map. It will go out of scope
            map2write.reset();
        }
        write_io_running_ = false;
        return;
    }

    bool ChunkManager::isReadIORunning() const
    {
        return read_io_running_;
    }

    bool ChunkManager::isWriteIORunning() const
    {
        return write_io_running_;
    }

    bool ChunkManager::is26neibor(const Bonxai::CoordT& chunk_origin,const Bonxai::CoordT& chunk_coordinate)
    {
        auto delta = chunk_coordinate - chunk_origin;

        return (std::abs(delta.x)<=1 &&
                std::abs(delta.y)<=1 &&
                std::abs(delta.z)<=1 && !(delta.x==0 && delta.y==0 && delta.z==0));
    }

    Bonxai::CoordT ChunkManager::mapPointToVoxelCoord(const PCLPoint& point)
    {
        double inv_resolution = 1.0 / map_params_.resolution;
        return {
            static_cast<int32_t>(std::floor(point.x * inv_resolution)),
            static_cast<int32_t>(std::floor(point.y * inv_resolution)),
            static_cast<int32_t>(std::floor(point.z * inv_resolution))};
    }

    ChunkManager::PCLPoint ChunkManager::voxelCoordToMapPoint(const Bonxai::CoordT& coord)
    {
        double res = map_params_.resolution;
        return {
            (static_cast<double>(coord.x)) * res,
            (static_cast<double>(coord.y)) * res,
            (static_cast<double>(coord.z)) * res};
    }

    ChunkManager::PCLPoint ChunkManager::voxelCoordToMapCenterPoint(const Bonxai::CoordT& coord)
    {   
        double res = map_params_.resolution;
        double half = 0.5 * map_params_.resolution;
        return {
            (static_cast<double>(coord.x)) * res + half,
            (static_cast<double>(coord.y)) * res + half,
            (static_cast<double>(coord.z)) * res + half};
    }

    Bonxai::CoordT ChunkManager::voxelCoordToChunkCoord(const Bonxai::CoordT& vc)
    {
        const int32_t chunk_size = chunk_params_.chunk_dim;

        return {
        vc.x >= 0 ? vc.x / chunk_size : (vc.x - chunk_size + 1) / chunk_size,
        vc.y >= 0 ? vc.y / chunk_size : (vc.y - chunk_size + 1) / chunk_size,
        vc.z >= 0 ? vc.z / chunk_size : (vc.z - chunk_size + 1) / chunk_size
        };
    }

    Bonxai::CoordT ChunkManager::chunkCoordToVoxelCoord(const Bonxai::CoordT& cc)
    {
        const int32_t chunk_size = chunk_params_.chunk_dim;
        return {
            cc.x * chunk_size,
            cc.y * chunk_size,
            cc.z * chunk_size
        };
    }

    Bonxai::CoordT ChunkManager::mapFramePointToChunkFrameCoord(const PCLPoint& mP)
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

    ChunkManager::PCLPoint ChunkManager::mapFramePointToChunkFramePoint(const PCLPoint& mp)
    {
        auto mVC = mapPointToVoxelCoord(mp);
        auto mCC = voxelCoordToChunkCoord(mVC);
        auto mCC_Voxel = chunkCoordToVoxelCoord(mCC);
        PCLPoint chunk_origin = voxelCoordToMapPoint(mCC_Voxel);

        PCLPoint new_pt(mp.x - chunk_origin.x,
                        mp.y - chunk_origin.y,
                        mp.z - chunk_origin.z);
        
        return new_pt;
    }
}
