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
                                       std::vector<Bonxai::CoordT> &chunks_in_play)
    {
        setSourceCoord(source_chunk);

        for (auto &c : chunks_in_play)
        {
            MapPtr new_map = std::make_shared<Bonxai::OccupancyMap>(map_params_.resolution);
            new_map->setOptions(moption_);
            
            //Grab the chunk type
            ChunkType chunk_type = getChunkType(source_chunk,c);
            //Convert to idx
            size_t chunk_idx = chunkTypeToIndex(chunk_type);
            //Get the chunk key
            ChunkKey k = chunkCoordToChunkKey(c);

            chunks_[chunk_idx] = new_map;
            chunk_states_[chunk_idx] = std::make_pair(k,ChunkState::CLEAN);
        }
        is_init_ = true;
    }

    void ChunkManager::updateChunks(PCLPointCloud &points,PCLPoint& origin,
                          std::vector<Bonxai::CoordT> &chunks_in_play)
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

        if (!is_same_source)
        {
            //update whatever maps you can
            bestEffortChunkUpdate(points,origin,source_chunk,chunks_in_play);

            //Update the cache
            updateCache(source_chunk,chunks_in_play);
        }

        else
        {
            //Just update all the occupancies
            fullChunkUpdate(points,origin,source_chunk,chunks_in_play);
        }
    }

    void ChunkManager::bestEffortChunkUpdate(PCLPointCloud &points,PCLPoint& origin,
                                             Bonxai::CoordT& source_chunk,
                                             std::vector<Bonxai::CoordT> &chunks_in_play)
    {
        //todo
    }

    void ChunkManager::fullChunkUpdate(PCLPointCloud &points,PCLPoint& origin,
                                             Bonxai::CoordT& source_chunk,
                                             std::vector<Bonxai::CoordT> &chunks_in_play)
    {
        //todo
    }

    void ChunkManager::updateCache(Bonxai::CoordT& source_chunk,std::vector<Bonxai::CoordT> &chunks_in_play)
    {
        //todo
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
