#ifndef CHUNK_MANAGER_H
#define CHUNK_MANAGER_H

#include "bonxai_map/occupancy_map.hpp"
#include "rolling_map/rolling_map_params.h"
#include "bonxai_core/serialization.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <filesystem>
#include <algorithm>
#include <unordered_map>

namespace RM
{
    /*
    The ChunkManager receives a set of update instructions from the MapManager. It uses the
    pointcloud it receives and passes it to the appropriate chunk maps so that they can do their
    occupancy grid updates. After updating, the chunk manager decides what chunks to evict and
    what chunks to read from storage. The key idea is that the ChunkManager will try to update
    as much as possible first before deciding to do the evictions/loading.
    
    ChunkManager can hold up to 27 map chunks. The order of these chunks are strictly defined. In general,
    the ordering of a Chunk and its neibors will be clearly defined by the ChunkType enum class. The 
    underlying size_t maps each enum to a corresponding integer. This integer defines the key where that
    map can be found. As a result, the source coordinate, which is the "center", defines how unique 
    the set of up to 27 cached chunks are. The deltas of each neibor index will be calculated based on the 
    source. Deltas map to Enums, Enums map to Indices.

    ChunkManager recieves a set of up to 27 chunk indices that are in play for a particular frame, and the
    complete set of PointClouds and the source chunk. First, it checks if there are any IO operations currently
    running. This would skip this frame right away and terminate, because the cache is in a transitory state
    */
    class ChunkManager
    {
    public:
        using PCLPoint = pcl::PointXYZ;
        using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
        using PCLPointCloudSharedPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
        using ChunkKey = std::string;
        using MapPtr = std::shared_ptr<Bonxai::OccupancyMap>;
        using MapAccessor = Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>::Accessor;
        /**
         * @brief Constructor for Chunk Manager
         * @param SensorParams sensor_params
         * @param MapParams map_params
         * @param ChunkParams chunk_params
         */
        explicit ChunkManager(SensorParams& sensor_params,MapParams& map_params,ChunkParams& chunk_params);
        ChunkManager();

        /**
         * @brief Destructor for Chunk Manager
         */
        ~ChunkManager();

        ChunkManager(const ChunkManager&) = default;
        ChunkManager(ChunkManager&&) = default;
        ChunkManager& operator=(const ChunkManager&) = default;
        ChunkManager& operator=(ChunkManager&&) = default;

        void initFirstChunks(Bonxai::CoordT &source_chunk,
                             std::vector<Bonxai::CoordT> &chunks_in_play);
        
        void updateChunks(PCLPointCloud &points,PCLPoint &origin,
                          std::vector<Bonxai::CoordT> &chunks_in_play);


    private:
        enum class ChunkType : size_t
        {
            SOURCE = 0,        //+0,+0,+0
            FACE_F = 1,        //1,0,0
            FACE_B = 2,        //-1,0,0
            FACE_L = 3,        //0,1,0
            FACE_R = 4,        //0,-1,0
            FACE_T = 5,        //0,0,1
            FACE_D = 6,        //0,0,-1
            EDGE_B_R = 7,      //-1,-1,0
            EDGE_B_L = 8,      //-1,1,0
            EDGE_F_R = 9,      //1,-1,0
            EDGE_F_L = 10,     //1,1,0
            EDGE_B_D = 11,     //-1,0,-1
            EDGE_B_T = 12,     //-1,0,1
            EDGE_F_D = 13,     //1,0,-1
            EDGE_F_T = 14,     //1,0,1
            EDGE_R_D = 15,     //0,-1,-1
            EDGE_R_T = 16,     //0,-1,1
            EDGE_L_D = 17,     //0,1,-1
            EDGE_L_T = 18,     //0,1,1
            CORNER_B_R_D = 19, //-1,-1,-1
            CORNER_B_R_T = 20, //-1,-1,1
            CORNER_B_L_D = 21, //-1,1,-1
            CORNER_B_L_T = 22, //-1,1,1
            CORNER_F_R_D = 23, // 1,-1,-1
            CORNER_F_R_T = 24, // 1,-1,1
            CORNER_F_L_D = 25, // 1,1,-1
            CORNER_F_L_T = 26, // 1,1,1
            INVALID = 27
        };

        enum class ChunkState : size_t
        {
            CLEAN = 0,
            DIRTY = 1,
            UNSET = 2
        };

        /**
         * @brief Set the center coord
         * @param Bonxai::CoordT& source
         */
        void setSourceCoord(Bonxai::CoordT& source);
  
        /**
         * @brief Update chunks in a full manner when the centers are the same
         * @param PCLPointCloud& points
         * @param Bonxai::CoordT& source_chunk
         * @param std::vector<Bonxai::CoordT>& chunks_in_play
         */
        void updateAllOccupancy(PCLPointCloud &points,PCLPoint& origin,
                            Bonxai::CoordT& source_chunk);

        /**
         * @brief Update the end points of the laser scan as hit
         * @param hit_voxel in map frame
         * @param source chunk
         */
        void updateAllHitPoints(Bonxai::CoordT& hit_voxel,Bonxai::CoordT& source_chunk);

        /**
         * @brief Update the end points of the laser scan as miss
         * @param hit_voxel in map frame
         * @param source chunk
         */
        void updateAllMissPoints(Bonxai::CoordT& miss_voxel,Bonxai::CoordT& source_chunk);

        /**
         * @brief Update the end points of the laser scan as miss
         * @param hit_voxel in map frame
         * @param source chunk
         * @param bool is_hit, true if the update is for a hit point, false if miss point
         */
        void updateAllHitOrMissPoints(Bonxai::CoordT& target_voxel,Bonxai::CoordT& source_chunk,bool is_hit);

        /**
         * @brief Raycast from start to end and update all the free cells
         * @param std::vector<CoordT> end_point_voxels
         * @param CoordT sensor voxel
         */
        void updateAllFreeCells(std::vector<Bonxai::CoordT> &end_voxels, Bonxai::CoordT& source_chunk,Bonxai::CoordT &sensor_voxel);
        
        /**
         * @brief Increment the update count of all the chunks that were touched
         * 
         */
        void incrementUpdateCount();

        /**
         * @brief Update the cache of chunks
         */
        void updateCache();

        /**
         * @brief Get the chunk type
         * @param Bonxai::CoordT& source
         * @param Bonxai::CoordT& c
         * @return ChunkType
         */
        ChunkType getChunkType(const Bonxai::CoordT& source,const Bonxai::CoordT& c);

        /**
         * @brief Get the index of the chunk type
         * @param ChunkType Enum Class type 
         * @return size_t
         */
        size_t chunkTypeToIndex(ChunkType type);
        
        /**
         * @brief Get the chunk key
         * @param Bonxai::CoordT& cc
         * @return ChunkKey aka string
         */
        ChunkKey chunkCoordToChunkKey(const Bonxai::CoordT& cc);

        /**
         * @brief Retrieve the chunk coordinate from a chunk key
         * @param ChunkKey& key
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT chunkKeyToChunkCoord(ChunkKey& key);

        /**
         * @brief Get the chunk path
         * @param ChunkKey& key
         * @return std::string
         * */
        std::string chunkKeyToChunkPath(ChunkKey& key);

        /**
         * @brief Check if the chunk file exists
         * @param ChunkKey& key
         * @return bool
         */
        bool chunkFileExists(ChunkKey& key);

        /**
         * @brief Removes file associated with a chunk key's path
         * @param ChunkKey& key
         */
        void removeChunkFile(ChunkKey& key);

        /**
         * @brief Mark the chunk as clean
         * @param size_t idx
         */
        void markClean(size_t idx);

        /**
         * @brief Mark the chunk as dirty
         * @param size_t idx
         */
        void markDirty(size_t idx);

        /**
         * @brief Mark the chunk as unset
         * @param size_t idx
         */
        void markUnset(size_t idx);
        
        /**
         * @brief Retrieve the state of the chunk
         * @param idx of the chunk
         * @return ChunkState of the chunk
         */
        ChunkState& getChunkState(size_t idx);

        /**
         * @brief Push a chunk key to the read queue
         * @param ChunkKey& key
         */
        void rqPush(ChunkKey &key, Bonxai::CoordT &coord);

        /**
         * @brief Push a chunk key to the write queue
         * @param ChunkKey& key
         */
        void wqPush(ChunkKey &key, MapPtr mptr);

        /**
         * @brief Read worker thread that reads from disc and into the cache
         * @details This is the thread that reads from disc and into the cache
         */
        void readWorker();

        /**
         * @brief Write worker thread that writes to disc from the cache
         * @details This is the thread that writes to disc from the cache
         */
        void writeWorker();

        /**
         * @brief Check if the read IO is running
         * @return bool
         */
        bool isReadIORunning() const;

        /**
         * @brief Check if the write IO is running
         * @return bool
         */
        bool isWriteIORunning() const;
        
        /**
         * @brief Check if the chunk is a 26 neighbor
         * @param const Bonxai::CoordT& chunk_origin
         * @param const Bonxai::CoordT& chunk_coordinate
         * @return bool
         */
        bool is26neibor(const Bonxai::CoordT& chunk_origin,const Bonxai::CoordT& chunk_coordinate);


        //Parameters
        SensorParams sensor_params_;
        MapParams map_params_;
        ChunkParams chunk_params_;
        std::string folder_pth_;

        //Map Options
        Bonxai::MapUtils::OccupancyOptions moption_;

        /////////////////
        //Data Structures
        /////////////////

        //27 sized fixed array of map pointers
        std::array<MapPtr,27> chunks_;
        //27 sized fixed array of booleans indicating if the chunk is clean
        std::array<std::pair<ChunkKey,ChunkState>,27> chunk_states_;
        //Boolean values to track if a chunk was touched atleast once during a particular update. It is reset every update
        std::array<bool,27> touched_once_;
        //The center coordinate
        Bonxai::CoordT current_source_coord_;
        //Flag to check if the first map has been initted
        bool is_init_ {false};

        /////////////////
        ////IO Stuff/////
        /////////////////

        //Queue containing the set of map ids to read from the disc  
        std::queue<std::pair<ChunkKey,Bonxai::CoordT>> read_queue_;
        size_t rq_size_ {0};

        //Queue containing the set of maps to write to the disc
        std::queue<std::pair<ChunkKey, MapPtr>> write_queue_;
        size_t wq_size_ {0};

        //Mutexes
        std::mutex read_mutex_, write_mutex_;
        std::condition_variable read_cv_, write_cv_;

        //Atomic Flags
        std::atomic<bool> io_running_{true};
        std::atomic<bool> read_io_running_{false};
        std::atomic<bool> write_io_running_{false};

        //Read thread
        std::thread read_thread_;

        //Write thread
        std::thread write_thread_;

        ///////////////////
        ////Conversion/////
        ///////////////////
        /**
         * @brief Converts a 3D point in map frame to a voxel coord. Exactly the same as Bonxai::posToCoord
         * @param PCLPoint& point
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT mapPointToVoxelCoord(const PCLPoint& point);

        /**
         * @brief Converts a voxel coord to a 3D point in map frame. Exactly the same as Bonxai::coordToPos
         * @param Bonxai::CoordT& coord
         * @return PCLPoint
         */
        PCLPoint voxelCoordToMapPoint(const Bonxai::CoordT& coord);

        /**
         * @brief Convert a voxel coord to a 3D point in the map frame to the center of the voxel
         * @param Bonxai::CoordT& coord
         * @return PCLPoint
         */
        PCLPoint voxelCoordToMapCenterPoint(const Bonxai::CoordT& coord);

        /**
         * @brief COnverts a voxel coord to a chunk coordinate
         * @param Bonxai::CoordT& coord
         * @return CoordT
         */
        Bonxai::CoordT voxelCoordToChunkCoord(const Bonxai::CoordT& coord);

        /**
         * @brief Maps a chunk coordinate to the voxel coordinate, which is the back right down voxel in the map frame
         * @param const Bonxai::CoordT& coord
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT chunkCoordToVoxelCoord(const Bonxai::CoordT& coord);

        /**
         * @brief Maps a 3D point in the map frame to a voxel coordinate that is relative to the voxel origin of the chunk
         * @param const PCLPoint& point
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT mapFramePointToChunkFrameCoord(const PCLPoint& mP);
        
        /**
         * Maps a 3D point from map frame into chunk frame
         */
        PCLPoint mapFramePointToChunkFramePoint(const PCLPoint& mp);
    };
}

#endif // CHUNK_MANAGER_H