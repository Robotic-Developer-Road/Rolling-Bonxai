#ifndef ROLLING_MAP__CHUNK_MANAGER_H
#define ROLLING_MAP__CHUNK_MANAGER_H
#include "rclcpp/rclcpp.hpp"

#include "bonxai_core/serialization.hpp"
#include "bonxai_map/occupancy_map.hpp"
#include "rolling_map/map_params.h"
#include "rolling_map/utils.h"

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

        /**
         * @brief Initialize the first set of chunks
         * @param Bonxai::CoordT &source_chunk
         * @param std::vector<Bonxai::CoordT> &nb_chunks
         */
        void initFirstChunks(Bonxai::CoordT &source_chunk);

        /**
         * @brief Update the chunks
         * @param PCLPointCloud &points
         * @param PCLPoint &origin
         * @param std::vector<Bonxai::CoordT> &nb_chunks
         * @return bool
         */
        bool updateChunks(PCLPointCloud &points,PCLPoint &origin);
        
        /**
         * @brief Get all the occupied voxels
         * @param std::vector<PCLPoint>& points
         */
        void getAllOccupiedVoxels(PCLPointCloud& points);

        /**
         * @brief Get all the free voxels
         * @param std::vector<PCLPoint>& points
         */
        void getAllFreeVoxels(PCLPointCloud& points);

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

        //Chunk Metadata
        struct Chunkmd
        {
            Bonxai::CoordT coord {0,0,0};
            ChunkKey key {"_"};
            ChunkState state {ChunkState::UNSET};
            bool touched_once {false};
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
         * @param Bonxai::CoordT source_chunk
         */
        void updateCache(Bonxai::CoordT& new_source_chunk);

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
         * @brief Get the chunk type given an index
         * @param size_t index
         * @return ChunkType enum
         */
        ChunkType indexToChunkType(size_t index);

        /**
         * @brief Get the chunk coordinate given a chunk type and a source chunk coordinate
         * @param ChunkType type
         * @param Bonxai::CoordT& source
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT getChunkCoord(ChunkManager::ChunkType ctype,const Bonxai::CoordT& source);
        
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
        void rqPush(ChunkKey &key, size_t cache_index);

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
         * @brief Check if there are pending IO tasks running by polling the emptiness of the read and write queues
         * @return bool true if atleast one of the queues is open
         */
        bool hasPendingIO();

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

        const size_t CACHE_SIZE {27};

        //27 sized fixed array of map pointers
        std::array<MapPtr,27> chunks_;
        //27 sized fixed array of chunk metadata structs
        std::array<Chunkmd,27> chunks_metadata_;
        //The center coordinate
        Bonxai::CoordT current_source_coord_;
        //Flag to check if the first map has been initted
        std::array<bool,27> map_live_;

        /////////////////
        ////IO Stuff/////
        /////////////////

        //Queue containing the set of map ids to read from the disc  
        std::queue<std::pair<ChunkKey,size_t>> read_queue_;
        size_t rq_size_ {0};

        //Queue containing the set of maps to write to the disc
        std::queue<std::pair<ChunkKey, MapPtr>> write_queue_;
        size_t wq_size_ {0};

        //Mutexes
        std::mutex read_mutex_, write_mutex_, liveliness_mutex_;
        std::condition_variable read_cv_, write_cv_;

        //Atomic Flags
        std::atomic<bool> io_running_{true};

        //Read thread
        std::thread read_thread_;

        //Write thread
        std::thread write_thread_;
    };
}

#endif // ROLLING_MAP__CHUNK_MANAGER_H