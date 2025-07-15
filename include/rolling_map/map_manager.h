#ifndef ROLLING_MAP__MAP_MANAGER_H
#define ROLLING_MAP__MAP_MANAGER_H

#include "rclcpp/rclcpp.hpp"

#include "bonxai_map/occupancy_map.hpp"
#include "rolling_map/chunk_manager.h"
#include "rolling_map/map_params.h"
#include "rolling_map/utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>
#include <array>
#include <utility>

namespace RM
{
    class MapManager
    {
    public:
        using PCLPoint = pcl::PointXYZ;
        using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
        using PCLPointCloudSharedPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
        using ChunkKey = std::string;
        using MapPtr = std::shared_ptr<Bonxai::OccupancyMap>;

        /**
         * @brief Constructor for Map Manager class.
         * @param RM::SensorParams sensor_params
         * @param RM::MapParams map_params
         * @param RM::ChunkParams chunk_params
         */
        MapManager(SensorParams& sensor_params,MapParams& map_params,ChunkParams& chunk_params);
        MapManager();

        /**
         * @brief Update function called by some cloud callback to update the entire map
         * @param PCLPointCloud& points
         * @param PCLPoint& origin
         * @return bool
         */
        bool updateMap(PCLPointCloud& points,PCLPoint& origin);

        /**
         * @brief Get the free voxels in the map
         * @param std::vector<PCLPoint>& points
         * @return void
         */
        void getFreeVoxels(PCLPointCloud& points);

        /**
         * @brief Get the occupied voxels in the map
         * @param std::vector<PCLPoint>& points
         * @return void
         */
        void getOccupiedVoxels(PCLPointCloud& points);

        /**
         * @brief Get the chunk metadata
         * @param std::array<pair<PCLPoint,uint8_t>,27>& metadata
         * @return void
         */
        void getChunkMetadata(std::array<std::pair<PCLPoint,uint8_t>,27>& metadata);

    private:

        //Chunk Manager
        std::unique_ptr<ChunkManager> chunk_manager_ {nullptr};

        //parameters
        SensorParams sp_;
        MapParams mp_;
        ChunkParams cp_;
        double inv_resolution_;

        //Check update status
        bool first_update_ {true};

    };

} //namespace RM
#endif // ROLLING_MAP__MAP_MANAGER_H