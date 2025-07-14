#ifndef ROLLING_MAP__UTILS_H
#define ROLLING_MAP__UTILS_H

#include "bonxai_core/bonxai.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


namespace RM
{
    
    namespace utils
    {
        using PCLPoint = pcl::PointXYZ;
        using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
        using PCLPointCloudSharedPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


        /**
         * @brief Get the 6 face neibors of a chunk
         * @param Bonxai::CoordT& c
         * @return std::array<Bonxai::CoordT,6>
         */
        std::array<Bonxai::CoordT,6> getFaceNeibors(const Bonxai::CoordT& coord);

        /**
         * @brief Get the 12 edge neibors of a chunk
         * @param Bonxai::CoordT& c
         * @return std::array<Bonxai::CoordT,12>
         */
        std::array<Bonxai::CoordT,12> getEdgeNeibors(const Bonxai::CoordT& coord);

        /**
         * @brief Get the 8 corner neibors of a chunk
         * @param Bonxai::CoordT& c
         * @return std::array<Bonxai::CoordT,8>
         */
        std::array<Bonxai::CoordT,8> getCornerNeibors(const Bonxai::CoordT& coord);

        /**
         * @brief Converts a 3D point in map frame to a voxel coord. Exactly the same as Bonxai::posToCoord
         * @param PCLPoint& point
         * param double resolution
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT mapPointToVoxelCoord(const PCLPoint& point,double resolution);

        /**
         * @brief Converts a voxel coord to a 3D point in map frame. Exactly the same as Bonxai::coordToPos
         * @param Bonxai::CoordT& coord
         * param double resolution
         * @return PCLPoint
         */
        PCLPoint voxelCoordToMapPoint(const Bonxai::CoordT& coord,double resolution);

        /**
         * @brief Convert a voxel coord to a 3D point in the map frame to the center of the voxel
         * @param Bonxai::CoordT& coord
         * @param double resolution
         * @return PCLPoint
         */
        PCLPoint voxelCoordToMapCenterPoint(const Bonxai::CoordT& coord, double resolution);

        /**
         * @brief COnverts a voxel coord to a chunk coordinate
         * @param Bonxai::CoordT& coord
         * @param int32_t chunk_dim. The resolution of a chunk in terms of voxels
         * @return CoordT
         */
        Bonxai::CoordT voxelCoordToChunkCoord(const Bonxai::CoordT& coord,int32_t chunk_dim);

        /**
         * @brief Maps a chunk coordinate to the voxel coordinate, which is the back right down voxel in the map frame
         * @param const Bonxai::CoordT& coord
         * @param int32_t chunk_dim. The resolution of a chunk in terms of voxels
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT chunkCoordToVoxelCoord(const Bonxai::CoordT& coord,int32_t chunk_dim);

        /**
         * @brief Maps a 3D point in the map frame to a voxel coordinate that is relative to the voxel origin of the chunk
         * @param const PCLPoint& point
         * @param double resolution
         * @param int32_t chunk_dim. The resolution of a chunk in terms of voxels
         * @return Bonxai::CoordT
         */
        Bonxai::CoordT mapFramePointToChunkFrameCoord(const PCLPoint& mP,double resolution,int32_t chunk_dim);
        
        /**
         * Maps a 3D point from map frame into chunk frame
         * @param const PCLPoint& mp
         * @param double resolution
         * @param int32_t chunk_dim
         * @return PCLPoint
         */
        PCLPoint mapFramePointToChunkFramePoint(const PCLPoint& mp,double resolution, int32_t chunk_dim);
    }
}

#endif 