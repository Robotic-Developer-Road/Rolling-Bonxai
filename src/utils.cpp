#include "rolling_map/utils.h"

namespace RM
{
    
    namespace utils
    {
        std::array<Bonxai::CoordT,6> getFaceNeibors(const Bonxai::CoordT& coord)
        {
            //Follow the right-handle rule system X(front),Y(Left), Z(Up)
            return {{
                {coord.x + 1, coord.y, coord.z}, //front
                {coord.x - 1, coord.y, coord.z}, //back
                {coord.x, coord.y + 1, coord.z}, //left
                {coord.x, coord.y - 1, coord.z}, //right
                {coord.x, coord.y, coord.z + 1}, //top
                {coord.x, coord.y, coord.z - 1}  //down
            }};
        }

        std::array<Bonxai::CoordT,12> getEdgeNeibors(const Bonxai::CoordT& coord)
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
                {coord.x - 1, coord.y, coord.z + 1}, //back-top
                {coord.x + 1, coord.y, coord.z - 1}, //front-down
                {coord.x + 1, coord.y, coord.z + 1}, //front-top

                // yz-plane edges
                {coord.x, coord.y - 1, coord.z - 1}, //right-down
                {coord.x, coord.y - 1, coord.z + 1}, //right-top
                {coord.x, coord.y + 1, coord.z - 1}, //left-down
                {coord.x, coord.y + 1, coord.z + 1}  //left-top
            }};
        }

        std::array<Bonxai::CoordT,8> getCornerNeibors(const Bonxai::CoordT& coord)
        {
            //Follow the right-handle rule system X(front),Y(Left), Z(Up)
            return {{
                {coord.x - 1, coord.y - 1, coord.z - 1}, //back  - right  -  down
                {coord.x - 1, coord.y - 1, coord.z + 1}, //back  - right  -  top
                {coord.x - 1, coord.y + 1, coord.z - 1}, //back  - left   -  down
                {coord.x - 1, coord.y + 1, coord.z + 1}, //back  - left  -   top
                {coord.x + 1, coord.y - 1, coord.z - 1}, //front - right  -  down
                {coord.x + 1, coord.y - 1, coord.z + 1}, //front - right  -  top
                {coord.x + 1, coord.y + 1, coord.z - 1}, //front - left   -  down
                {coord.x + 1, coord.y + 1, coord.z + 1}  //front - left   -  top
            }};
        }

        Bonxai::CoordT mapPointToVoxelCoord(const PCLPoint& point,double resolution)
        {
            double inv_resolution = 1.0 / resolution;
            return {
                static_cast<int32_t>(std::floor(point.x * inv_resolution)),
                static_cast<int32_t>(std::floor(point.y * inv_resolution)),
                static_cast<int32_t>(std::floor(point.z * inv_resolution))};
        }

        PCLPoint voxelCoordToMapPoint(const Bonxai::CoordT& coord,double resolution)
        {
            return 
            {
            (static_cast<double>(coord.x)) * resolution,
            (static_cast<double>(coord.y)) * resolution,
            (static_cast<double>(coord.z)) * resolution};

        }

        PCLPoint voxelCoordToMapCenterPoint(const Bonxai::CoordT& coord, double resolution)
        {
            double half = 0.5 * resolution;
            return {
            (static_cast<double>(coord.x)) * resolution + half,
            (static_cast<double>(coord.y)) * resolution + half,
            (static_cast<double>(coord.z)) * resolution + half};
        }

        Bonxai::CoordT voxelCoordToChunkCoord(const Bonxai::CoordT& coord,int32_t chunk_dim)
        {

            return {
            coord.x >= 0 ? coord.x / chunk_dim : (coord.x - chunk_dim + 1) / chunk_dim,
            coord.y >= 0 ? coord.y / chunk_dim : (coord.y - chunk_dim + 1) / chunk_dim,
            coord.z >= 0 ? coord.z / chunk_dim : (coord.z - chunk_dim + 1) / chunk_dim
            };

        }

        Bonxai::CoordT chunkCoordToVoxelCoord(const Bonxai::CoordT& coord,int32_t chunk_dim)
        {
            return {
                coord.x * chunk_dim,
                coord.y * chunk_dim,
                coord.z * chunk_dim
            };

        }

        Bonxai::CoordT mapFramePointToChunkFrameCoord(const PCLPoint& mP,double resolution,int32_t chunk_dim)
        {
            using Coord = Bonxai::CoordT;

            // Convert the Map Frame Point mP to Map Frame Voxel Coordinate mVC
            Coord mVC = mapPointToVoxelCoord(mP,resolution);

            // Find the Coordinate of the Chunk mCC
            Coord mCC = voxelCoordToChunkCoord(mVC,chunk_dim);

            // Get the Origin Voxel in the Chunk as a Voxel Coordinate in map frame
            Coord mOriginVC = chunkCoordToVoxelCoord(mCC,chunk_dim);

            // And Finally, make the mVC we calculated earlier relative to mOriginVC
            Coord cVC = mVC - mOriginVC;

            return cVC;
        }
        
        PCLPoint mapFramePointToChunkFramePoint(const PCLPoint& mp,double resolution, int32_t chunk_dim)
        {
            auto mVC = mapPointToVoxelCoord(mp,resolution);
            auto mCC = voxelCoordToChunkCoord(mVC,chunk_dim);
            auto mCC_Voxel = chunkCoordToVoxelCoord(mCC,chunk_dim);
            PCLPoint chunk_origin = voxelCoordToMapPoint(mCC_Voxel,resolution);

            PCLPoint new_pt(mp.x - chunk_origin.x,
                            mp.y - chunk_origin.y,
                            mp.z - chunk_origin.z);
            
            return new_pt;
        }

    }
}