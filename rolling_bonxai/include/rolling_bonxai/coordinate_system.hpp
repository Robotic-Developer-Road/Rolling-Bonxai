#pragma once
#include "bonxai_core/bonxai.hpp"
#include "bonxai_map/occupancy_map.hpp"
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

namespace RollingBonxai
{

// For potentially useful stuff using robot velocity later.
using LinearVelocity3D = Eigen::Vector3d;

struct ChunkCoord
{
    int32_t x;
    int32_t y;
    int32_t z;

    /**
     * @brief Default Constructor that initialises all fields to 0
     */
    ChunkCoord();

    /**
     * @brief Default Constructor that initialises all fields to inputs
     * @param x_ Input to x
     * @param y_ Input to y
     * @param z_ Input to z
     */
    ChunkCoord(int32_t x_, int32_t y_, int32_t z_);

    /**
     * @brief == operator
     */
    bool operator==(const ChunkCoord& other) const;

    /**
     * @brief != operator
     */
    bool operator!=(const ChunkCoord& other) const;

    /**
     * @brief + operator
     */
    ChunkCoord operator+(const ChunkCoord& offset) const;

    /**
     * @brief - operator
     */
    ChunkCoord operator-(const ChunkCoord& offset) const;

    /**
     * @brief += operator
     */
    ChunkCoord& operator+=(const ChunkCoord& offset);

    /**
     * @brief -= operator
     */
    ChunkCoord& operator-=(const ChunkCoord& offset);
};

// Ripped this from jippity
/*
https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function#FNV_hash_parameters
algorithm fnv-1a is
    hash := FNV_offset_basis

    for each byte_of_data to be hashed do
        hash := hash XOR byte_of_data
        hash := hash × FNV_prime

    return hash 
*/
struct ChunkCoordHash 
{
    std::size_t operator()(const ChunkCoord& c) const noexcept {
        std::size_t h = 14695981039346656037ull; // FNV-1a 64-bit offset

        //XOR first, then prime for each byte

        h ^= static_cast<std::uint32_t>(c.x);
        h *= 1099511628211ull; // FNV-1a 64-bit prime

        h ^= static_cast<std::uint32_t>(c.y);
        h *= 1099511628211ull;

        h ^= static_cast<std::uint32_t>(c.z);
        h *= 1099511628211ull;

        return h;
    }
};

class ChunkCoordinateSystem
{
    using Vector3D = Eigen::Vector3d;
public:

    // Enum Class for Neighbour types
    enum class NeighbourType : uint8_t {
        SOURCE = 0,
        FACE = 1, // one axis delta
        EDGE = 2, // two axes delta
        CORNER = 3, // three axes delta
        INDETERMINATE = 4 // not an immediate neighbour
    };

    enum class AxisType : uint8_t {
        X = 0,
        Y = 1,
        Z = 2,
        INDETERMINATE = 4
    };

    static inline std::string reflectNeighbourType(const NeighbourType& type) {
        // build a mapping
        std::array<std::string,5> mapping = {"SOURCE","FACE","EDGE","CORNER","INDETERMINATE"};

        return mapping[static_cast<uint8_t>(type)];
    }

    static inline std::string reflectAxisType(const AxisType& type) {
        // create the mapping
        std::array<std::string,4> mapping = {"X","Y","Z","INDETERMINATE"};
        
        return mapping[static_cast<uint8_t>(type)];
    }

    /**
     * @brief Contructor
     * @param chunk_size Size of the chunk
     */
    explicit ChunkCoordinateSystem(double chunk_size);

    /**
     * @brief 
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] ChunkCoord positionToChunkCoordinate(const PositionCoordinateT& position_coordinate) const {
        // Convert it to a point we can all agree upon
        Vector3D point_to_use = Bonxai::ConvertPoint<Vector3D>(position_coordinate);
        return positionToChunkCoordinate(point_to_use.x(),point_to_use.y(),point_to_use.z());
    }

    /**
     * @brief
     */
    [[nodiscard]] ChunkCoord positionToChunkCoordinate(double x, double y, double z) const;

    /**
     * @brief
     */
    [[nodiscard]] Vector3D chunkToPositionCoordinate(const ChunkCoord& chunk_coord) const;

    /**
     * @brief
     */
    [[nodiscard]] Vector3D chunkMinBounds(const ChunkCoord& chunk_coord) const;

    /**
     * @brief
     */
    [[nodiscard]] double chunkMinBounds(const ChunkCoord& chunk_coord, const AxisType &axis) const;

    /**
     * @brief
     */
    [[nodiscard]] Vector3D chunkMaxBounds(const ChunkCoord& chunk_coord) const;

    /**
     * @brief
     */
    [[nodiscard]] double chunkMaxBounds(const ChunkCoord& chunk_coord, const AxisType &axis) const;


    /**
     * @brief
     */
    [[nodiscard]] double getChunkSize() const;

    /**
     * @brief
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] bool isPositionInChunk(const PositionCoordinateT& position, const ChunkCoord& chunk_coord) const {
        Vector3D point_to_use = Bonxai::ConvertPoint<Vector3D>(position);
        return isPositionInChunk(point_to_use,chunk_coord);
    }

    /**
     * @brief 
     */
    [[nodiscard]] bool isPositionInChunk(const Vector3D& position, const ChunkCoord& chunk_coord) const;

    /**
     * @brief 
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] double distanceToBoundary(const PositionCoordinateT& position, 
                                            const ChunkCoord& chunk_coord,
                                            std::optional<AxisType> optional_axis = std::nullopt) const {
        Vector3D pos_to_use = Bonxai::ConvertPoint<Vector3D>(position);
        return distanceToBoundary(pos_to_use,chunk_coord,optional_axis);
    }

    /**
     * @brief
     */
    [[nodiscard]] double distanceToBoundary(const Vector3D& position, 
                                            const ChunkCoord& chunk_coord,
                                            std::optional<AxisType> optional_axis = std::nullopt) const;

    /**
     * @brief
     */
    [[nodiscard]] static std::vector<ChunkCoord> getFaceNeighbours(const ChunkCoord& chunk_coord);

    /**
     * @brief
     */
    [[nodiscard]] static std::vector<ChunkCoord> getEdgeNeighbours(const ChunkCoord& chunk_coord);

    /**
     * @brief
     */
    [[nodiscard]] static std::vector<ChunkCoord> getCornerNeighbours(const ChunkCoord& chunk_coord);
    
    /**
     * @brief
     */
    [[nodiscard]] static std::vector<ChunkCoord> getAllNeighbours(const ChunkCoord& chunk_coord);

    /**
     * @brief
     */
    [[nodiscard]] static NeighbourType getNeighbourType(const ChunkCoord& delta);

    template <typename Func>
    static void forEachNeighbour(const ChunkCoord& center, int radius, Func&& func) {
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    ChunkCoord nb{center.x + dx, center.y + dy, center.z + dz};
                    func(nb); 
                }
            }
        }
    }

    /**
     * @brief 
     */
    [[nodiscard]] static int32_t roundToInt(double val);

private:
    double chunk_size_;
    double chunk_size_half_;
    Vector3D half_chunk_vec;

}; // class ChunkCoordinateSystem

} // namespace RollingBonxai