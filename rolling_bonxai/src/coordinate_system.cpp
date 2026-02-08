#include "rolling_bonxai/coordinate_system.hpp"


namespace RollingBonxai
{
// ============================================================================
// Coordinate System Impl
// ============================================================================
ChunkCoordinateSystem::ChunkCoordinateSystem(double voxel_resolution,double chunk_size) {
    voxel_resolution_ = voxel_resolution;
    voxel_inv_resolution_ = 1.0 / voxel_resolution_;
    chunk_size_ = chunk_size;
    chunk_size_half_ = chunk_size / 2.0;
    half_chunk_vec = Vector3D(chunk_size_half_,chunk_size_half_,chunk_size_half_);
}

ChunkCoord ChunkCoordinateSystem::positionToChunkCoordinate(double x, double y, double z) const {
    return {roundToInt(x / chunk_size_),roundToInt(y / chunk_size_),roundToInt(z / chunk_size_)};
}

Bonxai::CoordT ChunkCoordinateSystem::positionToVoxelCoordinate(const Bonxai::Point3D& bonxai_3d_pt) const {
    return Bonxai::PosToCoord(bonxai_3d_pt,voxel_inv_resolution_);
}

Bonxai::CoordT ChunkCoordinateSystem::positionToVoxelCoordinate(double x, double y, double z) const {
    const auto p3d = Bonxai::ConvertPoint<Bonxai::Point3D>(Eigen::Vector3d(x,y,z));
    return positionToVoxelCoordinate(p3d);
}

ChunkCoordinateSystem::Vector3D ChunkCoordinateSystem::chunkToPositionCoordinate(const ChunkCoord& chunk_coord) const {
    return Vector3D(chunk_coord.x * chunk_size_,
                    chunk_coord.y * chunk_size_,
                    chunk_coord.z * chunk_size_);
}

ChunkCoordinateSystem::Vector3D ChunkCoordinateSystem::voxelToPositionCoordinate(const Bonxai::CoordT& voxel_coord) const {
    auto p3d_ret = Bonxai::CoordToPos(voxel_coord,voxel_resolution_);
    Vector3D vec_to_ret = Bonxai::ConvertPoint<Vector3D>(p3d_ret);
    return vec_to_ret;
}

ChunkCoordinateSystem::Vector3D ChunkCoordinateSystem::chunkMinBounds(const ChunkCoord& chunk_coord) const {
    // Compute the center position
    auto center = chunkToPositionCoordinate(chunk_coord);

    auto min_coordinate = center - half_chunk_vec;

    return min_coordinate;
}

double ChunkCoordinateSystem::chunkMinBounds(const ChunkCoord& chunk_coord, const AxisType &axis) const {
    // Get the min coordinate
    Vector3D coordinate = chunkMinBounds(chunk_coord);
    if (axis == AxisType::X) {
        return coordinate.x();
    }
    else if (axis == AxisType::Y) {
        return coordinate.y();
    }
    else if (axis == AxisType::Z) {
        return coordinate.z();
    }
    else {
        return coordinate.x();;
    }
}

ChunkCoordinateSystem::Vector3D ChunkCoordinateSystem::chunkMaxBounds(const ChunkCoord& chunk_coord) const {
    // Compute the center position
    auto center = chunkToPositionCoordinate(chunk_coord);

    auto max_coordinate = center + half_chunk_vec;

    return max_coordinate;
}

double ChunkCoordinateSystem::chunkMaxBounds(const ChunkCoord& chunk_coord, const AxisType &axis) const {
    // Get the min coordinate
    Vector3D coordinate = chunkMaxBounds(chunk_coord);
    if (axis == AxisType::X) {
        return coordinate.x();
    }
    else if (axis == AxisType::Y) {
        return coordinate.y();
    }
    else if (axis == AxisType::Z) {
        return coordinate.z();
    }
    else {
        return coordinate.x();
    }
}

std::pair<Bonxai::CoordT, Bonxai::CoordT> 
ChunkCoordinateSystem::worldBoundsToVoxelBounds(const ChunkCoord& chunk) const
{
    const Vector3D world_min = this->chunkMinBounds(chunk);
    const Vector3D world_max = this->chunkMaxBounds(chunk);

    Bonxai::CoordT voxel_min = this->positionToVoxelCoordinate(world_min);
    Bonxai::CoordT voxel_max = this->positionToVoxelCoordinate(world_max);

    return {voxel_min, voxel_max};
}

double ChunkCoordinateSystem::getChunkSize() const {
    return this->chunk_size_;
}

bool ChunkCoordinateSystem::isPositionInChunk(const Vector3D& position, const ChunkCoord& chunk_coord) const {
    // convert the position to chunk and see if it matches
    const double cx = chunk_coord.x * chunk_size_;
    const double cy = chunk_coord.y * chunk_size_;
    const double cz = chunk_coord.z * chunk_size_;
    return (position.x() >= cx - chunk_size_half_) && (position.x() < cx + chunk_size_half_)
        && (position.y() >= cy - chunk_size_half_) && (position.y() < cy + chunk_size_half_)
        && (position.z() >= cz - chunk_size_half_) && (position.z() < cz + chunk_size_half_);
}

double ChunkCoordinateSystem::distanceToBoundary(const Vector3D& position, 
                                            const ChunkCoord& chunk_coord,
                                            std::optional<AxisType> optional_axis) const {

    // Check if the position even exists in this chunk. Return -1 if it is not in the chunk
    if (!isPositionInChunk(position,chunk_coord)) { 
        return -1.0;
    }

    // Retrieve the upper and lower bounds
    auto upper_bound = chunkMaxBounds(chunk_coord);
    auto lower_bound = chunkMinBounds(chunk_coord);

    // Compute the differences to the upper and lower bounds
    auto abs_diff_to_upper = (position - upper_bound).cwiseAbs();
    auto abs_diff_to_lower = (position - lower_bound).cwiseAbs();

    bool optional_axis_arg_given = optional_axis.has_value();
    if (optional_axis_arg_given) {
        // Check
        if (*optional_axis == AxisType::INDETERMINATE) {
            // Cmon, you literally just need to either give 0:x axis, 1: y axis, 2: z_axis
            // Non complance will give you a failure value
            return -1.0;
        }
    }

    double diff = std::numeric_limits<double>::max();
    
    for (Eigen::Index i = 0 ; i < 3 ; ++i) {
        // consistent with rounding up if equal
        double d = std::min(abs_diff_to_upper[i], abs_diff_to_lower[i]);
    
        if (optional_axis_arg_given && static_cast<int>(*optional_axis) == static_cast<int>(i)) {
            return d;
        }

        // otherwise compute the min diff
        diff = std::min(diff,d);
    }

    return diff;
}

std::vector<ChunkCoord> ChunkCoordinateSystem::getFaceNeighbours(const ChunkCoord& chunk_coord) {
    return 
    {{
    {chunk_coord.x + 1, chunk_coord.y, chunk_coord.z}, //front
    {chunk_coord.x - 1, chunk_coord.y, chunk_coord.z}, //back
    {chunk_coord.x, chunk_coord.y + 1, chunk_coord.z}, //left
    {chunk_coord.x, chunk_coord.y - 1, chunk_coord.z}, //right
    {chunk_coord.x, chunk_coord.y, chunk_coord.z + 1}, //top
    {chunk_coord.x, chunk_coord.y, chunk_coord.z - 1}  //down
    }};
}

std::vector<ChunkCoord> ChunkCoordinateSystem::getEdgeNeighbours(const ChunkCoord& chunk_coord) {
    //Follow the right-handle rule system X(front),Y(Left), Z(Up)
    return 
    {{
        // xy-plane edges
        {chunk_coord.x - 1, chunk_coord.y - 1, chunk_coord.z}, //back-right
        {chunk_coord.x - 1, chunk_coord.y + 1, chunk_coord.z}, //back-left
        {chunk_coord.x + 1, chunk_coord.y - 1, chunk_coord.z}, //front-right
        {chunk_coord.x + 1, chunk_coord.y + 1, chunk_coord.z}, //front-left

        // xz-plane edges
        {chunk_coord.x - 1, chunk_coord.y, chunk_coord.z - 1}, //back-down
        {chunk_coord.x - 1, chunk_coord.y, chunk_coord.z + 1}, //back-top
        {chunk_coord.x + 1, chunk_coord.y, chunk_coord.z - 1}, //front-down
        {chunk_coord.x + 1, chunk_coord.y, chunk_coord.z + 1}, //front-top

        // yz-plane edges
        {chunk_coord.x, chunk_coord.y - 1, chunk_coord.z - 1}, //right-down
        {chunk_coord.x, chunk_coord.y - 1, chunk_coord.z + 1}, //right-top
        {chunk_coord.x, chunk_coord.y + 1, chunk_coord.z - 1}, //left-down
        {chunk_coord.x, chunk_coord.y + 1, chunk_coord.z + 1}  //left-top
    }};
}

std::vector<ChunkCoord> ChunkCoordinateSystem::getCornerNeighbours(const ChunkCoord& chunk_coord) {
    //Follow the right-handle rule system X(front),Y(Left), Z(Up)
    return 
    {{
        {chunk_coord.x - 1, chunk_coord.y - 1, chunk_coord.z - 1}, //back  - right  -  down
        {chunk_coord.x - 1, chunk_coord.y - 1, chunk_coord.z + 1}, //back  - right  -  top
        {chunk_coord.x - 1, chunk_coord.y + 1, chunk_coord.z - 1}, //back  - left   -  down
        {chunk_coord.x - 1, chunk_coord.y + 1, chunk_coord.z + 1}, //back  - left  -   top
        {chunk_coord.x + 1, chunk_coord.y - 1, chunk_coord.z - 1}, //front - right  -  down
        {chunk_coord.x + 1, chunk_coord.y - 1, chunk_coord.z + 1}, //front - right  -  top
        {chunk_coord.x + 1, chunk_coord.y + 1, chunk_coord.z - 1}, //front - left   -  down
        {chunk_coord.x + 1, chunk_coord.y + 1, chunk_coord.z + 1}  //front - left   -  top
    }};
}

std::vector<ChunkCoord> ChunkCoordinateSystem::getAllNeighbours(const ChunkCoord& chunk_coord) {
    // createa vector for all neighbours
    std::vector<ChunkCoord> all_neighbours;
    all_neighbours.reserve(26);

    forEachNeighbour(chunk_coord, 1, [&all_neighbours](const ChunkCoord& nb) {
        all_neighbours.push_back(nb);
    });
    
    return all_neighbours;
}

[[nodiscard]] ChunkCoordinateSystem::NeighbourType ChunkCoordinateSystem::getNeighbourType(const ChunkCoord& delta) {

    int32_t abs_x = std::abs(delta.x);
    int32_t abs_y = std::abs(delta.y);
    int32_t abs_z = std::abs(delta.z);

    if (abs_x > 1 || abs_y > 1 || abs_z > 1) {
        return NeighbourType::INDETERMINATE;
    }

    int32_t sum = abs_x + abs_y + abs_z;
    
    switch (sum) {
        case 0:
            return NeighbourType::SOURCE;
            break;
        case 1:
            return NeighbourType::FACE;
            break;
        case 2:
            return NeighbourType::EDGE;
            break;
        case 3:
            return NeighbourType::CORNER;
        default:
            return NeighbourType::INDETERMINATE;
    }
}

int32_t ChunkCoordinateSystem::roundToInt(double val) {
    return static_cast<int32_t>(val + (val >= 0 ? 0.5 : -0.5));
}

} //namespace RollingBonxai