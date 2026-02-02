#include "rolling_bonxai/common.hpp"

namespace RollingBonxai
{
// ============================================================================
// ChunkCoord impl
// ============================================================================

// Initialise with 0s
ChunkCoord::ChunkCoord():x{0},y{0},z{0}{}

// Initialise with inputs
ChunkCoord::ChunkCoord(int32_t x_, int32_t y_, int32_t z_):x{x_},y{y_},z{z_} {

}

bool ChunkCoord::operator==(const ChunkCoord& other) const {
    return (this->x == other.x) && (this->y == other.y) && (this->z == other.z);
}

bool ChunkCoord::operator!=(const ChunkCoord& other) const {
    return !(*this == other);
}

ChunkCoord ChunkCoord::operator+(const ChunkCoord& offset) const {
    return {this->x + offset.x , this->y + offset.y, this->z + offset.z};
}

ChunkCoord ChunkCoord::operator-(const ChunkCoord& offset) const {
    return {this->x - offset.x , this->y - offset.y, this->z - offset.z};
}

ChunkCoord& ChunkCoord::operator+=(const ChunkCoord& offset) {
    this->x+=offset.x;
    this->y+=offset.y;
    this->z+=offset.z;
    return *this;
}

ChunkCoord& ChunkCoord::operator-=(const ChunkCoord& offset) {
    this->x-=offset.x;
    this->y-=offset.y;
    this->z-=offset.z;
    return *this;
}

// ============================================================================
// ChunkTimestamp impl
// ============================================================================
ChunkTimestamp::ChunkTimestamp()
    : creation_time_ns(0),
      last_modified_ns(0),
      last_accessed_ns(0),
      access_count(0) {}

std::chrono::system_clock::time_point ChunkTimestamp::getCreationTime() const {
    return std::chrono::system_clock::time_point(std::chrono::nanoseconds(creation_time_ns));
}

std::chrono::system_clock::time_point ChunkTimestamp::getLastModified() const {
    return std::chrono::system_clock::time_point(std::chrono::nanoseconds(last_modified_ns));
}

std::chrono::system_clock::time_point ChunkTimestamp::getLastAccessed() const {
    return std::chrono::system_clock::time_point(std::chrono::nanoseconds(last_accessed_ns));
}

std::chrono::seconds ChunkTimestamp::getAge() const {
    auto now = std::chrono::system_clock::now();
    auto creation = getCreationTime();
    return std::chrono::duration_cast<std::chrono::seconds>(now - creation);
}

std::chrono::seconds ChunkTimestamp::getTimeSinceModified() const {
    auto now = std::chrono::system_clock::now();
    auto modified = getLastModified();
    return std::chrono::duration_cast<std::chrono::seconds>(now - modified);
}

std::chrono::seconds ChunkTimestamp::getTimeSinceAccessed() const {
    auto now = std::chrono::system_clock::now();
    auto accessed = getLastAccessed();
    return std::chrono::duration_cast<std::chrono::seconds>(now - accessed);
}

uint64_t ChunkTimestamp::getAccessCount() const {
    return this->access_count;
}

} //namespace RollingBonxai