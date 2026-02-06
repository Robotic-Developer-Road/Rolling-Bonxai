#include "rolling_bonxai/file_storage_backend.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>



namespace RollingBonxai 
{

// ============================================================================
// FileStorageBackend Implementation
// ============================================================================

FileStorageBackend::FileStorageBackend(const std::filesystem::path& base_dir)
: 
base_dir_(base_dir),
backend_init_(false) 
{}

// ============================================================================
// Core Operations
// ============================================================================

bool FileStorageBackend::save(const ChunkCoord& coord, 
                               const Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>& grid) {
    try {
        // Create chunk directory if it doesn't exist
        auto chunk_dir = getChunkDir(coord);
        std::error_code ec;
        std::filesystem::create_directories(chunk_dir, ec);
        
        if (ec) {
            return false;  // Failed to create directory
        }
        
        // Save grid to .chunk file
        auto chunk_path = getChunkPath(coord);
        std::ofstream chunk_file(chunk_path, std::ios::binary);
        if (!chunk_file.is_open()) {
            return false;  // Failed to open chunk file
        }
        Bonxai::Serialize(chunk_file, grid);
        chunk_file.close();
        
        // Update timestamp
        auto timestamp_opt = loadTimestamp(coord);
        ChunkTimestamp timestamp;
        
        if (timestamp_opt.has_value()) {
            // Chunk already existed - update modified time
            timestamp = timestamp_opt.value();
            timestamp.last_modified_ns = getCurrentTimeNs();
        } else {
            // New chunk - set creation and modified time
            timestamp = createTimestamp();
        }
        
        return saveTimestamp(coord, timestamp);
        
    } catch (const std::exception&) {
        return false;  // Catch any serialization or I/O errors
    }
}

std::unique_ptr<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>> 
FileStorageBackend::load(const ChunkCoord& coord) {
    try {
        auto chunk_path = getChunkPath(coord);
        
        // Check if chunk file exists
        if (!std::filesystem::exists(chunk_path)) {
            return nullptr;  // Chunk doesn't exist
        }
        
        // Load and deserialize chunk
        std::ifstream chunk_file(chunk_path, std::ios::binary);
        if (!chunk_file.is_open()) {
            return nullptr;  // Failed to open chunk file
        }
        
        // Read header to get grid info
        std::string header_line;
        std::getline(chunk_file, header_line);
        
        auto header_info = Bonxai::GetHeaderInfo(header_line);
        
        // Deserialize the grid
        auto grid = Bonxai::Deserialize<Bonxai::Occupancy::CellOcc>(chunk_file, header_info);
        
        chunk_file.close();
        
        // Update timestamp (accessed time and count)
        bool update_access_ok = updateAccessTime(coord);

        // 
        
        return std::make_unique<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>>(std::move(grid));
        
    } catch (const std::exception&) {
        return nullptr;  // Deserialization or I/O error
    }
}

bool FileStorageBackend::exists(const ChunkCoord& coord) const {
    auto chunk_path = getChunkPath(coord);
    return std::filesystem::exists(chunk_path);
}

// ============================================================================
// Timestamp Operations
// ============================================================================

std::optional<ChunkTimestamp> FileStorageBackend::loadTimestamp(const ChunkCoord& coord) const {
    auto stamp_path = getStampPath(coord);
    
    if (!std::filesystem::exists(stamp_path)) {
        return std::nullopt;  // Timestamp file doesn't exist
    }
    
    std::ifstream stamp_file(stamp_path);
    if (!stamp_file.is_open()) {
        return std::nullopt;  // Failed to open timestamp file
    }
    
    return parseTimestamp(stamp_file);
}

bool FileStorageBackend::updateModifiedTime(const ChunkCoord& coord) {
    auto timestamp_opt = loadTimestamp(coord);
    ChunkTimestamp timestamp;
    
    if (timestamp_opt.has_value()) {
        timestamp = timestamp_opt.value();
        timestamp.last_modified_ns = getCurrentTimeNs();
    } else {
        // Create new timestamp if doesn't exist
        timestamp = createTimestamp();
    }
    
    return saveTimestamp(coord, timestamp);
}

bool FileStorageBackend::updateAccessTime(const ChunkCoord& coord) {
    auto timestamp_opt = loadTimestamp(coord);
    ChunkTimestamp timestamp;
    
    if (timestamp_opt.has_value()) {
        timestamp = timestamp_opt.value();
        timestamp.last_accessed_ns = getCurrentTimeNs();
        timestamp.access_count++;
    } else {
        // Create new timestamp if doesn't exist
        timestamp = createTimestamp();
        timestamp.access_count = 1;
    }
    
    return saveTimestamp(coord, timestamp);
}
// ============================================================================
// Getters
// ============================================================================
const std::filesystem::path& FileStorageBackend::getBaseDir() const {
    return base_dir_;
}

std::string FileStorageBackend::getBaseDirStr() const {
    return base_dir_.string();
}

bool FileStorageBackend::isBackendInit() const {
    return backend_init_;
}
// ============================================================================
// Initialization
// ============================================================================
bool FileStorageBackend::initStorageBackend(){
    // Create base directory if it doesn't exist
    std::error_code ec;
    std::filesystem::create_directories(base_dir_, ec);
    
    if (ec) {
        return false;
    }
    
    // Verify directory is writable by attempting to create a test file
    auto test_file = base_dir_ / ".write_test";
    std::ofstream test_stream(test_file);
    if (!test_stream.is_open()) {
        return false;
    }
    test_stream.close();
    std::filesystem::remove(test_file, ec);  // Clean up test file

    this->backend_init_ = true;
    return true;
}

// ============================================================================
// Internal Methods
// ============================================================================

std::filesystem::path FileStorageBackend::getChunkDir(const ChunkCoord& coord) const {
    return base_dir_ / coordToString(coord);
}

std::filesystem::path FileStorageBackend::getChunkPath(const ChunkCoord& coord) const {
    auto coord_str = coordToString(coord);
    return getChunkDir(coord) / (coord_str + ".chunk");
}

std::filesystem::path FileStorageBackend::getStampPath(const ChunkCoord& coord) const {
    auto coord_str = coordToString(coord);
    return getChunkDir(coord) / (coord_str + ".timestamp");
}

std::string FileStorageBackend::coordToString(const ChunkCoord& coord) const {
    std::ostringstream oss;
    oss << coord.x << "_" << coord.y << "_" << coord.z;
    return oss.str();
}

ChunkTimestamp FileStorageBackend::createTimestamp() const {
    ChunkTimestamp timestamp;
    int64_t now_ns = getCurrentTimeNs();
    
    timestamp.creation_time_ns = now_ns;
    timestamp.last_modified_ns = now_ns;
    timestamp.last_accessed_ns = now_ns;
    timestamp.access_count = 0;
    
    return timestamp;
}

bool FileStorageBackend::saveTimestamp(const ChunkCoord& coord, const ChunkTimestamp& timestamp) {
    try {
        auto stamp_path = getStampPath(coord);
        
        // Create YAML node
        YAML::Node node;
        node["creation"] = timestamp.creation_time_ns;
        node["modified"] = timestamp.last_modified_ns;
        node["accessed"] = timestamp.last_accessed_ns;
        node["count"] = timestamp.access_count;
        
        // Write to file
        std::ofstream stamp_file(stamp_path);
        if (!stamp_file.is_open()) {
            return false;
        }
        
        stamp_file << node;
        stamp_file.close();
        return true;
        
    } catch (const std::exception&) {
        return false;
    }
}


std::optional<ChunkTimestamp> FileStorageBackend::parseTimestamp(std::ifstream& input) const {
    try {
        YAML::Node node = YAML::Load(input);
        
        ChunkTimestamp timestamp;
        timestamp.creation_time_ns = node["creation"].as<int64_t>();
        timestamp.last_modified_ns = node["modified"].as<int64_t>();
        timestamp.last_accessed_ns = node["accessed"].as<int64_t>();
        timestamp.access_count = node["count"].as<uint64_t>();
        
        return timestamp;
        
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

int64_t FileStorageBackend::getCurrentTimeNs() const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

} // namespace RollingBonxai