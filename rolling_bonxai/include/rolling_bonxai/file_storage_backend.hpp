#pragma once

#include <filesystem>
#include <optional>
#include <cstdint>
#include <fstream>
#include <chrono>

#include "bonxai_core/bonxai.hpp"
#include "bonxai_core/serialization.hpp"
#include "bonxai_map/occupancy_map.hpp"
#include "rolling_bonxai/common.hpp"


namespace RollingBonxai 
{
/**
 * @brief File-based storage backend for occupancy map chunks
 * 
 * @note User must remember to call initBackend() method
 * Storage structure:
 * ```
 * <base_dir>/
 *   <x>_<y>_<z>/
 *     <x>_<y>_<z>.chunk       <- Bonxai serialized VoxelGrid
 *     <x>_<y>_<z>.timestamp   <- yaml file
 * ```
 * 
 * Example:
 * ```
 * /var/rolling_bonxai/chunks/
 *   5_10_2/
 *     5_10_2.chunk
 *     5_10_2.timestamp
 *   -3_7_-2/
 *     -3_7_-2.chunk
 *     -3_7_-2.timestamp
 * ```
 * 
 * Timestamp file format (text):
 * ```
 * creation: 1738077135123456789
 * modified: 1738077522987654321
 * accessed: 1738079445123789456
 * count: 42
 * ```
 */
class FileStorageBackend {
public:
    /**
     * @brief Construct storage backend with base directory
     * 
     * @param base_dir Base directory for chunk storage
     * @throws std::runtime_error if directory cannot be created
     * 
     * @post base_dir exists and is writable
     */
    explicit FileStorageBackend(const std::filesystem::path& base_dir);
    
    ~FileStorageBackend() = default;
    
    // No copying
    FileStorageBackend(const FileStorageBackend&) = delete;
    FileStorageBackend& operator=(const FileStorageBackend&) = delete;
    
    // Move semantics
    FileStorageBackend(FileStorageBackend&&) = default;
    FileStorageBackend& operator=(FileStorageBackend&&) = default;
    
    // ========================================================================
    // Core Operations
    // ========================================================================
    
    /**
     * @brief Save chunk to storage
     * 
     * @param coord Chunk coordinate
     * @param grid VoxelGrid to serialize
     * @return true on success, false on failure
     * 
     * @details
     * - Creates chunk directory if needed
     * - Serializes grid to .chunk file using Bonxai::Serialize
     * - Updates .timestamp file (modified time)
     * - If chunk doesn't exist, sets creation time
     * 
     * @complexity O(N) where N = number of active cells in grid
     * @performance ~1-10 ms per chunk depending on size
     */
    [[nodiscard]] bool save(const ChunkCoord& coord, 
              const Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>& grid);
    
    /**
     * @brief Load chunk from storage
     * 
     * @param coord Chunk coordinate
     * @return unique_ptr to VoxelGrid if found, nullptr if not found
     * 
     * @details
     * - Deserializes grid from .chunk file using Bonxai::Deserialize
     * - Updates .timestamp file (accessed time, increment count)
     * - Returns nullptr if chunk doesn't exist or deserialization fails
     * 
     * @complexity O(N) where N = number of active cells in grid
     * @performance ~1-10 ms per chunk depending on size
     */
    [[nodiscard]] std::unique_ptr<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>> 
        load(const ChunkCoord& coord);
    
    /**
     * @brief Check if chunk exists in storage
     * 
     * @param coord Chunk coordinate
     * @return true if .chunk file exists
     * 
     * @note Does not validate that file is readable or valid
     * @complexity O(1) - just checks file existence
     */
    [[nodiscard]] bool exists(const ChunkCoord& coord) const;
    
    // ========================================================================
    // Timestamp Operations
    // ========================================================================
    
    /**
     * @brief Load timestamp metadata for a chunk
     * 
     * @param coord Chunk coordinate
     * @return ChunkTimestamp if found, std::nullopt if not found or parse error
     * 
     * @complexity O(1) - reads small text file
     */
    [[nodiscard]] std::optional<ChunkTimestamp> loadTimestamp(const ChunkCoord& coord) const;
    
    /**
     * @brief Update modified time to current time
     * 
     * @param coord Chunk coordinate
     * @return true on success, false on failure
     * 
     * @note If timestamp doesn't exist, creates it with current time
     */
    [[nodiscard]] bool updateModifiedTime(const ChunkCoord& coord);
    
    /**
     * @brief Update accessed time to current time and increment count
     * 
     * @param coord Chunk coordinate
     * @return true on success, false on failure
     * 
     * @note If timestamp doesn't exist, creates it with current time
     */
    [[nodiscard]] bool updateAccessTime(const ChunkCoord& coord);
    
    /**
     * @brief Get base directory path
     * 
     * @return const reference to base directory
     */
    [[nodiscard]] const std::filesystem::path& getBaseDir() const;

    /**
     * @brief Get the base directory path as a string
     * 
     * @return string form of base directory (full path)
     */
    [[nodiscard]] std::string getBaseDirStr() const;

    /**
     * @brief Check if the backend has been initialised
     * This is done by initStorageBackend() method
     */
    [[nodiscard]] bool isBackendInit() const;

    /**
     * @brief Initialise the file storage backend
     * 
     * @return bool true if successful, false otherwises
     * 
     * @warning User must remember to initialise this
     */
    [[nodiscard]] bool initStorageBackend();

    
private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    /**
     * @brief Get directory path for a chunk
     * 
     * @param coord Chunk coordinate
     * @return Path to chunk directory (e.g., "<base_dir>/5_10_2/")
     */
    std::filesystem::path getChunkDir(const ChunkCoord& coord) const;
    
    /**
     * @brief Get chunk file path
     * 
     * @param coord Chunk coordinate
     * @return Path to .chunk file (e.g., "<base_dir>/5_10_2/5_10_2.chunk")
     */
    std::filesystem::path getChunkPath(const ChunkCoord& coord) const;
    
    /**
     * @brief Get timestamp file path
     * 
     * @param coord Chunk coordinate
     * @return Path to .timestamp file (e.g., "<base_dir>/5_10_2/5_10_2.timestamp")
     */
    std::filesystem::path getStampPath(const ChunkCoord& coord) const;
    
    /**
     * @brief Get coordinate string representation
     * 
     * @param coord Chunk coordinate
     * @return String like "5_10_2" or "-3_7_-2"
     */
    std::string coordToString(const ChunkCoord& coord) const;
    
    /**
     * @brief Create timestamp with current time
     * 
     * @return ChunkTimestamp initialized to now
     */
    ChunkTimestamp createTimestamp() const;
    
    /**
     * @brief Save timestamp to file
     * 
     * @param coord Chunk coordinate
     * @param timestamp Timestamp to save to yaml
     * @return true on success, false on failure
     */
    bool saveTimestamp(const ChunkCoord& coord, const ChunkTimestamp& timestamp);
    
    /**
     * @brief Parse timestamp from file stream
     * 
     * @param input Input stream
     * @return ChunkTimestamp if successful, std::nullopt on parse error
     */
    std::optional<ChunkTimestamp> parseTimestamp(std::ifstream& input) const;
    
    /**
     * @brief Get current time in nanoseconds since epoch
     * 
     * @return Nanoseconds since epoch
     */
    int64_t getCurrentTimeNs() const;
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    std::filesystem::path base_dir_;
    bool backend_init_ {false};
};

} // namespace RollingBonxai