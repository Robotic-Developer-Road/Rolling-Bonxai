#pragma once
#include <functional>
#include <memory>
#include <vector>
#include <deque>
#include <unordered_map>
#include "bonxai_map/occupancy_map.hpp"
#include "rolling_bonxai/common.hpp"

/*
The Chunk Storage interface is essentially an ADT that
provides the following API:
1. insert(...) > Insert a ManagedChunk
2. find() > Finds and returns a ref to the ManagedChunk 
3. erase() > Deletes a ManagedChunk with a given ChunkCoord (no-op if not found)
4. contains() > Returns true if a ManagedChunk with a given ChunkCoord is present
5. size() > Returns the number of ManagedChunks currently stored
6. empty() > Return true if the container is empty
*/

namespace RollingBonxai
{
class ChunkStorage
{
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~ChunkStorage() = default;

    /**
     * @brief Insert a chunk. If coord already exists, insertion is skipped.
     */
    virtual void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) = 0;

    /**
     * @brief Find a chunk by coordinate. Returns nullptr if not found.
     */
    virtual ManagedChunk* find(const ChunkCoord& coord) = 0;
    virtual const ManagedChunk* find(const ChunkCoord& coord) const = 0;

    /**
     * @brief Check if a chunk with given coordinate exists.
     */
    virtual bool contains(const ChunkCoord& coord) const = 0;

    /**
     * @brief Erase a chunk by coordinate. No-op if not found.
     */
    virtual void erase(const ChunkCoord& coord) = 0;

    /**
     * @brief Clear all chunks.
     */
    virtual void clear() = 0;

    /**
     * @brief Get number of chunks currently stored.
     */
    virtual size_t size() const = 0;

    /**
     * @brief Get current capacity (storage-dependent meaning).
     */
    virtual size_t capacity() const = 0;

    /**
     * @brief Check if storage is empty.
     */
    virtual bool empty() const = 0;

    /// Mutable traversal
    virtual void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) = 0;

    /// Read-only traversal
    virtual void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const = 0;
};

/*
ChunkStorage implementation using std::vector with linear search.
Best for small chunk counts (< 100 chunks).
*/
class VectorChunkStorage final : public ChunkStorage
{
public:
    /**
     * @brief Construct with initial capacity of 27 (typical neighborhood size).
     */
    VectorChunkStorage();
    
    /**
     * @brief Construct with initial capacity of n (typical neighborhood size).
     */
    VectorChunkStorage(size_t capacity_hint);

    /**
     * @brief Destructor
     */
    ~VectorChunkStorage() override = default;

    // Delete copy operations (ManagedChunk is move-only)
    VectorChunkStorage(const VectorChunkStorage&) = delete;
    VectorChunkStorage& operator=(const VectorChunkStorage&) = delete;

    // Default move operations
    VectorChunkStorage(VectorChunkStorage&&) noexcept = default;
    VectorChunkStorage& operator=(VectorChunkStorage&&) noexcept = default;

    /**
     * @brief Insert a chunk. If coord already exists, insertion is skipped.
     * @complexity O(n) where n = current size
     */
    void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) override;

    /**
     * @brief Find a chunk. Returns nullptr if not found.
     * @complexity O(n) linear search
     */
    ManagedChunk* find(const ChunkCoord& coord) override;

    /**
     * @brief Find a chunk (const). Returns nullptr if not found.
     * @complexity O(n) linear search
     */
    const ManagedChunk* find(const ChunkCoord& coord) const override;

    /**
     * @brief Check if chunk exists.
     * @complexity O(n) linear search
     */
    bool contains(const ChunkCoord& coord) const override;

    /**
     * @brief Erase a chunk by coordinate. 
     * Uses swap-and-pop for O(1) erase (does not preserve order).
     * No-op if coordinate not found.
     * @complexity O(n) search + O(1) erase
     */
    void erase(const ChunkCoord& coord) override;

    /**
     * @brief Clear all chunks and reset capacity to 27.
     */
    void clear() override;

    /**
     * @brief Get number of chunks stored.
     */
    size_t size() const override;

    /**
     * @brief Get current vector capacity (reserved memory).
     */
    size_t capacity() const override;

    /**
     * @brief Check if empty.
     */
    bool empty() const override;

    /**
     * @brief Iterate over all chunks (mutable).
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) override;

    /**
     * @brief Iterate over all chunks (const).
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const override;

    /**
     * @brief Get a hint of the capacity required
     */
    size_t getCapacityHint() const;

private:
    std::vector<std::pair<ChunkCoord, ManagedChunk>> chunks_;
    size_t capacity_hint_{27};
};

/*
ChunkStorage implementation using std::unordered_map.
Best for large chunk counts (> 100 chunks) or when fast lookup is critical.
*/
class HashMapChunkStorage final : public ChunkStorage
{
public:
    /**
     * @brief Construct with initial bucket count for ~27 chunks.
     */
    HashMapChunkStorage();

    /**
     * @brief Construct with initial bucket count for ~n chunks.
     */
    HashMapChunkStorage(size_t capacity_hint);

    /**
     * @brief Destructor
     */
    ~HashMapChunkStorage() override = default;

    // Delete copy operations (ManagedChunk is move-only)
    HashMapChunkStorage(const HashMapChunkStorage&) = delete;
    HashMapChunkStorage& operator=(const HashMapChunkStorage&) = delete;

    // Default move operations
    HashMapChunkStorage(HashMapChunkStorage&&) noexcept = default;
    HashMapChunkStorage& operator=(HashMapChunkStorage&&) noexcept = default;

    /**
     * @brief Insert a chunk. If coord already exists, insertion is skipped.
     * @complexity O(1) average, O(n) worst case
     */
    void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) override;

    /**
     * @brief Find a chunk. Returns nullptr if not found.
     * @complexity O(1) average, O(n) worst case
     */
    ManagedChunk* find(const ChunkCoord& coord) override;

    /**
     * @brief Find a chunk (const). Returns nullptr if not found.
     * @complexity O(1) average, O(n) worst case
     */
    const ManagedChunk* find(const ChunkCoord& coord) const override;

    /**
     * @brief Check if chunk exists.
     * @complexity O(1) average, O(n) worst case
     */
    bool contains(const ChunkCoord& coord) const override;

    /**
     * @brief Erase a chunk by coordinate.
     * No-op if coordinate not found.
     * @complexity O(1) average, O(n) worst case
     */
    void erase(const ChunkCoord& coord) override;

    /**
     * @brief Clear all chunks and reset bucket count to ~27.
     */
    void clear() override;

    /**
     * @brief Get number of chunks stored.
     */
    size_t size() const override;

    /**
     * @brief Get current bucket count (number of hash buckets allocated).
     */
    size_t capacity() const override;

    /**
     * @brief Check if empty.
     */
    bool empty() const override;

    /**
     * @brief Iterate over all chunks (mutable).
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) override;

    /**
     * @brief Iterate over all chunks (const).
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const override;

    size_t getCapacityHint() const;

private:
    std::unordered_map<ChunkCoord, ManagedChunk, ChunkCoordHash> chunks_;
    size_t capacity_hint_{27};
};

/*
ChunkStorage implementation using std::deque.
Provides stable iterators and pointers during insertion (unlike vector).
Linear search like vector.
*/
class DequeChunkStorage final : public ChunkStorage
{
public:
    /**
     * @brief Default constructor
     */
    DequeChunkStorage() = default;

    /**
     * @brief Destructor
     */
    ~DequeChunkStorage() override = default;

    // Delete copy operations (ManagedChunk is move-only)
    DequeChunkStorage(const DequeChunkStorage&) = delete;
    DequeChunkStorage& operator=(const DequeChunkStorage&) = delete;

    // Default move operations
    DequeChunkStorage(DequeChunkStorage&&) noexcept = default;
    DequeChunkStorage& operator=(DequeChunkStorage&&) noexcept = default;

    /**
     * @brief Insert a chunk. If coord already exists, insertion is skipped.
     * @complexity O(n) where n = current size
     */
    void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) override;

    /**
     * @brief Find a chunk. Returns nullptr if not found.
     * @complexity O(n) linear search
     */
    ManagedChunk* find(const ChunkCoord& coord) override;

    /**
     * @brief Find a chunk (const). Returns nullptr if not found.
     * @complexity O(n) linear search
     */
    const ManagedChunk* find(const ChunkCoord& coord) const override;

    /**
     * @brief Check if chunk exists.
     * @complexity O(n) linear search
     */
    bool contains(const ChunkCoord& coord) const override;

    /**
     * @brief Erase a chunk by coordinate.
     * Uses swap-and-pop for O(1) erase (does not preserve order).
     * No-op if coordinate not found.
     * @complexity O(n) search + O(1) erase
     */
    void erase(const ChunkCoord& coord) override;

    /**
     * @brief Clear all chunks.
     */
    void clear() override;

    /**
     * @brief Get number of chunks stored.
     */
    size_t size() const override;

    /**
     * @brief Get current size (deque has no meaningful capacity concept).
     * @note std::deque does not expose capacity like std::vector.
     *       Returns current size as a proxy.
     */
    size_t capacity() const override;

    /**
     * @brief Check if empty.
     */
    bool empty() const override;

    /**
     * @brief Iterate over all chunks (mutable).
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) override;

    /**
     * @brief Iterate over all chunks (const).
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const override;

private:
    std::deque<std::pair<ChunkCoord, ManagedChunk>> chunks_;
};

} // namespace RollingBonxai