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
3. erase() > Deletes a ManagedChunk with a given 
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
     * @brief
     */
    virtual void insert(const ChunkCoord& coord,ManagedChunk&& chunk_ob) = 0;

    /**
     * @brief
     */
    virtual ManagedChunk* find(const ChunkCoord& coord) = 0;
    virtual const ManagedChunk* find(const ChunkCoord& coord) const = 0;

    /**
     * @brief
     */
    virtual bool contains(const ChunkCoord& coord) const = 0;

    /**
     * @brief 
     */
    virtual void erase(const ChunkCoord& coord) = 0;

    /**
     * @brief
     */
    virtual void clear() = 0;

    /**
     * @brief
     */
    virtual size_t size() = 0;

    /**
     * @brief
     */
    virtual size_t capacity() = 0;

    /**
     * @brief
     */
    virtual bool empty() = 0;

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
ChunkStorage implementation using std::vector
*/
class VectorChunkStorage final : public ChunkStorage
{
public:
    /**
     * @brief
     */
    VectorChunkStorage();

    /**
     * @brief
     */
    ~VectorChunkStorage() override = default;

    /**
     * @brief
     */
    void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) override;

    /**
     * @brief
     */
    ManagedChunk* find(const ChunkCoord& coord) override;

    /**
     * @brief
     */
    const ManagedChunk* find(const ChunkCoord& coord) const override;

    /**
     * @brief
     */
    bool contains(const ChunkCoord& coord) const override;

    /**
     * @brief
     */
    void erase(const ChunkCoord& coord) override;

    /**
     * @brief
     */
    void clear() override;

    /**
     * @brief
     */
    size_t size() override;

    /**
     * @brief
     */
    size_t capacity() override;

    /**
     * @brief
     */
    bool empty() override;

    /**
     * @brief
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) override;

    /**
     * @brief
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const override;

private:
    std::vector<std::pair<ChunkCoord, ManagedChunk>> chunks_;
};

/*
ChunkStorage implementation using std::unordered_map
*/
class HashMapChunkStorage final : public ChunkStorage
{
public:
    /**
     * @brief
     */
    HashMapChunkStorage();

    /**
     * @brief
     */
    ~HashMapChunkStorage() override = default;

    /**
     * @brief
     */
    void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) override;

    /**
     * @brief
     */
    ManagedChunk* find(const ChunkCoord& coord) override;

    /**
     * @brief
     */
    const ManagedChunk* find(const ChunkCoord& coord) const override;

    /**
     * @brief
     */
    bool contains(const ChunkCoord& coord) const override;

    /**
     * @brief
     */
    void erase(const ChunkCoord& coord) override;

    /**
     * @brief
     */
    void clear() override;

    /**
     * @brief
     */
    size_t size() override;

    /**
     * @brief
     */
    size_t capacity() override;

    /**
     * @brief
     */
    bool empty() override;

    /**
     * @brief
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) override;

    /**
     * @brief
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const override;

private:
    std::unordered_map<ChunkCoord, ManagedChunk, ChunkCoordHash> chunks_;
};


class DequeChunkStorage final : public ChunkStorage
{
public:
    /**
     * @brief
     */
    DequeChunkStorage() = default;

    /**
     * @brief
     */
    ~DequeChunkStorage() override = default;

    /**
     * @brief
     */
    void insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) override;

    /**
     * @brief
     */
    ManagedChunk* find(const ChunkCoord& coord) override;

    /**
     * @brief
     */
    const ManagedChunk* find(const ChunkCoord& coord) const override;

    /**
     * @brief
     */
    bool contains(const ChunkCoord& coord) const override;

    /**
     * @brief
     */
    void erase(const ChunkCoord& coord) override;

    /**
     * @brief
     */
    void clear() override;

    /**
     * @brief
     */
    size_t size() override;

    /**
     * @brief
     */
    size_t capacity() override;

    /**
     * @brief
     */
    bool empty() override;

    /**
     * @brief
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) override;

    /**
     * @brief
     */
    void forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const override;

private:
    std::deque<std::pair<ChunkCoord, ManagedChunk>> chunks_;
};

}