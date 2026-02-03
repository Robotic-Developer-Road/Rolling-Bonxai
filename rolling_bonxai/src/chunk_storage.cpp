#include "rolling_bonxai/chunk_storage.hpp"
#include <algorithm>
#include <utility>
namespace RollingBonxai
{
    ////////////////////////////////////////////////////////
    //////////////////////Vector Impl///////////////////////
    ////////////////////////////////////////////////////////
    VectorChunkStorage::VectorChunkStorage() {
        // reserve space minimally for 26 neighbours + 1 source
        chunks_.reserve(27);
    }
    
    void VectorChunkStorage::insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) {
        bool already_contains = contains(coord);
        if (already_contains) {
            return;
        }
        else {
            chunks_.emplace_back(coord,std::move(chunk_ob));
        }
    }

    ManagedChunk* VectorChunkStorage::find(const ChunkCoord& coord) {
        auto it = std::find_if(chunks_.begin(),chunks_.end(),[&coord](const auto& entry_pair)
        {
            return entry_pair.first == coord;
        });
        
        return (it != chunks_.end()) ? &(it->second) : nullptr;
    }

    const ManagedChunk* VectorChunkStorage::find(const ChunkCoord& coord) const {
        auto it = std::find_if(chunks_.begin(),chunks_.end(),[&coord](const auto& entry_pair)
        {
            return entry_pair.first == coord;
        });
        
        return (it != chunks_.cend()) ? &(it->second) : nullptr;
    }

    bool VectorChunkStorage::contains(const ChunkCoord& coord) const {
        auto it = std::find_if(chunks_.begin(),chunks_.end(),[&coord](const auto& entry_pair)
        {
            return entry_pair.first == coord;
        });
        
        bool found = (it != chunks_.end());

        return found;
    }

    void VectorChunkStorage::erase(const ChunkCoord& coord) {
        // implements a swap-pop erase because order doesnt really matter
        // Check the last element
        if (chunks_.back().first == coord) {
            chunks_.pop_back();
            return;
        }

        for (size_t i = 0 ; i < chunks_.size() - 1 ; ++i) {
            if (chunks_[i].first == coord) {
                // this prevents any self moves where chunks[i] == chunks.back()
                chunks_[i] = std::move(chunks_.back());
                chunks_.pop_back();
            }
        }
    }

    void VectorChunkStorage::clear() {
        chunks_.clear();
        chunks_.reserve(27);
    }

    size_t VectorChunkStorage::size() {
        return chunks_.size();
    }

    size_t VectorChunkStorage::capacity() {
        return chunks_.capacity();
    }

    bool VectorChunkStorage::empty() {
        return chunks_.empty();
    }

    void VectorChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    )
    {
        for (auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    void VectorChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const
    {
        for (const auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    ////////////////////////////////////////////////////////
    //////////////////Unordered Map Impl////////////////////
    ////////////////////////////////////////////////////////
    HashMapChunkStorage::HashMapChunkStorage() {
        // reserve space minimally for 26 neighbours + 1 source
        chunks_.reserve(27);
    }
    
    void HashMapChunkStorage::insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) {
        chunks_.try_emplace(coord,std::move(chunk_ob));
    }

    ManagedChunk* HashMapChunkStorage::find(const ChunkCoord& coord) {
        auto it = chunks_.find(coord);
        if (it != chunks_.end()) {
            return &(it->second);
        }
        else {
            return nullptr;
        }
    }

    const ManagedChunk* HashMapChunkStorage::find(const ChunkCoord& coord) const {
        auto it = chunks_.find(coord);
        if (it != chunks_.end()) {
            return &(it->second);
        }
        else {
            return nullptr;
        }
    }

    bool HashMapChunkStorage::contains(const ChunkCoord& coord) const {
        return chunks_.find(coord) != chunks_.end();
    }

    void HashMapChunkStorage::erase(const ChunkCoord& coord) {
        auto it = chunks_.find(coord);
        if (it != chunks_.end()) {
            chunks_.erase(it);
        }
    }

    void HashMapChunkStorage::clear() {
        chunks_.clear();
        chunks_.reserve(27);
    }

    size_t HashMapChunkStorage::size() {
        return chunks_.size();
    }

    size_t HashMapChunkStorage::capacity() {
        return chunks_.bucket_count() * chunks_.max_load_factor();
    }

    bool HashMapChunkStorage::empty() {
        return chunks_.empty();
    }

    void HashMapChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    )
    {
        for (auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    void HashMapChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const
    {
        for (const auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    ////////////////////////////////////////////////////////
    //////////////////////Deque Impl///////////////////////
    ////////////////////////////////////////////////////////
    
    void DequeChunkStorage::insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) {
        bool already_contains = contains(coord);
        if (already_contains) {
            return;
        }
        else {
            chunks_.emplace_back(coord,std::move(chunk_ob));
        }
    }

    ManagedChunk* DequeChunkStorage::find(const ChunkCoord& coord) {
        auto it = std::find_if(chunks_.begin(),chunks_.end(),[&coord](const auto& entry_pair)
        {
            return entry_pair.first == coord;
        });
        
        return (it != chunks_.end()) ? &(it->second) : nullptr;
    }

    const ManagedChunk* DequeChunkStorage::find(const ChunkCoord& coord) const {
        auto it = std::find_if(chunks_.begin(),chunks_.end(),[&coord](const auto& entry_pair)
        {
            return entry_pair.first == coord;
        });
        
        return (it != chunks_.cend()) ? &(it->second) : nullptr;
    }

    bool DequeChunkStorage::contains(const ChunkCoord& coord) const {
        auto it = std::find_if(chunks_.begin(),chunks_.end(),[&coord](const auto& entry_pair)
        {
            return entry_pair.first == coord;
        });
        
        bool found = (it != chunks_.end());

        return found;
    }

    void DequeChunkStorage::erase(const ChunkCoord& coord) {
        // implements a swap-pop erase because order doesnt really matter
        // Check the last element
        if (chunks_.back().first == coord) {
            chunks_.pop_back();
            return;
        }

        for (size_t i = 0 ; i < chunks_.size() - 1 ; ++i) {
            if (chunks_[i].first == coord) {
                // this prevents any self moves where chunks[i] == chunks.back()
                chunks_[i] = std::move(chunks_.back());
                chunks_.pop_back();
            }
        }
    }

    void DequeChunkStorage::clear() {
        chunks_.clear();
    }

    size_t DequeChunkStorage::size() {
        return chunks_.size();
    }

    size_t DequeChunkStorage::capacity() {
        return chunks_.max_size();
    }

    bool DequeChunkStorage::empty() {
        return chunks_.empty();
    }

    void DequeChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    )
    {
        for (auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    void DequeChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const
    {
        for (const auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }
}