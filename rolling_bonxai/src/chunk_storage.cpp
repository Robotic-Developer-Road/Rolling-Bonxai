#include "rolling_bonxai/chunk_storage.hpp"
#include <algorithm>
#include <utility>

namespace RollingBonxai
{
    ////////////////////////////////////////////////////////
    //////////////////////Vector Impl///////////////////////
    ////////////////////////////////////////////////////////
    
    VectorChunkStorage::VectorChunkStorage(size_t capacity_hint) {
        // Reserve space minimally for 26 neighbours + 1 source
        capacity_hint_ = capacity_hint;
        chunks_.reserve(capacity_hint_);
    }
    
    void VectorChunkStorage::insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) {
        // Single-pass search: check if already exists
        for (const auto& [existing_coord, _] : chunks_) {
            if (existing_coord == coord) {
                return; // Already exists, skip insertion
            }
        }
        
        chunks_.emplace_back(coord, std::move(chunk_ob));
    }

    ManagedChunk* VectorChunkStorage::find(const ChunkCoord& coord) {
        // If empty just return nullptre
        if (chunks_.empty()) {return nullptr;}

        auto it = std::find_if(chunks_.begin(), chunks_.end(), 
            [&coord](const auto& entry) { return entry.first == coord; });
        
        return (it != chunks_.end()) ? &(it->second) : nullptr;
    }

    const ManagedChunk* VectorChunkStorage::find(const ChunkCoord& coord) const {
        // If empty just return nullptre
        if (chunks_.empty()) {return nullptr;}
        auto it = std::find_if(chunks_.begin(), chunks_.end(), 
            [&coord](const auto& entry) { return entry.first == coord; });
        
        return (it != chunks_.cend()) ? &(it->second) : nullptr;
    }

    bool VectorChunkStorage::contains(const ChunkCoord& coord) const {
        if (chunks_.empty()) {return false;}
        auto it = std::find_if(chunks_.begin(), chunks_.end(), 
            [&coord](const auto& entry) { return entry.first == coord; });
        
        return (it != chunks_.end());
    }

    void VectorChunkStorage::erase(const ChunkCoord& coord) {
        // Swap-and-pop erase: O(1) deletion after O(n) search
        // Check the last element first
        // If empty just return nullptr. next line with segfault if we are not careful
        if (chunks_.empty()) {return; /*a no op*/}

        if (chunks_.back().first == coord) {
            chunks_.pop_back();
            return;
        }

        // Search remaining elements
        for (size_t i = 0; i < chunks_.size() - 1; ++i) {
            if (chunks_[i].first == coord) {
                // Swap with back and pop
                chunks_[i] = std::move(chunks_.back());
                chunks_.pop_back();
                return;
            }
        }
        
        // If not found, no-op (silent failure as documented)
    }

    void VectorChunkStorage::clear() {
        chunks_.clear();
        chunks_.reserve(capacity_hint_);
    }

    size_t VectorChunkStorage::size() const {
        return chunks_.size();
    }

    size_t VectorChunkStorage::capacity() const {
        return chunks_.capacity();
    }

    bool VectorChunkStorage::empty() const {
        return chunks_.empty();
    }

    void VectorChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) {
        for (auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    void VectorChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const {
        for (const auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    size_t VectorChunkStorage::getCapacityHint() const {
        return capacity_hint_;
    }

    ////////////////////////////////////////////////////////
    //////////////////Unordered Map Impl////////////////////
    ////////////////////////////////////////////////////////
    
    HashMapChunkStorage::HashMapChunkStorage(size_t capacity_hint) {
        // Reserve space minimally for 26 neighbours + 1 source
        capacity_hint_ = capacity_hint;
        chunks_.reserve(capacity_hint_);
    }
    
    void HashMapChunkStorage::insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) {
        // try_emplace: only inserts if key doesn't exist
        chunks_.try_emplace(coord, std::move(chunk_ob));
    }

    ManagedChunk* HashMapChunkStorage::find(const ChunkCoord& coord) {
        if (chunks_.empty()) {return nullptr;}
        auto it = chunks_.find(coord);
        return (it != chunks_.end()) ? &(it->second) : nullptr;
    }

    const ManagedChunk* HashMapChunkStorage::find(const ChunkCoord& coord) const {
        if (chunks_.empty()) {return nullptr;}
        auto it = chunks_.find(coord);
        return (it != chunks_.end()) ? &(it->second) : nullptr;
    }

    bool HashMapChunkStorage::contains(const ChunkCoord& coord) const {
        if (chunks_.empty()) {return false;}
        return chunks_.find(coord) != chunks_.end();
    }

    void HashMapChunkStorage::erase(const ChunkCoord& coord) {
        // unordered_map::erase handles "not found" gracefully (no-op)
        if (chunks_.empty()) {return/*no op*/;}
        chunks_.erase(coord);
    }

    void HashMapChunkStorage::clear() {
        chunks_.clear();
        chunks_.reserve(capacity_hint_);
    }

    size_t HashMapChunkStorage::size() const {
        return chunks_.size();
    }

    size_t HashMapChunkStorage::capacity() const {
        // Return number of buckets allocated
        return chunks_.bucket_count();
    }

    bool HashMapChunkStorage::empty() const {
        return chunks_.empty();
    }

    void HashMapChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) {
        for (auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    void HashMapChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const {
        for (const auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    size_t HashMapChunkStorage::getCapacityHint() const {
        return capacity_hint_;
    }

    ////////////////////////////////////////////////////////
    //////////////////////Deque Impl////////////////////////
    ////////////////////////////////////////////////////////
    
    void DequeChunkStorage::insert(const ChunkCoord& coord, ManagedChunk&& chunk_ob) {
        // Single-pass search: check if already exists
        for (const auto& [existing_coord, _] : chunks_) {
            if (existing_coord == coord) {
                return; // Already exists, skip insertion
            }
        }
        
        chunks_.emplace_back(coord, std::move(chunk_ob));
    }

    ManagedChunk* DequeChunkStorage::find(const ChunkCoord& coord) {
        if (chunks_.empty()) {return nullptr;}
        auto it = std::find_if(chunks_.begin(), chunks_.end(), 
            [&coord](const auto& entry) { return entry.first == coord; });
        
        return (it != chunks_.end()) ? &(it->second) : nullptr;
    }

    const ManagedChunk* DequeChunkStorage::find(const ChunkCoord& coord) const {
        if (chunks_.empty()) {return nullptr;}
        auto it = std::find_if(chunks_.begin(), chunks_.end(), 
            [&coord](const auto& entry) { return entry.first == coord; });
        
        return (it != chunks_.cend()) ? &(it->second) : nullptr;
    }

    bool DequeChunkStorage::contains(const ChunkCoord& coord) const {
        if (chunks_.empty()) {return false;}
        auto it = std::find_if(chunks_.begin(), chunks_.end(), 
            [&coord](const auto& entry) { return entry.first == coord; });
        
        return (it != chunks_.end());
    }

    void DequeChunkStorage::erase(const ChunkCoord& coord) {
        // Swap-and-pop erase: O(1) deletion after O(n) search
        // Check the last element first
        if (chunks_.empty()) {return; /*a no op*/}
        
        if (chunks_.back().first == coord) {
            chunks_.pop_back();
            return;
        }

        // Search remaining elements
        for (size_t i = 0; i < chunks_.size() - 1; ++i) {
            if (chunks_[i].first == coord) {
                // Swap with back and pop
                chunks_[i] = std::move(chunks_.back());
                chunks_.pop_back();
                return;
            }
        }
        
        // If not found, no-op (silent failure as documented)
    }

    void DequeChunkStorage::clear() {
        chunks_.clear();
    }

    size_t DequeChunkStorage::size() const {
        return chunks_.size();
    }

    size_t DequeChunkStorage::capacity() const {
        // Deque has no meaningful capacity; return current size
        return chunks_.size();
    }

    bool DequeChunkStorage::empty() const {
        return chunks_.empty();
    }

    void DequeChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, ManagedChunk&)>& fn
    ) {
        for (auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }

    void DequeChunkStorage::forEachChunk(
        const std::function<void(const ChunkCoord&, const ManagedChunk&)>& fn
    ) const {
        for (const auto& [coord, chunk] : chunks_) {
            fn(coord, chunk);
        }
    }
    
} // namespace RollingBonxai