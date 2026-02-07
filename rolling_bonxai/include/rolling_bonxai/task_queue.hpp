#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <atomic>
#include <stdexcept>

namespace RollingBonxai
{

/**
 * @brief Thread-safe FIFO queue
 * 
 * Wraps std::queue with mutex + condition variable for safe concurrent access.
 * Supports blocking pop (waits for item) and non-blocking tryPop.
 * 
 * @tparam T Element type (must be movable)
 * 
 * @thread_safety All public methods are thread-safe.
 * 
 * @note Call shutdown() to unblock all waiting threads before destruction.
 *       After shutdown, push() throws and pop()/tryPop() return std::nullopt.
 */
template <typename T>
class ThreadSafeQueue
{
public:
    ThreadSafeQueue() = default;
    ~ThreadSafeQueue() = default;

    // Non-copyable, non-movable (mutex is not movable)
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue(ThreadSafeQueue&&) = delete;
    ThreadSafeQueue& operator=(ThreadSafeQueue&&) = delete;

    /**
     * @brief Push an item into the queue
     * 
     * @param item Item to enqueue (moved)
     * @throws std::runtime_error if queue has been shut down
     * 
     * @complexity O(1)
     */
    void push(T item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) {
                throw std::runtime_error("push() called on shut down ThreadSafeQueue");
            }
            queue_.push(std::move(item));
        }
        condition_.notify_one();
    }

    /**
     * @brief Blocking pop - waits until an item is available or shutdown
     * 
     * @return Item if available, std::nullopt if queue was shut down
     * 
     * @note Blocks calling thread until an item is available or shutdown() is called.
     * @complexity O(1) (excluding wait time)
     */
    std::optional<T> pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {
            return !queue_.empty() || shutdown_;
        });

        if (shutdown_ && queue_.empty()) {
            return std::nullopt;
        }

        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    /**
     * @brief Non-blocking pop attempt
     * 
     * @return Item if available, std::nullopt if queue is empty or shut down
     * 
     * @complexity O(1)
     */
    std::optional<T> tryPop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty() || shutdown_) {
            return std::nullopt;
        }

        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    /**
     * @brief Check if queue is empty
     * 
     * @return true if queue has no items
     * @note Result may be stale by the time caller acts on it.
     * 
     * @complexity O(1)
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief Get current queue size
     * 
     * @return Number of items in queue
     * @note Result may be stale by the time caller acts on it.
     * 
     * @complexity O(1)
     */
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /**
     * @brief Shut down the queue, unblocking all waiting threads
     * 
     * After shutdown:
     * - All blocked pop() calls return std::nullopt
     * - Future push() calls throw std::runtime_error
     * - tryPop() returns std::nullopt
     * 
     * @note This is irreversible. Call before destroying the queue
     *       if any threads may be blocked on pop().
     */
    void shutdown()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        condition_.notify_all();
    }

    /**
     * @brief Check if queue has been shut down
     * 
     * @return true if shutdown() has been called
     */
    bool isShutdown() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return shutdown_;
    }

private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool shutdown_{false};
};
} // namespace RollingBonxai