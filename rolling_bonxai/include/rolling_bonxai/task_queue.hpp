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

/**
 * @brief Thread-safe priority queue (max-heap by default)
 * 
 * Wraps std::priority_queue with mutex + condition variable for safe concurrent access.
 * Exposes the same API surface as ThreadSafeQueue for consistency.
 * 
 * @tparam T Element type (must be movable and have operator< defined)
 * @tparam Compare Comparison functor (default: std::less<T>, which gives max-heap)
 * 
 * @thread_safety All public methods are thread-safe.
 * 
 * @note std::priority_queue is a max-heap by default. The element with the
 *       highest priority (as defined by Compare) is popped first.
 *       For LoadRequest where operator< gives reverse ordering for max-heap,
 *       use the default std::less<T>.
 * 
 * @note Call shutdown() to unblock all waiting threads before destruction.
 */
template <typename T, typename Compare = std::less<T>>
class ThreadSafePriorityQueue
{
public:
    ThreadSafePriorityQueue() = default;
    ~ThreadSafePriorityQueue() = default;

    // Non-copyable, non-movable
    ThreadSafePriorityQueue(const ThreadSafePriorityQueue&) = delete;
    ThreadSafePriorityQueue& operator=(const ThreadSafePriorityQueue&) = delete;
    ThreadSafePriorityQueue(ThreadSafePriorityQueue&&) = delete;
    ThreadSafePriorityQueue& operator=(ThreadSafePriorityQueue&&) = delete;

    /**
     * @brief Push an item into the priority queue
     * 
     * @param item Item to enqueue (moved)
     * @throws std::runtime_error if queue has been shut down
     * 
     * @complexity O(log n)
     */
    void push(T item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) {
                throw std::runtime_error("push() called on shut down ThreadSafePriorityQueue");
            }
            queue_.push(std::move(item));
        }
        condition_.notify_one();
    }

    /**
     * @brief Blocking pop - waits until an item is available or shutdown
     * 
     * Returns the highest-priority item (as defined by Compare).
     * 
     * @return Item if available, std::nullopt if queue was shut down
     * 
     * @note Blocks calling thread until an item is available or shutdown() is called.
     * @complexity O(log n) (excluding wait time)
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

        // std::priority_queue::top() returns const T& by design (to protect heap invariant).
        // We copy rather than const_cast + move to avoid undefined behavior.
        // This is acceptable because LoadRequest members are cheap to copy.
        T item = queue_.top();
        queue_.pop();
        return item;
    }

    /**
     * @brief Non-blocking pop attempt
     * 
     * @return Highest-priority item if available, std::nullopt if empty or shut down
     * 
     * @complexity O(log n)
     */
    std::optional<T> tryPop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty() || shutdown_) {
            return std::nullopt;
        }

        T item = queue_.top();
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
    std::priority_queue<T, std::vector<T>, Compare> queue_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool shutdown_{false};
};

} // namespace RollingBonxai