#ifndef CARMAKER_GATEWAY_CORE_LOCK_FREE_TIME_RING_BUFFER_HPP
#define CARMAKER_GATEWAY_CORE_LOCK_FREE_TIME_RING_BUFFER_HPP

#include <array>
#include <atomic>
#include <memory>
#include <vector>
#include <limits>
#include <cmath>

namespace carmaker_gateway {

/**
 * @brief Extractor to retrieve Timestamp from ROS message headers.
 */
template <typename T>
struct TimestampExtractor {
    double operator()(const T& data) const {
        return data.header.stamp.toSec();
    }
};

/**
 * @brief Extractor to retrieve WallTime from CarMaker messages.
 */
template <typename T>
struct WallTimeExtractor {
    double operator()(const T& data) const {
        return data.time.toSec();
    }
};

/**
 * @brief Lock-free circular buffer for time-based data synchronization.
 * Maintains a fixed-size history of data samples, allowing readers to
 * find the best match for a given timestamp without blocking writers.
 */
template <typename T, size_t Size = 5, typename Extractor = TimestampExtractor<T>>
class LockFreeTimeRingBuffer {
public:
    LockFreeTimeRingBuffer() : write_index_(0) {
        for (auto& slot : buffer_) {
            slot = nullptr;
        }
    }

    /**
     * @brief Push new data into the ring buffer (called from I/O thread).
     */
    void Push(std::unique_ptr<T> new_data) {
        std::shared_ptr<const T> shared_data = std::move(new_data);
        Push(shared_data);
    }

    void Push(std::shared_ptr<const T> shared_data) {
        // Increment global write index atomically
        uint64_t current_idx = write_index_.fetch_add(1, std::memory_order_relaxed);

        // Atomic store into the buffer slot to ensure visibility for readers
        std::atomic_store_explicit(&buffer_[current_idx % Size], shared_data, std::memory_order_release);
    }

    /**
     * @brief Find the sample closest to the target timestamp (called from Sync thread).
     * This implementation is zero-allocation and avoids heap overhead in the hot path.
     * @param anchor_timestamp The reference time for synchronization.
     * @return Best matching sample found in the current buffer snapshot.
     */
    std::shared_ptr<const T> GetBestMatch(double anchor_timestamp) const {
        std::shared_ptr<const T> best_match = nullptr;
        double min_diff = std::numeric_limits<double>::max();
        Extractor extract_ts;

        // Iterate through the buffer slots and atomically load pointers.
        // We find the one with the minimum absolute time difference.
        for (size_t i = 0; i < Size; ++i) {
            auto ptr = std::atomic_load_explicit(&buffer_[i], std::memory_order_acquire);
            if (!ptr) continue;

            double ts = extract_ts(*ptr);
            double diff = std::abs(ts - anchor_timestamp);
            if (diff < min_diff) {
                min_diff = diff;
                best_match = std::move(ptr);
            }
        }

        return best_match;
    }

    /**
     * @brief Get the latest pushed sample (O(1)).
     */
    std::shared_ptr<const T> GetLatest() const {
        uint64_t latest_idx = write_index_.load(std::memory_order_relaxed);
        if (latest_idx == 0) return nullptr;
        // Ensure index is within bounds and use acquire memory order for visibility.
        return std::atomic_load_explicit(&buffer_[(latest_idx - 1) % Size], std::memory_order_acquire);
    }

private:
    std::atomic<uint64_t> write_index_{0};

    // Buffer of shared pointers. Atomic load/store are used for thread safety.
    // In C++17, std::atomic_load/store on shared_ptr are the standard way.
    mutable std::array<std::shared_ptr<const T>, Size> buffer_;
};

} // namespace carmaker_gateway

#endif // CARMAKER_GATEWAY_CORE_LOCK_FREE_TIME_RING_BUFFER_HPP
