#ifndef CARMAKER_GATEWAY_CORE_MONOTONIC_ANCHOR_CACHE_HPP
#define CARMAKER_GATEWAY_CORE_MONOTONIC_ANCHOR_CACHE_HPP

#include "lock_free_time_ring_buffer.hpp"
#include <atomic>
#include <memory>

namespace carmaker_gateway {

/**
 * @brief Thread-safe cache for the synchronization anchor (clock reference).
 * Enforces monotonicity of incoming timestamps to prevent out-of-order
 * data processing and manages state resets during virtual heartbeats.
 */
template <typename T, typename Extractor = TimestampExtractor<T>>
class MonotonicAnchorCache {
public:
    MonotonicAnchorCache() {}

    /**
     * @brief Signal that the anchor signal has dropped and virtual heartbeat is active.
     */
    void NotifyVirtualHeartbeatTriggered() {
        is_in_virtual_heartbeat_.store(true, std::memory_order_release);
    }

    /**
     * @brief Add new anchor data to the cache (called from I/O thread).
     * @return True if a hard resynchronization (clock reset) occurred.
     */
    bool Push(std::shared_ptr<const T> new_data) {
        Extractor extract_ts;
        double current_packet_time = extract_ts(*new_data);

        // 1. Check for Resynchronization (Recovery from heartbeat or time jump)
        if (is_in_virtual_heartbeat_.load(std::memory_order_acquire)) {
            PerformResync(current_packet_time);
            ring_buffer_.Push(new_data);
            return true;
        }

        // 2. Enforce Monotonicity (Discard out-of-order packets)
        double last_time = latest_processed_timestamp_.load(std::memory_order_relaxed);
        if (current_packet_time <= last_time) {
            drop_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        // 3. Update the global clock reference and push to buffer
        UpdateLatestTime(current_packet_time);
        ring_buffer_.Push(new_data);
        return false;
    }

    /**
     * @brief Find best match in history.
     */
    std::shared_ptr<const T> GetBestMatch(double anchor_timestamp) const {
        return ring_buffer_.GetBestMatch(anchor_timestamp);
    }

    /**
     * @brief Get the number of dropped (out-of-order) packets since last call.
     */
    uint64_t GetAndResetDropCount() {
        return drop_count_.exchange(0, std::memory_order_relaxed);
    }

    /**
     * @brief Get the latest data sample regardless of timestamp (used for state monitoring).
     */
    std::shared_ptr<const T> GetMostRecent() const {
        return ring_buffer_.GetLatest();
    }

private:
    /**
     * @brief Forcibly reset the internal clock to a new physical time.
     */
    void PerformResync(double new_physical_time) {
        latest_processed_timestamp_.store(new_physical_time, std::memory_order_release);
        is_in_virtual_heartbeat_.store(false, std::memory_order_release);
    }

    /**
     * @brief Atomically update the latest timestamp if the new value is greater.
     */
    void UpdateLatestTime(double new_time) {
        double expected = latest_processed_timestamp_.load(std::memory_order_relaxed);
        while (new_time > expected && !latest_processed_timestamp_.compare_exchange_weak(expected, new_time, std::memory_order_release, std::memory_order_relaxed)) {
        }
    }

    // [Future Work] In C++20, consider using std::atomic::wait() and notify_one() on
    // latest_processed_timestamp_ to enable efficient event-driven wakeups for
    // downstream consumers without the overhead of condition variables or polling.
    std::atomic<double> latest_processed_timestamp_{0.0};
    std::atomic<bool> is_in_virtual_heartbeat_{false};
    std::atomic<uint64_t> drop_count_{0};
    LockFreeTimeRingBuffer<T, 20, Extractor> ring_buffer_;
};

} // namespace carmaker_gateway

#endif // CARMAKER_GATEWAY_CORE_MONOTONIC_ANCHOR_CACHE_HPP
