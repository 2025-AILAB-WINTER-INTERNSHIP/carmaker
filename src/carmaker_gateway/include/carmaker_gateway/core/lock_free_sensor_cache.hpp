#ifndef CARMAKER_GATEWAY_CORE_LOCK_FREE_SENSOR_CACHE_HPP
#define CARMAKER_GATEWAY_CORE_LOCK_FREE_SENSOR_CACHE_HPP

#include <memory>
#include <atomic>

namespace carmaker_gateway {

/**
 * @brief Thread-safe, lock-free cache for a single sensor data sample.
 * This class ensures that the I/O thread can update data while the
 * Sync Timer thread reads it without mutex contention, using atomic pointer swapping.
 */
template <typename T>
class LockFreeSensorCache {
public:
    LockFreeSensorCache() : cache_ptr_(std::make_shared<const T>()) {}

    /**
     * @brief Update the cache with new data (called from I/O thread).
     * Uses std::atomic_store to ensure the pointer swap is atomic and visible.
     */
    void Update(std::unique_ptr<T> new_data) {
        // Promote to shared_ptr<const T> to enforce immutability
        std::shared_ptr<const T> updated_ptr = std::move(new_data);
        std::atomic_store_explicit(&cache_ptr_, updated_ptr, std::memory_order_release);
    }

    /**
     * @brief Overload for shared_ptr input.
     */
    void Update(std::shared_ptr<const T> new_data) {
        std::atomic_store_explicit(&cache_ptr_, std::move(new_data), std::memory_order_release);
    }

    /**
     * @brief Retrieve the latest cached data (called from Sync thread).
     * @return A shared pointer to the latest data, ensuring zero-copy access.
     */
    std::shared_ptr<const T> GetLatest() const {
        return std::atomic_load_explicit(&cache_ptr_, std::memory_order_acquire);
    }

private:
    // Atomic pointer to the latest data sample
    std::shared_ptr<const T> cache_ptr_;
};

} // namespace carmaker_gateway

#endif // CARMAKER_GATEWAY_CORE_LOCK_FREE_SENSOR_CACHE_HPP
