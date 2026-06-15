# C++ & ROS Autonomous Driving Coding Rules

Strictly enforce these constraints when generating or editing C++ and ROS code. If a user's request violates these rules, auto-correct the code to comply and briefly note the correction.

## 1. Architecture & Core Isolation

- **Pure C++ Core**: Core algorithms must not `#include <ros/ros.h>` or use ROS loggers. Return statuses/warnings via structs (e.g., `struct Result { bool success; std::vector<std::string> warnings; };`).
- **SRP Callbacks**: Subscriber callbacks must only enqueue/store data under a lock. Delegate all algorithmic processing to separate member functions.
- **Smart Pointers**: Direct `new`/`delete` is forbidden. Use `std::make_unique` or `std::make_shared` (RAII).

## 2. Robustness & Temporal Safety

- **Predicate Returns**: Functions that can fail must return `bool` and output results via reference (`&`). Do not `throw` exceptions in real-time loops.
- **Transactional Structs**: Return aggregated results (data, status, duration, warnings) in a single struct to prevent race conditions.
- **Null Guards**: Always prefix ROS callback logic with a pointer check: `if (!msg) return;` (or use `_THROTTLE` warnings).
- **Time Jump Reset**: Monitor `dt`. If `dt < 0.0` (time jump backward), immediately call `resetFilter()` or clear state buffers.

## 3. Real-Time Performance & Multithreading

- **Lock-free Flags**: Use `std::atomic<T>` for scalar status/counters instead of mutexes.
- **Fine-grained Locks**: Do not lock an entire class. Use specific mutexes for specific shared resources.
- **Async Workers**: Run heavy algorithms (e.g., ICP, A*) in background threads using `std::queue` and `std::condition_variable`.
- **Static Eigen Matrices**: Real-time matrix math must use fixed-size templates (e.g., `Eigen::Matrix<double, 6, 6>`). Never use dynamic `Eigen::MatrixXd`.
- **Scoped Locks**: Restrict lock scope using `{}` blocks. Copy data quickly and exit the lock before calling heavy operations.

## 4. Networking & Configuration

- **tcpNoDelay**: High-frequency subscribers (images, lidar, >=100Hz) must use `ros::TransportHints().tcpNoDelay()` and queue size = 1 or 2.
- **Throttled Logging**: High-frequency loops (>= 10Hz) must explicitly use throttled loggers (e.g., `ROS_WARN_THROTTLE(1.0, ...)`).
- **YAML Config**: No hardcoding of topics or parameters. Fetch securely via `ros::NodeHandle::param`. Standardize hierarchy: `vehicle/`, `[module]/`, `topics/`.
