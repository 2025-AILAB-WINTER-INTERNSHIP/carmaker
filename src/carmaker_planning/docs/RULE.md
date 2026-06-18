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

## 5. Code Hygiene & Debt Management

- **Ghost Code Evaluation**: Do not blindly delete commented-out code blocks (e.g., `#if 0` or `//`) or unused functions; evaluate their intent first. If the code is truly obsolete, delete it immediately (rely on Git for history). If the code represents a valid alternative or optional feature, do not leave it as dead text. You must formalize it into a configurable ROS parameter, a boolean feature flag, or a properly documented `#ifdef` block. Raw, undocumented dead code is strictly prohibited.
- **Tracked Technical Debt**: If a temporary hack or structural workaround is unavoidable, you must document it strictly using the format `// TODO([Ticket-ID] or [Name]): [Clear reason and resolution plan]`. Untracked TODOs or undocumented structural flaws must be rejected during code review.
- **Boy Scout Rule**: When modifying existing code or adding features, always leave the module cleaner than you found it. Regularly pay off minor technical debts incurred for rapid deployment to prevent them from compounding into massive architectural failures.

## 6. Minimal Code, Maximal Impact

- **Standard Algorithms Over Custom Loops**: Prioritize C++ standard algorithms (`<algorithm>`) over verbose and error-prone manual `for` loops. Using `std::find_if`, `std::transform`, `std::erase_if`, etc., drastically reduces lines of code while maximizing both readability and compiler optimization.
- **Preallocation Guards**: Dynamic memory reallocation causes severe latency spikes in real-time threads. When initializing containers like `std::vector` or `std::unordered_map`, you must determine the maximum or expected capacity and explicitly call `.reserve()`.
- **DRY & Zero-Copy**: Do not duplicate operational or data-conversion logic; extract them into inline utility functions. Enforce the use of `ConstPtr` and pass-by-reference (`const Type&`) to prevent unnecessary data copying during inter-node message processing, achieving maximum execution efficiency with minimal code.
- **KISS (Keep It Simple, Stupid)**: Prohibit overly complex template metaprogramming or obscure syntactic tricks unless strictly required for real-time performance. Code must be written simply and intuitively so that it can be read and maintained by humans before it is executed by machines.
