#include "carmaker_localization/state_buffer.h"

namespace carmaker_localization {

StateBuffer::StateBuffer(double duration_sec) : duration_(duration_sec) {
}

void StateBuffer::addFrame(const StateFrame& frame) {
    buffer_[frame.timestamp] = frame;
    while (!buffer_.empty() && (frame.timestamp - buffer_.begin()->first) > duration_) {
        buffer_.erase(buffer_.begin());
    }
}

bool StateBuffer::getFrameAt(double timestamp, StateFrame& frame) const {
    if (buffer_.empty()) return false;

    auto it = buffer_.lower_bound(timestamp);
    if (it == buffer_.end()) {
        it--;
    } else if (it != buffer_.begin() && std::abs(it->first - timestamp) > std::abs(std::prev(it)->first - timestamp)) {
        it--;
    }

    frame = it->second;
    return true;
}

std::vector<StateFrame> StateBuffer::getFramesSince(double timestamp) const {
    std::vector<StateFrame> frames;
    auto it = buffer_.upper_bound(timestamp);
    for (; it != buffer_.end(); ++it) {
        frames.push_back(it->second);
    }
    return frames;
}

void StateBuffer::clear() {
    buffer_.clear();
}

} // namespace carmaker_localization
