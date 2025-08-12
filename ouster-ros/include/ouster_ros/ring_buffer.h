#pragma once
#include <deque>

template <typename T>
class RingBuffer {
  public:
    explicit RingBuffer(std::size_t capacity = 100) : capacity_(capacity) {}

    void push_back(const T& item) {
        if(buffer_.size() >= capacity_) {
            buffer_.pop_front();
        }
        buffer_.push_back(item);
    }

    void push_back(T&& item) {
        if(buffer_.size() >= capacity_) {
            buffer_.pop_front();
        }
        buffer_.push_back(std::move(item));
    }

    void clear() {
        buffer_.clear();
    }

    std::size_t size() const {
        return buffer_.size();
    }

    bool empty() const {
        return buffer_.empty();
    }

    const T& operator[](std::size_t index) const {
        return buffer_[index];
    }

    const T& back() const {
        return buffer_.back();
    }

    const T& front() const {
        return buffer_.front();
    }

    // Iterator support
    auto begin() const {
        return buffer_.begin();
    }
    auto end() const {
        return buffer_.end();
    }
    auto begin() {
        return buffer_.begin();
    }
    auto end() {
        return buffer_.end();
    }

    void set_capacity(std::size_t new_capacity) {
        capacity_ = new_capacity;
        while(buffer_.size() > capacity_) {
            buffer_.pop_front();
        }
    }

    std::size_t capacity() const {
        return capacity_;
    }

  private:
    std::deque<T> buffer_;
    std::size_t capacity_;
};