/**
 * Copyright (c) 2018-2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file thread_safe_ring_buffer.h
 * @brief File contains thread safe implementation of a ring buffer
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <vector>
#include <atomic>

/**
 * @class LockFreeRingBuffer thread safe ring buffer.
 * 
 * @remarks current implementation has effective (capacity-1) when writing elements
 */
class LockFreeRingBuffer {
   public:
    LockFreeRingBuffer(size_t capacity)
        : capacity_(capacity),
          write_idx_(0),
          read_idx_(0) {}

    /**
     * Gets the maximum number of items that this ring buffer can hold.
     */
    size_t capacity() const { return capacity_; }

    /**
     * Gets the number of item that currently occupy the ring buffer. This
     * number would vary between 0 and the capacity().
     *
     * @remarks
     *  if returned value was 0 or the value was equal to the buffer capacity(),
     *  this does not guarantee that a subsequent call to read() or write()
     *  wouldn't cause the calling thread to be blocked.
     */
    size_t size() const {
        return write_idx_ >= read_idx_ ?
            write_idx_ - read_idx_ :
            write_idx_ + capacity_ - read_idx_;
    }

    size_t available() const {
        return capacity_ - 1 - size();
    }

    /**
     * Checks if the ring buffer is empty.
     */
    bool empty() const {
        return read_idx_ == write_idx_;
    }

    /**
     * Checks if the ring buffer is full.
     */
    bool full() const {
        return read_idx_ == (write_idx_ + 1) % capacity_;
    }

    /**
     */
    bool write(size_t count = 1) {
        if (count > available()) return false;
        write_idx_ = (write_idx_ + count) % capacity_;
        return true;
    }

    /**
     */
    bool read(size_t count = 1) {
        if (count > size()) return false;
        read_idx_ = (read_idx_ + count) % capacity_;
        return true;
    }

    size_t write_head() const { return write_idx_; }
    size_t read_head() const { return read_idx_; }

   private:
    const size_t capacity_;
    std::atomic<size_t> write_idx_;
    std::atomic<size_t> read_idx_;
};