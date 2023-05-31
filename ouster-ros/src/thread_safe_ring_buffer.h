/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file thread_safe_ring_buffer.h
 * @brief File contains thread safe implementation of a ring buffer
 */

#pragma once

#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>

/**
 * @class ThreadSafeRingBuffer thread safe ring buffer.
 */
class ThreadSafeRingBuffer {
public:
    ThreadSafeRingBuffer(size_t item_size_, size_t items_count_) :
        buffer(item_size_ * items_count_), item_size(item_size_),
        max_items_count(items_count_), active_items_count(0),
        write_idx(0), read_idx(0) {
    }

    /**
     * Gets the maximum number of items that this ring buffer can hold.
     */
    size_t capacity() const { return max_items_count; }

    /**
     * Gets the number of item that currently occupy the ring buffer.
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return active_items_count;
    }

    /**
     * Checks if the ring buffer is empty.
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex);
        return active_items_count == 0;
    }

    /**
     * Checks if the ring buffer is full.
     */
    bool full() const {
        std::lock_guard<std::mutex> lock(mutex);
        return active_items_count == max_items_count;
    }

    /**
     * Writes to the buffer safely, the method will keep blocking until the there
     * is a space available within the buffer.
     */
    template <class BufferWriteT>
    void write(BufferWriteT buffer_write) {
        std::unique_lock<std::mutex> lock(mutex);
        fullCondition.wait(lock, [this] { return active_items_count < capacity(); });
        buffer_write(&buffer[write_idx * item_size]);
        write_idx = (write_idx + 1) % capacity();
        ++active_items_count;
        emptyCondition.notify_one();
    }


    /**
     * Writes to the buffer safely, if there is not space left then this method
     * will overite the last item.
     */
    template <class BufferWriteT>
    void write_overwrite(BufferWriteT buffer_write) {
        std::unique_lock<std::mutex> lock(mutex);
        buffer_write(&buffer[write_idx * item_size]);
        write_idx = (write_idx + 1) % capacity();
        if (active_items_count < capacity()) {
            ++active_items_count;
        } else {
            read_idx = (read_idx + 1) % capacity();
        }
        emptyCondition.notify_one();
    }

    /**
     * Gives access to read the buffer through a callback, the method will block
     * until there is something to read is available.
     */
    template <typename BufferReadT>
    void read(BufferReadT&& buffer_read) {
        std::unique_lock<std::mutex> lock(mutex);
        emptyCondition.wait(lock, [this] { return active_items_count > 0; });
        buffer_read(&buffer[read_idx * item_size]);
        read_idx = (read_idx + 1) % capacity();
        --active_items_count;
        fullCondition.notify_one();
    }

    /**
     * Gives access to read the buffer through a callback, if buffer is
     * inaccessible the method will timeout and buffer_read gets a nullptr.
     */
    template <typename BufferReadT>
    void read_timeout(BufferReadT buffer_read, std::chrono::seconds timeout) {
        std::unique_lock<std::mutex> lock(mutex);
        if (emptyCondition.wait_for(lock, timeout, [this] { return active_items_count > 0; })) {
            buffer_read(&buffer[read_idx * item_size]);
            read_idx = (read_idx + 1) % capacity();
            --active_items_count;
            fullCondition.notify_one();
        } else {
            buffer_read((uint8_t*)nullptr);
        }
    }

private:
    std::vector<uint8_t> buffer;
    size_t item_size;
    size_t max_items_count;
    size_t active_items_count;
    size_t write_idx;
    size_t read_idx;
    mutable std::mutex mutex;
    std::condition_variable fullCondition;
    std::condition_variable emptyCondition;
};