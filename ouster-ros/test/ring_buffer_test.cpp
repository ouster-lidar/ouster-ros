#include <thread>
#include <cstring>
#include <random>
#include <gtest/gtest.h>
#include "../src/thread_safe_ring_buffer.h"

using namespace std::chrono_literals;

class ThreadSafeRingBufferTest : public ::testing::Test {
  protected:
    static constexpr int ITEM_SIZE = 4;   // predefined size for all items used in
    static constexpr int ITEM_COUNT = 3;  // number of item the buffer could hold

    void SetUp() override {
        buffer = std::make_unique<ThreadSafeRingBuffer>(ITEM_SIZE, ITEM_COUNT);
    }

    void TearDown() override {
        buffer.reset(nullptr);
    }

    std::string rand_str(int size) {
      const std::string characters =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<int> dist(0, characters.size() - 1);

      std::string result;
      for (int i = 0; i < size; ++i) {
          result += characters[dist(gen)];
      }
      return result;
    }

    std::vector<std::string> rand_vector_str(int vec_size, int str_size) {
      std::vector<std::string> output(vec_size);
      for (auto i = 0; i < vec_size; ++i)
        output[i] = rand_str(str_size);
      return output;
    }

    std::vector<std::string> known_vector_str(int vec_size, const std::string& known) {
      std::vector<std::string> output(vec_size);
      for (auto i = 0; i < vec_size; ++i)
        output[i] = known;
      return output;
    }

    void reset_writing() { buffer->reset_write_idx(); }

    void reset_reading() { buffer->reset_read_idx(); }

    std::unique_ptr<ThreadSafeRingBuffer> buffer;
};

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferSimple) {

    assert (ITEM_COUNT > 1 && "or this test can't run");

    static constexpr int TOTAL_ITEMS = 10; // total items to process
    const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
    std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    for (int i = 0; i < ITEM_COUNT; ++i) {
      buffer->write([i, &source](uint8_t* buffer) {
          std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
    }

    EXPECT_FALSE(buffer->empty());
    EXPECT_TRUE(buffer->full());

    // remove one item
    buffer->read([&target](uint8_t* buffer){
        std::memcpy(&target[0][0], buffer, ITEM_SIZE);
    });

    EXPECT_FALSE(buffer->empty());
    EXPECT_FALSE(buffer->full());
    EXPECT_EQ(buffer->size(), static_cast<size_t>(ITEM_COUNT - 1));

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
    for (int i = 1; i < ITEM_COUNT; ++i) {
      buffer->read([i, &target](uint8_t* buffer){
          std::memcpy(&target[i][0], buffer, ITEM_SIZE);
      });
    }

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    for (int i = 0; i < ITEM_COUNT; ++i) {
        std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
        EXPECT_EQ(target[i], source[i]); 
    }
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferBlocking) {

    static constexpr int TOTAL_ITEMS = 10; // total items to process
    const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
    std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    std::thread producer([this, &source]() {
        for (int i = 0; i < TOTAL_ITEMS; ++i) {
            buffer->write([i, &source](uint8_t* buffer){
                std::memcpy(buffer, &source[i][0], ITEM_SIZE);
            });
        }

        // Due to the lock-free implementation, that last item would not be read, since
        // the reader can not know if it's still being written to. So we have to reset
        // the write index before reading out the buffer.
        reset_writing();
    });

    std::thread consumer([this, &target]() {
        int i = 0;
        while (i < TOTAL_ITEMS) {
            buffer->read([&i, &target](uint8_t* buffer){
                std::memcpy(&target[i++][0], buffer, ITEM_SIZE);
            });
        }
    });

    producer.join();
    consumer.join();

    for (int i = 0; i < TOTAL_ITEMS; ++i) {
        std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
        EXPECT_EQ(target[i], source[i]);
    }

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferWithOverwrite) {

    static constexpr int TOTAL_ITEMS = 10; // total items to process
    const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
    std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    std::thread producer([this, &source]() {
        for (int i = 0; i < TOTAL_ITEMS; ++i) {
            buffer->write_overwrite([i, &source](uint8_t* buffer){
                std::memcpy(buffer, &source[i][0], ITEM_SIZE);
            });
        }

        // Due to the lock-free implementation, that last item would not be read, since
        // the reader can not know if it's still being written to. So we have to reset
        // the write index before reading out the buffer.
        reset_writing();
    });

    // wait for 1 second before starting the consumer thread
    // allowing sufficient time for the producer thread to be
    // completely done
    std::this_thread::sleep_for(1s);
    std::thread consumer([this, &target]() {
        for (int i = 0; i < TOTAL_ITEMS; ++i) {
            buffer->read_timeout([i, &target](uint8_t* buffer){
                  std::memcpy(&target[i][0], buffer, ITEM_SIZE);
            }, 1s);
        }
    });

    producer.join();
    consumer.join();

    // Since our buffer can host only up to ITEM_COUNT simultaneously only the
    // last ITEM_COUNT items would have remained in the buffer by the time
    // the consumer started processing.
    // If TOTAL_ITEMS is not divisible by ITEM_COUNT, the beginning of the buffer,
    // will contain a section of ITEM_COUNT items with the latest overwritten data.
    for (int i = 0; i < TOTAL_ITEMS % ITEM_COUNT; ++i) {
      std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
      EXPECT_EQ(target[i], source[TOTAL_ITEMS - (TOTAL_ITEMS % ITEM_COUNT) + i]);
    }
    // If TOTAL_ITEMS is divisible by ITEM_COUNT, the whole buffer will contain
    // exactly the last ITEM_COUNT items. Otherwise, the end of the buffer will
    // contain a section of ITEM_COUNT items with older data.
    for (int i = TOTAL_ITEMS % ITEM_COUNT; i < ITEM_COUNT; ++i) {
      std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
      EXPECT_EQ(target[i], source[TOTAL_ITEMS - (TOTAL_ITEMS % ITEM_COUNT) - ITEM_COUNT + i]);
    }
    // The remaining part of the target will not have any new data, since the buffer,
    // will now be completely read out.
    for (int i = ITEM_COUNT; i < TOTAL_ITEMS; ++i) {
      std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
      EXPECT_EQ(target[i], "0000");
    }

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferNonblocking) {

  static constexpr int TOTAL_ITEMS = 10; // total items to process
  const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
  std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());

  std::thread producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write_nonblock([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  // wait for 1 second before starting the consumer thread
  // allowing sufficient time for the producer thread to be
  // completely done
  std::this_thread::sleep_for(1s);
  std::thread consumer([this, &target]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->read_nonblock([i, &target](uint8_t* buffer){
        std::memcpy(&target[i][0], buffer, ITEM_SIZE);
      });
    }
  });

  producer.join();
  consumer.join();

  // Since our buffer can host only up to ITEM_COUNT simultaneously only the
  // first ITEM_COUNT items will be written into the buffer, with the rest being
  // ignored.
  for (int i = 0; i < ITEM_COUNT; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[i]);
  }
  // The remaining part of the target will not have any new data, since the buffer,
  // will now be completely read out.
  for (int i = ITEM_COUNT; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], "0000");
  }

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferBlockingThrottling) {

  static constexpr int TOTAL_ITEMS = 10; // total items to process
  const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
  std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");
  static constexpr std::chrono::milliseconds period(10);
  static constexpr int period_slowing_factor = 4;

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());

  // First, the consumer will read faster than the producer can write.
  std::thread slower_producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
      std::this_thread::sleep_for(period * period_slowing_factor);
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  std::thread faster_consumer([this, &target]() {
    int i = 0;
    while (i < TOTAL_ITEMS) {
      buffer->read([&i, &target](uint8_t* buffer){
        std::memcpy(&target[i++][0], buffer, ITEM_SIZE);
      });
      std::this_thread::sleep_for(period);
    }
  });

  slower_producer.join();
  faster_consumer.join();

  // Blocking read and write should always be synchronized even if one thread is faster.
  std::cout << "Slower producer, faster consumer:" << std::endl;
  for (int i = 0; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[i]);
  }

  ASSERT_TRUE(buffer->empty());
  ASSERT_FALSE(buffer->full());
  target = known_vector_str(TOTAL_ITEMS, "0000");
  reset_writing();
  reset_reading();

  // Then, the producer will write to the buffer faster than the consumer can read.
  std::thread faster_producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
      std::this_thread::sleep_for(period);
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  std::thread slower_consumer([this, &target]() {
    int i = 0;
    while (i < TOTAL_ITEMS) {
      buffer->read([&i, &target](uint8_t* buffer){
        std::memcpy(&target[i++][0], buffer, ITEM_SIZE);
      });
      std::this_thread::sleep_for(period * period_slowing_factor);
    }
  });

  faster_producer.join();
  slower_consumer.join();

  // Blocking read and write should always be synchronized even if one thread is faster.
  std::cout << "Faster producer, slower consumer:" << std::endl;
  for (int i = 0; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[i]);
  }

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferWithOverwriteThrottling) {

  static constexpr int TOTAL_ITEMS = 10; // total items to process
  const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
  std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");
  static constexpr std::chrono::milliseconds period(10);
  static constexpr int period_slowing_factor = 4;

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());

  // First, the consumer will read faster than the producer can write.
  std::thread slower_producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write_overwrite([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
      std::this_thread::sleep_for(period * period_slowing_factor);
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  std::thread faster_consumer([this, &target]() {
    int i = 0;
    while (i < TOTAL_ITEMS) {
      buffer->read_timeout([&i, &target](uint8_t* buffer){
        std::memcpy(&target[i++][0], buffer, ITEM_SIZE);
      }, 1s);
      std::this_thread::sleep_for(period);
    }
  });

  slower_producer.join();
  faster_consumer.join();

  // If the consumer is faster, it should always keep up with the latest data
  // and outputs should be synchronized.
  std::cout << "Slower producer, faster consumer:" << std::endl;
  for (int i = 0; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[i]);
  }

  ASSERT_TRUE(buffer->empty());
  ASSERT_FALSE(buffer->full());
  target = known_vector_str(TOTAL_ITEMS, "0000");
  reset_writing();
  reset_reading();

  // Then, the producer will write to the buffer faster than the consumer can read.
  std::thread faster_producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write_overwrite([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
      std::this_thread::sleep_for(period);
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  std::thread slower_consumer([this, &target]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->read_timeout([i, &target](uint8_t* buffer){
        std::memcpy(&target[i][0], buffer, ITEM_SIZE);
      }, 1s);
      std::this_thread::sleep_for(period * period_slowing_factor);
    }
  });

  faster_producer.join();
  slower_consumer.join();

  // This part should be automated for reasonable combinations of ITEM_COUNT and
  // TOTAL_ITEMS. Assuming ITEM_COUNT == 3, TOTAL_ITEMS == 10, and period_slowing_factor == 4
  // we should expect the following behavior:
  // The first read attempt will start before the producer takes ownership of the next
  // index, so the first item will not be filled.
  // By the second read attempt, the producer will have performed 1*4 writes,
  // ending back at index 0. So the second item is not filled.
  // By the third read attempt, the producer wil have performed 2*4 writes,
  // now ending at index 1. The consumer will fill one item.
  // By the fourth attempt, the producer will have finished writing with the last
  // 2 writes. The consumer will fill the last items from the buffer.
  // The remaining items will not be filled, since there were no more writes
  // being made.
  assert ((TOTAL_ITEMS - ITEM_COUNT) > 2 && "or this test section can't run");

  static constexpr int complete_writes = TOTAL_ITEMS / period_slowing_factor;
  static constexpr int read_attempts = (((10 * TOTAL_ITEMS) / period_slowing_factor)
                                        > 10 * complete_writes) ? complete_writes + 1 : complete_writes;
  int reading_buffer_idx = 0, written_buffer_idx = 0;
  int expected_source_idx = 0, written_source_idx = 0;

  std::cout << "Faster producer, slower consumer:" << std::endl;
  std::cout << "source " << source[0] << ", target " << target[0] << std::endl;
  EXPECT_EQ(target[0], "0000");

  // Checking all read attempts happening after a full batch of writes, and
  // the final writes plus one extra read.
  for (int i = 1; i <= read_attempts + 1; ++i) {
    written_source_idx = std::min(i * period_slowing_factor, TOTAL_ITEMS) - 1;
    written_buffer_idx = written_source_idx % ITEM_COUNT;
    expected_source_idx = written_source_idx - written_buffer_idx + reading_buffer_idx;
    if (written_buffer_idx < reading_buffer_idx)
      expected_source_idx -= ITEM_COUNT;

    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    if ((written_buffer_idx == reading_buffer_idx) && (written_source_idx != (TOTAL_ITEMS - 1))) {
      EXPECT_EQ(target[i], "0000");
    } else {
      reading_buffer_idx = (reading_buffer_idx + 1) % ITEM_COUNT;
      EXPECT_EQ(target[i], source[expected_source_idx]);
    }
  }

  // We're not checking all final reads, since that would depend more on the
  // relationship between TOTAL_ITEMS and ITEM_COUNT, and not the running behavior.
  // So, just printing out any skipped items for completion.
  int items_left_in_buffer = ITEM_COUNT;
  if (written_buffer_idx < reading_buffer_idx)
    items_left_in_buffer += written_buffer_idx + 1 - reading_buffer_idx;
  for (int i = read_attempts + 2; i <= read_attempts + items_left_in_buffer; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
  }

  // Since the producer finished with overwrites faster that the consumer
  // could read them out, some final items should stay empty.
  for (int i = read_attempts + items_left_in_buffer + 1; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], "0000");
  }

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferNonblockingThrottling) {

  static constexpr int TOTAL_ITEMS = 10; // total items to process
  const std::vector<std::string> source = rand_vector_str(TOTAL_ITEMS, ITEM_SIZE);
  std::vector<std::string> target = known_vector_str(TOTAL_ITEMS, "0000");
  static constexpr std::chrono::milliseconds period(10);
  static constexpr int period_slowing_factor = 4;

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());

  // First, the consumer will read faster than the producer can write.
  std::thread slower_producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write_nonblock([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
      std::this_thread::sleep_for(period * period_slowing_factor);
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  std::thread faster_consumer([this, &target]() {
    int i = 0;
    while (i < TOTAL_ITEMS) {
      buffer->read_nonblock([&i, &target](uint8_t* buffer){
        std::memcpy(&target[i++][0], buffer, ITEM_SIZE);
      });
      std::this_thread::sleep_for(period);
    }
  });

  slower_producer.join();
  faster_consumer.join();

  // If the consumer is faster, it should always keep up with the latest data
  // and outputs should be synchronized.
  std::cout << "Slower producer, faster consumer:" << std::endl;
  for (int i = 0; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[i]);
  }

  ASSERT_TRUE(buffer->empty());
  ASSERT_FALSE(buffer->full());
  target = known_vector_str(TOTAL_ITEMS, "0000");
  reset_writing();
  reset_reading();

  // Then, the producer will write to the buffer faster than the consumer can read.
  std::thread faster_producer([this, &source]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->write_nonblock([i, &source](uint8_t* buffer){
        std::memcpy(buffer, &source[i][0], ITEM_SIZE);
      });
      std::this_thread::sleep_for(period);
    }

    // Due to the lock-free implementation, that last item would not be read, since
    // the reader can not know if it's still being written to. So we have to reset
    // the write index before reading out the buffer.
    reset_writing();
  });

  std::thread slower_consumer([this, &target]() {
    for (int i = 0; i < TOTAL_ITEMS; ++i) {
      buffer->read_nonblock([i, &target](uint8_t* buffer){
        std::memcpy(&target[i][0], buffer, ITEM_SIZE);
      });
      std::this_thread::sleep_for(period * period_slowing_factor);
    }
  });

  faster_producer.join();
  slower_consumer.join();

  // This part should be automated for reasonable combinations of ITEM_COUNT and
  // TOTAL_ITEMS. Assuming ITEM_COUNT == 3, TOTAL_ITEMS == 10, and period_slowing_factor == 4
  // we should expect the following behavior:
  // The first read attempt will start before the producer takes ownership of the next
  // index, so the first item will not be filled.
  // By the second read attempt, the producer will have performed 1*4 writes,
  // filling the buffer and dropping the last write. The consumer will fill in
  // the first item.
  // By the third read attempt, the producer will have performed 2*4 writes,
  // filling only one item and dropping the rest. The consumer will read the second
  // item.
  // By the fourth attempt, the producer will have finished with the last 2 writes,
  // filling only one item and dropping the final one.
  // The consumer will read the last items from the buffer.
  // The remaining items will not be filled, since there were no more writes
  // being made.
  assert ((TOTAL_ITEMS - ITEM_COUNT) > 2 && "or this test section can't run");
  assert(period_slowing_factor > 2 && "or this test section can't run");

  static constexpr int full_writes = ITEM_COUNT / period_slowing_factor;
  static constexpr int consecutive_reads = std::min(ITEM_COUNT + full_writes, TOTAL_ITEMS - 1);
  static constexpr int saturated_reads =
      (TOTAL_ITEMS + (TOTAL_ITEMS % period_slowing_factor) - consecutive_reads) / period_slowing_factor;
  int expected_source_idx = 0;

  std::cout << "Faster producer, slower consumer:" << std::endl;
  std::cout << "source " << source[0] << ", target " << target[0] << std::endl;
  EXPECT_EQ(target[0], "0000");

  // The buffer should always be filled with consecutive items upto the point,
  // when the producer catches up with the consumer on its second pass.
  for (int i = 1; i <= consecutive_reads; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[i - 1]);
  }

  // Since the producer cannot overwrite, it would always fill just the first item,
  // in its iteration.
  int writing_iteration = 1;
  for (int i = consecutive_reads + 1; i <= consecutive_reads + saturated_reads; ++i) {
    expected_source_idx = (full_writes + writing_iteration++) * period_slowing_factor;
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], source[expected_source_idx]);
  }

  // Since the producer finished with writes faster that the consumer
  // could read them out, some final items should stay empty.
  for (int i = consecutive_reads + saturated_reads + 1; i < TOTAL_ITEMS; ++i) {
    std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
    EXPECT_EQ(target[i], "0000");
  }

  EXPECT_TRUE(buffer->empty());
  EXPECT_FALSE(buffer->full());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}