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
        buffer.reset();
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

    std::unique_ptr<ThreadSafeRingBuffer> buffer;
};

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferSimple) {

    assert (ITEM_COUNT > 1 && "or this test can't run");

    const int TOTAL_ITEMS = 10; // total items to process
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

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBuffer) {

    const int TOTAL_ITEMS = 10; // total items to process
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
    });

    std::thread consumer([this, &target]() {
        for (int i = 0; i < TOTAL_ITEMS; ++i) {
            buffer->read([i, &target](uint8_t* buffer){
                std::memcpy(&target[i][0], buffer, ITEM_SIZE);
            });
        }
    });

    producer.join();
    consumer.join();

    for (int i = 0; i < TOTAL_ITEMS; ++i) {
        std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
        EXPECT_EQ(target[i], source[i]); 
    }
}

TEST_F(ThreadSafeRingBufferTest, ReadWriteToBufferWithOverwrite) {

    const int TOTAL_ITEMS = 10; // total items to process
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
    });

    // wait for 1 second before starting the consumer thread
    // allowing sufficient time for the producer thread to be
    // completely done
    std::this_thread::sleep_for(1s);
    std::thread consumer([this, &target]() {
        for (int i = 0; i < TOTAL_ITEMS; ++i) {
            buffer->read_timeout([i, &target](uint8_t* buffer){
                if (buffer != nullptr)
                    std::memcpy(&target[i][0], buffer, ITEM_SIZE);
            }, 1s);
        }
    });

    producer.join();
    consumer.join();

    // Since our buffer can host only up to ITEM_COUNT simultanously only the
    // last ITEM_COUNT items would have remained in the buffer by the time
    // the consumer started processing.
    for (int i = 0; i < ITEM_COUNT; ++i) {
        std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
        EXPECT_EQ(target[i], source[TOTAL_ITEMS-ITEM_COUNT+i]); 
    }

    for (int i = ITEM_COUNT; i < TOTAL_ITEMS; ++i) {
        std::cout << "source " << source[i] << ", target " << target[i] << std::endl;
        EXPECT_EQ(target[i], "0000"); 
    }
}
