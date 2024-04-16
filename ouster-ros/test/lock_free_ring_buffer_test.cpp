#include <thread>
#include <cstring>
#include <random>
#include <gtest/gtest.h>
#include "../src/lock_free_ring_buffer.h"

using namespace std::chrono_literals;

class LockFreeRingBufferTest : public ::testing::Test {
  protected:
    static constexpr size_t ITEM_COUNT = 3;  // number of item the buffer could hold

    void SetUp() override {
        buffer = std::make_unique<LockFreeRingBuffer>(ITEM_COUNT);
    }

    void TearDown() override {
        buffer.reset();
    }

    std::unique_ptr<LockFreeRingBuffer> buffer;
};


TEST_F(LockFreeRingBufferTest, ReadWriteToBufferFullEmpty) {

    assert (ITEM_COUNT > 1 && "or this test can't run");

    EXPECT_EQ(buffer->capacity(), ITEM_COUNT);
    EXPECT_EQ(buffer->available(), ITEM_COUNT - 1);
    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    for (size_t i = 0; i < ITEM_COUNT - 1; ++i) {
      buffer->write();
    }

    EXPECT_FALSE(buffer->empty());
    EXPECT_TRUE(buffer->full());

    // remove one item
    buffer->read();

    EXPECT_FALSE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    for (size_t i = 1; i < ITEM_COUNT - 1; ++i) {
      buffer->read();
    }

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());
}

TEST_F(LockFreeRingBufferTest, ReadWriteToBufferCheckReturn) {

    assert (ITEM_COUNT > 1 && "or this test can't run");

    EXPECT_TRUE(buffer->empty());

    for (size_t i = 0; i < ITEM_COUNT - 1; ++i) {
      EXPECT_TRUE(buffer->write());
    }

    EXPECT_TRUE(buffer->full());
    EXPECT_FALSE(buffer->write());

    // remove one item and re-write
    EXPECT_TRUE(buffer->read());
    EXPECT_TRUE(buffer->write());

    for (size_t i = 0; i < ITEM_COUNT - 1; ++i) {
      EXPECT_TRUE(buffer->read());
    }

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->read());
}


TEST_F(LockFreeRingBufferTest, ReadWriteToBufferSizeAvailable) {

    assert (ITEM_COUNT == 3 && "or this test can't run");

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);
    EXPECT_EQ(buffer->write_head(), 0U);
    EXPECT_EQ(buffer->read_head(), 0U);

    EXPECT_TRUE(buffer->write());
    EXPECT_EQ(buffer->size(), 1U);
    EXPECT_EQ(buffer->available(), 1U);
    EXPECT_EQ(buffer->write_head(), 1U);
    EXPECT_EQ(buffer->read_head(), 0U);

    EXPECT_TRUE(buffer->write());
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);
    EXPECT_EQ(buffer->write_head(), 2U);
    EXPECT_EQ(buffer->read_head(), 0U);

    EXPECT_TRUE(buffer->read());
    EXPECT_EQ(buffer->size(), 1U);
    EXPECT_EQ(buffer->available(), 1U);
    EXPECT_EQ(buffer->write_head(), 2U);
    EXPECT_EQ(buffer->read_head(), 1U);

    EXPECT_TRUE(buffer->write());
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);
    EXPECT_EQ(buffer->write_head(), 0U);
    EXPECT_EQ(buffer->read_head(), 1U);

    // Next write should fail, so size shouldn't change
    EXPECT_FALSE(buffer->write());
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);
    EXPECT_EQ(buffer->write_head(), 0U);
    EXPECT_EQ(buffer->read_head(), 1U);

    EXPECT_TRUE(buffer->read());
    EXPECT_EQ(buffer->size(), 1U);
    EXPECT_EQ(buffer->available(), 1U);
    EXPECT_EQ(buffer->write_head(), 0U);
    EXPECT_EQ(buffer->read_head(), 2U);

    EXPECT_TRUE(buffer->read());
    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);
    EXPECT_EQ(buffer->write_head(), 0U);
    EXPECT_EQ(buffer->read_head(), 0U);

    // Next read should fail, so size shouldn't change
    EXPECT_FALSE(buffer->read());
    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);
    EXPECT_EQ(buffer->write_head(), 0U);
    EXPECT_EQ(buffer->read_head(), 0U);
}

TEST_F(LockFreeRingBufferTest, ReadWriteToBufferAdvanceMultiple) {

    assert (ITEM_COUNT == 3 && "or this test can't run");

    EXPECT_TRUE(buffer->empty());
    EXPECT_FALSE(buffer->full());

    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);

    EXPECT_TRUE(buffer->write(2));
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);

    // This write should fail since we advance beyond capacity, so size shouldn't change
    EXPECT_FALSE(buffer->write(2));
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);

    EXPECT_TRUE(buffer->read());
    EXPECT_EQ(buffer->size(), 1U);
    EXPECT_EQ(buffer->available(), 1U);

    EXPECT_TRUE(buffer->read());
    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);

    EXPECT_TRUE(buffer->write(2));
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);

    // Any additional write will also fail, size shouldn't change
    EXPECT_FALSE(buffer->write());
    EXPECT_EQ(buffer->size(), 2U);
    EXPECT_EQ(buffer->available(), 0U);

    EXPECT_TRUE(buffer->read(2));
    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);

    // This read should fail since we advance beyond available, so size shouldn't change
    EXPECT_FALSE(buffer->read(2));
    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);

    // Any subsequent read will also fail, size shouldn't change
    EXPECT_FALSE(buffer->read());
    EXPECT_EQ(buffer->size(), 0U);
    EXPECT_EQ(buffer->available(), 2U);
}
