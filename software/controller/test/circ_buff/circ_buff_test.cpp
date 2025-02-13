/* Copyright 2020, RespiraWorks

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <cstdlib>
#include <optional>

#include "circular_buffer.h"
#include "gtest/gtest.h"

// Test the buffer counts and rollover
TEST(CircularBuffer, Counts) {
  constexpr uint BufferSize = 128;
  CircularBuffer<uint8_t, BufferSize> buff;

  ASSERT_EQ(buff.FreeCount(), BufferSize);
  ASSERT_EQ(buff.FullCount(), 0);

  // Add/remove bytes from the buffer and check counts along the way.
  // The full/free calculations change when the buffer wraps, so I
  // want to make sure I do this enough times to catch any errors in
  // that logic
  for (int i = 0; i < 100; i++) {
    int full = 0;
    int free = BufferSize;
    for (int j = 0; j < 20; j++) {
      // By putting j in the buffer, I can then check that it is stays a FIFO
      // even when it wraps around
      bool ok = buff.Put(static_cast<uint8_t>(j));
      ASSERT_EQ(ok, true);

      full++;
      free--;
      ASSERT_EQ(buff.FreeCount(), free);
      ASSERT_EQ(buff.FullCount(), full);
    }

    for (int j = 0; j < 20; j++) {
      std::optional<uint8_t> ch = buff.Get();
      ASSERT_EQ(*ch, static_cast<uint8_t>(j));

      full--;
      free++;
      ASSERT_EQ(buff.FreeCount(), free);
      ASSERT_EQ(buff.FullCount(), full);
    }
  }
}

// Make sure the data we add to the buffer is the same
// as the data we get out of it
TEST(CircularBuffer, DataIO) {
  constexpr uint BufferSize = 256;
  CircularBuffer<uint8_t, BufferSize> buff;

  uint8_t TestSet[BufferSize + 20];
  for (uint32_t i = 0; i < sizeof(TestSet); i++) TestSet[i] = static_cast<uint8_t>(rand());

  // Add data to the buffer until failure - multiple failures should not affect
  // our test results: the buffer's content stays unchanged during failure to
  // put more data
  for (int i = 0; i < BufferSize + 20; i++) {
    bool ok = buff.Put(TestSet[i]);
    if (i < BufferSize) {
      ASSERT_EQ(ok, true);
      ASSERT_EQ(buff.FullCount(), i + 1);
      ASSERT_EQ(buff.FreeCount(), BufferSize - 1 - i);
    } else {
      ASSERT_EQ(ok, false);
      ASSERT_EQ(buff.FullCount(), BufferSize);
      ASSERT_EQ(buff.FreeCount(), 0);
      break;
    }
  }

  // Retrieve all the data that was succesfully put in the buffer and check its
  // integrity
  for (int i = 0; i < BufferSize; i++) {
    std::optional<uint8_t> x = buff.Get();
    ASSERT_EQ(*x, TestSet[i]);
  }

  // The buffer should be empty now
  ASSERT_EQ(buff.FullCount(), 0);
  ASSERT_EQ(buff.FreeCount(), BufferSize);
  ASSERT_EQ(buff.Get(), std::nullopt);
  // failure to Get() has not changed anything
  ASSERT_EQ(buff.FullCount(), 0);
  ASSERT_EQ(buff.FreeCount(), BufferSize);
  ASSERT_EQ(buff.Get(), std::nullopt);
}

// Test that the buffer of minimal size (uint N = 0) is always empty
// This has no practical use but this way we make sure the template is safe.
TEST(CircularBuffer, Size0) {
  CircularBuffer<uint8_t, 0> buff;
  ASSERT_EQ(buff.FullCount(), 0);
  ASSERT_EQ(buff.FreeCount(), 0);
  ASSERT_EQ(buff.Put(0), false);
  ASSERT_EQ(buff.Get(), std::nullopt);
}

TEST(CircularBuffer, SmallBufferFullTest) {
  constexpr uint BufferSize = 5;
  CircularBuffer<uint8_t, BufferSize> buff;
  ASSERT_EQ(buff.FullCount(), 0);
  ASSERT_EQ(buff.FreeCount(), BufferSize);
  ASSERT_EQ(buff.Get(), std::nullopt);

  // Put a first element in the buffer.  This allows the next part to be just
  // one big for loop.
  uint8_t elem = 0;
  ASSERT_EQ(buff.Put(elem++), true);
  ASSERT_EQ(buff.FullCount(), 1);
  ASSERT_EQ(buff.FreeCount(), BufferSize - 1);

  // This loop tests that for all configurations of a non-empty buffer,
  // we can put and get stuff as we want, and that stuff is properly handled.
  for (int i = 0; i < BufferSize + 1; i++) {
    // Fill the buffer and check the counts while we are at it.  Note there is
    // already one elem in buffer so we only add BufferSize - 1 new elements.
    for (int j = 1; j < BufferSize; j++) {
      ASSERT_EQ(buff.Put(elem++), true);
      ASSERT_EQ(buff.FullCount(), j + 1);
      ASSERT_EQ(buff.FreeCount(), BufferSize - j - 1);
    }
    // Buffer must now be full, we can't put any more data in.
    ASSERT_EQ(buff.Put(0), false);
    // Pop all but one element and check the counts.
    // Because we leave 1 element, the next loop will iterate
    // with a different tail location: mod(tail - 1, BufferSize).
    for (int j = 1; j < BufferSize; j++) {
      std::optional<uint8_t> ch = buff.Get();
      ASSERT_EQ(*ch, static_cast<uint8_t>(elem - static_cast<uint8_t>(BufferSize + 1 - j)));
      ASSERT_EQ(buff.FullCount(), BufferSize - j);
      ASSERT_EQ(buff.FreeCount(), j);
    }
  }

  // Flush the buffer and go back to head=tail=0.
  buff.Flush();
  ASSERT_EQ(buff.FullCount(), 0);
  ASSERT_EQ(buff.FreeCount(), BufferSize);
  ASSERT_EQ(buff.Get(), std::nullopt);

  // Fill in the gap left by the original loop to include empty buffers.
  for (int i = 0; i < 5; i++) {
    ASSERT_EQ(buff.Put(elem), true);
    ASSERT_EQ(buff.FullCount(), 1);
    ASSERT_EQ(buff.FreeCount(), BufferSize - 1);
    std::optional<uint8_t> ch = buff.Get();
    ASSERT_EQ(*ch, elem++);
    ASSERT_EQ(buff.FullCount(), 0);
    ASSERT_EQ(buff.FreeCount(), BufferSize);
    // Trying to get an element from an empty buffer doesn't change anything,
    // whatever the head/tail
    ASSERT_EQ(buff.Get(), std::nullopt);
    ASSERT_EQ(buff.FullCount(), 0);
    ASSERT_EQ(buff.FreeCount(), BufferSize);
  }
}
