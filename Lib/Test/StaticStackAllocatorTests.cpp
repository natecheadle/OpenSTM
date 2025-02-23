#include <gtest/gtest.h>

#include <Allocator/StaticStackAllocator.hpp>

namespace openstm::lib::test {

class StaticStackAllocatorFixture : public testing::Test {
 public:
  struct TestStruct {
    double Value;
    int Value2;
  };
  static constexpr size_t BUFFER_SIZE = 256;
  using Buffer = StaticStackAllocator<BUFFER_SIZE>;
  StaticStackAllocatorFixture() { Buffer::Reset(); }
};

TEST_F(StaticStackAllocatorFixture, ValidateInitialState) {
  ASSERT_EQ(Buffer::RemainingSpace(), BUFFER_SIZE);
}

TEST_F(StaticStackAllocatorFixture, ValidateMallocate) {
  static constexpr size_t objSize = 8;
  void* pNew = Buffer::Mallocate(objSize);
  ASSERT_NE(pNew, nullptr);
  ASSERT_EQ(Buffer::RemainingSpace(), BUFFER_SIZE - objSize);
}

TEST_F(StaticStackAllocatorFixture, ValidateMallocateMax) {
  static constexpr size_t objSize = BUFFER_SIZE / 2;
  void* pNew = Buffer::Mallocate(objSize);
  void* pNew2 = Buffer::Mallocate(objSize);
  ASSERT_NE(pNew, nullptr);
  ASSERT_NE(pNew2, nullptr);
  ASSERT_EQ(Buffer::RemainingSpace(), 0);
}

TEST_F(StaticStackAllocatorFixture, ValidateMallocateFailsTooBig) {
  static constexpr size_t objSize = BUFFER_SIZE + 1;
  void* pNew = Buffer::Mallocate(objSize);
  ASSERT_EQ(pNew, nullptr);
  ASSERT_EQ(Buffer::RemainingSpace(), BUFFER_SIZE);
}

TEST_F(StaticStackAllocatorFixture, ValidateMallocateFailsTooBig2) {
  static constexpr size_t objSize = BUFFER_SIZE;
  void* pNew = Buffer::Mallocate(objSize);
  void* pNew2 = Buffer::Mallocate(objSize);
  ASSERT_NE(pNew, nullptr);
  ASSERT_EQ(pNew2, nullptr);
  ASSERT_EQ(Buffer::RemainingSpace(), 0);
}

TEST_F(StaticStackAllocatorFixture, ValidateMallocateMany) {
  static constexpr size_t objSize = 8;
  for (size_t i = 0; i < 10; ++i) {
    void* pNew = Buffer::Mallocate(objSize);
    ASSERT_NE(pNew, nullptr);
    ASSERT_EQ(Buffer::RemainingSpace(), BUFFER_SIZE - objSize * (i + 1));
  }
}

TEST_F(StaticStackAllocatorFixture, ValidateAllocateMany) {
  static constexpr size_t objSize = sizeof(TestStruct);
  for (size_t i = 0; i < 10; ++i) {
    TestStruct* pNew = Buffer::allocate<TestStruct>(1);
    ASSERT_NE(pNew, nullptr);
    ASSERT_EQ(Buffer::RemainingSpace(), BUFFER_SIZE - objSize * (i + 1));
  }
}

TEST_F(StaticStackAllocatorFixture, ValidateWithStdLibrary) {
  std::vector<TestStruct, StaticStackAllocatorT<TestStruct, BUFFER_SIZE>>
      vector;

  vector.resize(10);
}

}  // namespace openstm::lib::test
