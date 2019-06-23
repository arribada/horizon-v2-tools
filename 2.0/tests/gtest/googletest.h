// Copyright 2015 Icoteq Ltd.

// If we are running under Gimpel Flexelint, we need to mock away some of the
// GoogleTest code because flint doesn't understand it. Otherwise just include
// the standard GoogleTest header.

#ifndef _lint

#include <gtest/gtest.h>

#else // building for lint

#define TEST_F(testFixture, testName) \
        class testFixture ## testName : public testFixture { \
                void TestBody() const;\
        };\
        void testFixture ## testName::TestBody() const

#define TEST(testFixture, testName) \
        class testFixture ## testName : public testing::Test { \
                void TestBody() const;\
        };\
        void testFixture ## testName::TestBody() const

#define ASSERT_EQ(a, b) assert(a == b)
#define EXPECT_EQ(a, b) assert(a == b)
#define ASSERT_GT(a, b) assert(a > b)

namespace testing {
class Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};
}
#endif
