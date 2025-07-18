#include "../src/utils/utils.hpp"
#include <gtest/gtest.h>

#include <iostream>

TEST(UtilsTest, angularSum) {
  {
    EXPECT_NEAR(angularSum(0.0, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(angularSum(0.0, M_PI), M_PI, 1e-9);
    EXPECT_NEAR(angularSum(M_PI, 0.0), M_PI, 1e-9);
    EXPECT_NEAR(angularSum(M_PI_2, M_PI_2), M_PI, 1e-9);
    EXPECT_NEAR(angularSum(M_PI, 0.9999 * M_PI), M_PI + 0.9999 * M_PI, 1e-9);
    EXPECT_NEAR(angularSum(M_PI, M_PI), 0.0, 1e-9);
    EXPECT_NEAR(angularSum(2 * M_PI, M_PI), M_PI, 1e-9);
    EXPECT_NEAR(angularSum(10 * M_PI, 10 * M_PI), 0.0, 1e-9);
    EXPECT_NEAR(angularSum(0.0, -7 * M_PI_4), M_PI_4, 1e-9);
  }
}

TEST(UtilsTest, angularDiff) {
  {
    EXPECT_NEAR(angularDiff(0.0, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(angularDiff(M_PI, 0.0), M_PI, 1e-9);
    EXPECT_NEAR(angularDiff(M_PI, M_PI_2), M_PI_2, 1e-9);
    EXPECT_NEAR(angularDiff(2 * M_PI, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(angularDiff(2 * M_PI, -M_PI_4), M_PI_4, 1e-9);
    EXPECT_NEAR(angularDiff(11 * M_PI, 2 * M_PI), M_PI, 1e-9);
  }
}