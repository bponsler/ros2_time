// project headers
#include <ros2_time/time.hpp>

#include <rclcpp/rclcpp.hpp>

// gtest includes
#include <gtest/gtest.h>


TEST(DurationTestSuite, testCtor)
{
  ros2_time::Duration d1(0.0);
  EXPECT_EQ(d1.toSec(), 0);
  EXPECT_EQ(d1.toNSec(), 0);
  EXPECT_EQ(d1.sec, 0);
  EXPECT_EQ(d1.nsec, 0);

  ros2_time::Duration d2(5.0);
  EXPECT_NEAR(d2.toSec(), 5.0, 0.01);
  EXPECT_NEAR(d2.toNSec(), 5*1e9, 0.01*1e9);
  EXPECT_EQ(d2.sec, 5);
  EXPECT_EQ(d2.nsec, 0);

  ros2_time::Duration d3(-3.0);
  EXPECT_NEAR(d3.toSec(), -3.0, 0.01);
  EXPECT_NEAR(d3.toNSec(), -3*1e9, 0.01*1e9);
  EXPECT_EQ(d3.sec, -3);
  EXPECT_EQ(d3.nsec, 0);

  ros2_time::Duration d4(9.54);
  EXPECT_NEAR(d4.toSec(), 9.54, 0.01);
  EXPECT_NEAR(d4.toNSec(), 9.54*1e9, 0.01*1e9);
  EXPECT_EQ(d4.sec, 9);
  EXPECT_NEAR(d4.nsec, 0.54*1e9, 0.01*1e9);
}

TEST(DurationTestSuite, testNow)
{
  ros2_time::Time t1 = ros2_time::Time::now();
  ros2_time::Duration d1(5.0);
  EXPECT_EQ(d1.toSec(), 5);
  EXPECT_EQ(d1.toNSec(), 5*1e9);
  EXPECT_EQ(d1.sec, 5);
  EXPECT_EQ(d1.nsec, 0);

  ros2_time::Time t2 = t1 + d1;
  EXPECT_TRUE(t2.isValid());
  EXPECT_TRUE(t2.toSec() > 0);
  EXPECT_TRUE(t2.toNSec() > 0);
  EXPECT_TRUE(t2.getTimePoint().time_since_epoch().count() > 0);
  EXPECT_TRUE(t2.toSec() > t1.toSec());
  EXPECT_TRUE(
      t2.getTimePoint().time_since_epoch().count() >
      t1.getTimePoint().time_since_epoch().count());
  EXPECT_TRUE(t2.toNSec() > t1.toNSec());
  EXPECT_NEAR(t2.toSec() - t1.toSec(), 5, 0.01);
  EXPECT_NEAR(t2.toNSec() - t1.toNSec(), 5*1e9, 0.01*1e9);

  // Check that computing the duration returns the same results
  ros2_time::Duration d2 = t2 - t1;
  EXPECT_EQ(d2.toSec(), 5);
  EXPECT_EQ(d2.toNSec(), 5*1e9);
  EXPECT_EQ(d2.sec, 5);
  EXPECT_EQ(d2.nsec, 0);
}

// Run all tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("duration_tests");
    return RUN_ALL_TESTS();
}
