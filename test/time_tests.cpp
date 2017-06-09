// project headers
#include <ros2_time/time.hpp>

#include <rclcpp/rclcpp.hpp>

// gtest includes
#include <gtest/gtest.h>


TEST(TimeTestSuite, testCtor)
{
  ros2_time::Time t1;
  EXPECT_FALSE(t1.isValid());
  EXPECT_EQ(t1.toSec(), 0);
  EXPECT_EQ(t1.toNSec(), 0);
  EXPECT_EQ(t1.getTimePoint().time_since_epoch().count(), 0);
}

TEST(TimeTestSuite, testNow)
{
  ros2_time::Time t1 = ros2_time::Time::now();
  EXPECT_TRUE(t1.isValid());
  EXPECT_TRUE(t1.toSec() > 0);
  EXPECT_TRUE(t1.toNSec() > 0);
  EXPECT_TRUE(t1.getTimePoint().time_since_epoch().count() > 0);

  // Sleep for some time
  sleep(2);

  ros2_time::Time t2 = ros2_time::Time::now();
  EXPECT_TRUE(t2.isValid());
  EXPECT_TRUE(t2.toSec() > 0);
  EXPECT_TRUE(t2.toNSec() > 0);
  EXPECT_TRUE(t2.getTimePoint().time_since_epoch().count() > 0);
  EXPECT_TRUE(t2.toSec() > t1.toSec());
  EXPECT_TRUE(
      t2.getTimePoint().time_since_epoch().count() >
      t1.getTimePoint().time_since_epoch().count());
  EXPECT_TRUE(t2.toNSec() > t1.toNSec());

  // Check that subtracting two times yields the correct duration
  ros2_time::Duration d = t2 - t1;
  EXPECT_NEAR(d.sec, 2, 0.01);
  EXPECT_NEAR(d.nsec, 0, 0.01*1e9);
  EXPECT_NEAR(d.toSec(), 2, 0.01);

  // Check that subtracting the other way works
  d = t1 - t2;
  EXPECT_NEAR(d.sec, -2, 0.01);
  EXPECT_NEAR(d.nsec, 0, 0.01*1e9);
  EXPECT_NEAR(d.toSec(), -2, 0.01);

  ros2_time::Time t3 = t2;
  EXPECT_EQ(t3.isValid(), t2.isValid());
  EXPECT_EQ(t3.toSec(), t2.toSec());
  EXPECT_EQ(t3.toNSec(), t2.toNSec());
  EXPECT_EQ(t3.getTimePoint().time_since_epoch().count(), t2.getTimePoint().time_since_epoch().count());

  // Test += operator
  t3 += ros2_time::Duration(10.0);
  EXPECT_TRUE(t3.toSec() > t2.toSec());
  EXPECT_TRUE(t3.toNSec() > t2.toNSec());
  EXPECT_TRUE(t3.getTimePoint().time_since_epoch().count() > t2.getTimePoint().time_since_epoch().count());
}

// Run all tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto node = rclcpp::node::Node::make_shared("time_tests");
    return RUN_ALL_TESTS();
}
