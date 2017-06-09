#ifndef ROS2_TIME_DURATION_HPP
#define ROS2_TIME_DURATION_HPP


// standard headers
#include <chrono>
#include <iostream>


namespace ros2_time
{


class Duration
{
public:
    Duration();
    Duration(const double& secs);
    Duration(const uint32_t& _sec, const uint32_t& _nsec);

    double toSec() const;
    int64_t toNSec() const;
  
    Duration& fromSec(const unsigned long sec);
    Duration& fromNSec(const unsigned long nsec);

    bool sleep() const;

public:
    void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

    int32_t sec;
    int32_t nsec;

    bool operator> (const Duration& other);
    bool operator>= (const Duration& other);

    bool operator< (const Duration& other);
    bool operator<= (const Duration& other);

    Duration operator- (const Duration& other);
    Duration operator+ (const Duration& other);

    friend std::ostream& operator<< (std::ostream& out, const Duration& dur);
};


}  // end of the ros2_time namespace


#endif  // ROS2_TIME_DURATION_HPP
