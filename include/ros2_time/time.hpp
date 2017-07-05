#ifndef ROS2_TIME_TIME_HPP
#define ROS2_TIME_TIME_HPP


// standard headers
#include <chrono>

// project headers
#include <ros2_time/duration.hpp>

// builtin interface headers
#include <builtin_interfaces/msg/time.hpp>


namespace ros2_time
{

  
// various types
typedef std::chrono::time_point<std::chrono::steady_clock> TimePoint;

  
class Time
{
public:
    Time();
    Time(const TimePoint& tp);
    Time(const builtin_interfaces::msg::Time& time);

    bool isValid() const;
  
    int32_t toSec() const;
    uint64_t toNSec() const;

    Time& fromSec(const long sec);
    Time& fromNSec(const long time);

    TimePoint getTimePoint() const;

    builtin_interfaces::msg::Time toStamp() const;
    ros2_time::Time& fromStamp(const builtin_interfaces::msg::Time& time);
  
    static Time now();

    bool operator==(const Time& rhs);
    bool operator!=(const Time& rhs);
    bool operator>(const Time& rhs);
    bool operator<(const Time& rhs);
    Time operator+(const Duration& dur);
    Time operator-(const Duration& dur);
    Time& operator+=(const Duration& dur);
    Duration operator-(const Time& rhs);
    void operator=(const builtin_interfaces::msg::Time& time);
  
protected:
    void normalizeSecNSec(unsigned long& sec, unsigned long& nsec) const;

    TimePoint m_time;
};


// For now: provide WallTime and WallDuration as clones of normal time
// eventually support custom implementation
typedef Time WallTime;
typedef Duration WallDuration;
  

}  // end of the ros2_time namespace


#endif  // ROS2_TIME_TIME_HPP
