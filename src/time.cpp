#include <ros2_time/time.hpp>


namespace ros2_time
{


Time::Time()
    :
    m_time()
{
}
  
bool
Time::isValid() const
{
    return (m_time.time_since_epoch().count() > 0);
}
  
int32_t
Time::toSec() const
{
    double nsec = m_time.time_since_epoch().count();
    return (uint32_t)(nsec / 1e9);
}

uint64_t
Time::toNSec() const
{
    return (uint64_t)m_time.time_since_epoch().count();
}

Time&
Time::fromSec(const long sec)
{
    this->m_time = TimePoint(std::chrono::seconds(sec));
    return *this;
}

Time&
Time::fromNSec(const long time)
{
    unsigned long sec = time / 1000000000;
    unsigned long nsec = time % 1000000000;
    normalizeSecNSec(sec, nsec);
    this->m_time = TimePoint(std::chrono::seconds(sec) + std::chrono::nanoseconds(nsec));
    return *this;
}

TimePoint
Time::getTimePoint() const
{
    return m_time;
}
  
Time
Time::now()
{
    Time t;
    t.m_time = std::chrono::steady_clock::now();
}

bool
Time::operator<(const Time& rhs)
{
    return m_time < rhs.m_time;
}

Time
Time::operator+(const Duration& dur)
{
    auto nanos = std::chrono::nanoseconds(dur.toNSec());

    Time newTime;
    newTime.m_time = this->m_time;
    newTime.m_time += nanos;
    return newTime;
}
  
Time&
Time::operator+=(const Duration& dur)
{
    auto nanos = std::chrono::nanoseconds(dur.toNSec());
    this->m_time += nanos;
    return *this;
}

Duration
Time::operator-(const Time& rhs)
{
    // Compute the remaining nanoseconds, by subtracting
    // the number of seconds converted to nanoseconds
    uint64_t nsec1 = this->toNSec() - this->toSec()*1e9;
    uint64_t nsec2 = rhs.toNSec() - rhs.toSec()*1e9;
  
    return Duration(
      (int32_t)this->toSec() - (int32_t)rhs.toSec(),
      nsec1 - nsec2);
}
  
void
Time::normalizeSecNSec(unsigned long& sec, unsigned long& nsec)
{
    unsigned long nsecPart = nsec % 1000000000UL;
    unsigned long secPart = nsec / 1000000000UL;
    sec += secPart;
    nsec = nsecPart;
}

}  // end of the ros2_time namespace
