#include <ros2_time/duration.hpp>

// standard headers
#include <math.h>


namespace ros2_time
{


Duration::Duration()
    :
    sec(0.0),
    nsec(0)
{
}

Duration::Duration(const double& seconds)
    :
    sec((int32_t)trunc(seconds)),
    nsec((int32_t)((seconds - (double)sec)*1000000000))
{
}

Duration::Duration(const uint32_t& _sec, const uint32_t& _nsec)
  :
  sec(_sec),
  nsec(_nsec)
{
}

bool
Duration::sleep() const
{
    // TODO: this function needs to handle being canceled mid-sleep
    timespec req = {sec, nsec};
    timespec rem = {0, 0};
    while (nanosleep(&req, &rem) /*&& rclcpp::ok()*/) {
        req = rem;
    }

    return true; // rclcpp::ok();
}

double
Duration::toSec() const
{
    return (double)sec + 1e-9*(double)nsec;
}

int64_t
Duration::toNSec() const
{
    return (int64_t)sec*1000000000ll + (int64_t)nsec;
}

Duration&
Duration::fromSec(const double sec)
{
    this->sec = (int32_t)trunc(sec);
    this->nsec = (int32_t)((sec - (double)this->sec)*1000000000);
    normalizeSecNSecSigned(this->sec, this->nsec);
    return *this;
}

Duration&
Duration::fromNSec(const unsigned long nsec)
{
    this->sec = nsec / 1000000000UL;
    this->nsec = nsec % 1000000000UL;
    normalizeSecNSecSigned(this->sec, this->nsec);
    return *this;
}
  
bool
Duration::operator> (const Duration& other)
{
    if (this->sec > other.sec) return true;
    if (this->sec < other.sec) return false;
    return (this->nsec > other.nsec);
}

bool 
Duration::operator>= (const Duration& other)
{
    if (this->sec > other.sec) return true;
    if (this->sec < other.sec) return false;
    return (this->nsec >= other.nsec);
}

bool 
Duration::operator< (const Duration& other)
{
    if (this->sec < other.sec) return true;
    if (this->sec > other.sec) return false;
    return (this->nsec < other.nsec);
}

bool 
Duration::operator<= (const Duration& other)
{
    if (this->sec < other.sec) return true;
    if (this->sec > other.sec) return false;
    return (this->nsec <= other.nsec);
}

Duration
Duration::operator- (const Duration& other)
{
    Duration result;
    result.sec -= other.sec;
    result.nsec -= other.nsec;
    normalizeSecNSecSigned(result.sec, result.nsec);
    return result;
}

Duration
Duration::operator+ (const Duration& other)
{
    Duration result;
    result.sec += other.sec;
    result.nsec += other.nsec;
    normalizeSecNSecSigned(result.sec, result.nsec);
    return result;
}

void
Duration::normalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
{
    long nsecPart = nsec;
    long secPart = sec;

    while (nsecPart > 1000000000L) {
        nsecPart -= 1000000000L;
        ++secPart;
    }

    while (nsecPart < 0) {
         nsecPart += 1000000000L;
         --secPart;
    }

    sec = secPart;
    nsec = nsecPart;
}

std::ostream& operator<< (std::ostream& out, const Duration& dur)
{
    out << dur.toSec() << " seconds";
    return out;
}

}  // end of the ros2_time namespace
