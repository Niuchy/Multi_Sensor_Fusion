#ifndef __TIME_H_
#define __TIME_H_

#include <chrono>
#include <time.h>
#include <string>

namespace PQTIME
{
    int64_t getCurrentTimestampMs();

    int getDay(time_t time = 0);

    std::string time2str(const std::string &fmt = "%Y-%m-%d %H:%M:%S", time_t time = 0);
}

#endif //_TIME_H_