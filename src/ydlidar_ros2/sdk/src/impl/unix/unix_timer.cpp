#if !defined(_WIN32)
#include "timer.h"

namespace impl {
uint32_t getHDTimer() {
  struct timespec t;
  t.tv_sec = t.tv_nsec = 0;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
}
uint64_t getCurrentTime() {
#if HAS_CLOCK_GETTIME
  struct timespec  tim;
  clock_gettime(CLOCK_REALTIME, &tim);
  return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  return static_cast<uint64_t>(timeofday.tv_sec) * 1000000000LL +
         static_cast<uint64_t>(timeofday.tv_usec) * 1000LL;
#endif
}
}
#endif
