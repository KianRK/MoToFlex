#ifndef _PRECISION_TIMER_H
#define _PRECISION_TIMER_H

#define _PROFILING_NUM_MEASUREMENTS_ 5

/*
 * Namespace providing precision timing functions
 * Moved here from ModuleManager
 * added multi-Measurement support
 * 
 * Usage: call start(idx), then tend(idx), tval(idx) returns time in seconds
 *        idx is an optional argument (defaults to 0) if you want to time several sections at once
 *
 * Note: Max idx is (_PROFILING_NUM_MEASUREMENTS_ - 1).
 *       This restriction is unchecked and might result in crashes if violated
 */

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include <cstring>

class PrecisionTimer {
public:
  PrecisionTimer();

  void tstart(size_t measurementIndex = 0);
  void tend(size_t measurementIndex = 0);
  double tval(size_t measurementIndex = 0);

private:
#ifdef _WIN32
 /*static PROCESS_WIDE_STORAGE*/
  //namespace {
    LARGE_INTEGER _tstart[_PROFILING_NUM_MEASUREMENTS_], _tend[_PROFILING_NUM_MEASUREMENTS_];
    LARGE_INTEGER freq;
  //}
#else
    struct timeval _tstart[_PROFILING_NUM_MEASUREMENTS_], _tend[_PROFILING_NUM_MEASUREMENTS_];
    struct timezone tz;
#endif
};

#endif
