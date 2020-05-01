#ifndef ORCA_FILTER_PERF_H
#define ORCA_FILTER_PERF_H

#undef RUN_PERF
#ifdef RUN_PERF

#define NUM_SAMPLES 100

  void print_mean(std::string msg, int samples[]);

#define START_PERF()\
static int _samples_[NUM_SAMPLES];\
static int _index_ = 0;\
auto _start_ = std::chrono::high_resolution_clock::now();

#define STOP_PERF(msg)\
auto _stop_ = std::chrono::high_resolution_clock::now();\
_samples_[_index_] = std::chrono::duration_cast<std::chrono::microseconds>(_stop_ - _start_).count();\
if (++_index_ >= NUM_SAMPLES) { print_mean(msg, _samples_); _index_ = 0; }\

#else
#define START_PERF()
#define STOP_PERF(msg)
#endif

#endif // ORCA_FILTER_PERF_H