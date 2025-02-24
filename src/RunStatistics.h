#include <esp_timer.h>
#include <cstdint>
#include <std_msgs/msg/string.h>
#include <cmath>
#include <string>

class RunStatistics {
  public:

  std::string name;

  // use esp32 timing functions
  int64_t start_time = 0;
  int64_t sum_elapsed_us = 0;
  int64_t last_elapsed_us = 0;
  int64_t max_elapsed_us = 0;
  int64_t sum_elapsed_us_squared = 0;
  int32_t count = 0;

  
  RunStatistics(const char * name) : name(name) {
  }

  void start() {
    start_time = esp_timer_get_time();
  }

  void stop() {
    last_elapsed_us = esp_timer_get_time() - start_time;
    sum_elapsed_us += last_elapsed_us;
    sum_elapsed_us_squared += last_elapsed_us * last_elapsed_us;
    count++;
    if (last_elapsed_us > max_elapsed_us) {
      max_elapsed_us = last_elapsed_us;
    }
  }

  int64_t mean() {
    if(count == 0) {
      return 0;
    }
    return sum_elapsed_us / count;
  }

  int64_t variance() {
    if(count == 0) {
      return 0;
    }
    return sum_elapsed_us_squared / count - mean() * mean();
  }

  int64_t stddev() {
    return sqrt(variance());
  }

  int64_t max() {
    return max_elapsed_us;
  }

  int64_t last() {
    return last_elapsed_us;
  }

  void to_log_msg( std_msgs__msg__String * log_msg) {
    log_msg->data.size =  snprintf(log_msg->data.data, log_msg->data.capacity, 
      "%12s: count=%8d mean=%8.3f stddev=%8.3f max=%8.3f total=%8.3f last=%8.3f",
      name.c_str(),
      count,
      mean()/1000000.,
      stddev()/1000000.,
      max()/1000000.,
      sum_elapsed_us/1000000.,
      last()/1000000.
    );
  }
};

class BlockTimer {
  public:
  RunStatistics & stats;
  BlockTimer(RunStatistics & stats) : stats(stats) {
    stats.start();
  }
  ~BlockTimer() {
    stats.stop();
  }
};