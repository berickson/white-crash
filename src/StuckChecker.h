#pragma once
#include <Arduino.h>

#include <cmath>

class StuckChecker {
 private:
  const int move_threshold;
  const int stuck_time_threshold_ms;
  const float min_throttle;
  int last_odometer_value = 0;
  unsigned long stuck_start_ms = 0;
  bool stuck = false;

 public:
  StuckChecker(int move_threshold = 10, int stuck_time_threshold_ms = 1000, float min_throttle = 0.1)
      : move_threshold(move_threshold), stuck_time_threshold_ms(stuck_time_threshold_ms), min_throttle(min_throttle) {}

  void update(float throttle, int current_odometer) {
    if (abs(throttle) < min_throttle) {
      stuck_start_ms = 0;
      last_odometer_value = current_odometer;
      stuck = false;

      return;
    }

    if (abs(current_odometer - last_odometer_value) < move_threshold) {
      if (stuck_start_ms == 0) {
        stuck_start_ms = millis();
      }
    } else {
      stuck_start_ms = 0;
    }
    
    last_odometer_value = current_odometer;

    if ((abs(throttle) >= min_throttle) && (stuck_start_ms != 0) && ((millis() - stuck_start_ms) > stuck_time_threshold_ms)) {
      stuck = true;
    } else {
      stuck = false;
    }
  }

  bool is_stuck() const {
    return stuck;
  }
};