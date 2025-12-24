#include "Task.h"

#include <cstddef>
#include "Arduino.h"

void Task::reset_state() {
  done = false;
  completion_event = "done";
}

void Task::begin() {}

void Task::end() {}

bool Task::is_done() {
  return done;
}

void Task::get_display_string(char * buffer, int buffer_size) {
  strncpy(buffer, name, buffer_size);
}

void Task::set_done(const char * event) {
  completion_event = event;
  done = true;
}

const char * Task::get_completion_event() {
  return completion_event;
}