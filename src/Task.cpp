#include "Task.h"

#include <cstddef>
#include "Arduino.h"

void Task::begin() {}
void Task::end() {}
bool Task::is_done() {
  return done;
}
void Task::get_display_string(char * buffer, int buffer_size) {
  strncpy(buffer, name, buffer_size);
}