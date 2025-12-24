#pragma once

class Task {
protected:
  bool done = false;
  const char * completion_event = "done";

public:
  const char * name = 0; // Set in constructor, used in FSM edges
  
  // Framework methods (do not call directly)
  void reset_state();
  virtual bool is_done();
  const char * get_completion_event();
  
  // Lifecycle methods (override in subclass)
  virtual void begin();
  virtual void end();
  virtual void execute() = 0;
  virtual void get_display_string(char * buffer, int buffer_size);
  
  // Public API (call from your execute() method)
  void set_done(const char * event = "done");
};
