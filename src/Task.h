#pragma once

class Task {
public:
  const char * name = 0;
  bool done = false;
  virtual void begin();
  virtual void end();
  virtual void execute() = 0;
  virtual bool is_done();
};
