#pragma once
#include <stdlib.h>
#include "Task.h"
#include <vector>

class Fsm : public Task {
  public:
  // TBD: use numbers instead of strings?
  struct Edge {
    Edge(const char * from, const char * event, const char *);
    const char * from = NULL;
    const char * event = NULL;
    const char * to = NULL;
  };
  std::vector<Edge> edges;
  std::vector<Task*> tasks;

  // current task should be an iterator into tasks
  Task * current_task = NULL;

  void set_current_task(const char * name);

  Fsm(std::vector<Task*> tasks, std::vector<Edge> edges);
  void begin();
  void execute();
  void end();
  void set_event(const char * event);
};
