#include "Fsm.h"

#include <stdlib.h>
#include "Arduino.h"
#include <string.h> // including for strcomp had no impact on firmware size

bool equals(const char * p, const char * q) {
  return strcmp(p,q)==0;
}

Fsm::Edge::Edge(const char * _from, const char * _event, const char * _to) {
  from = _from;
  event = _event;
  to = _to;
}

Fsm::Fsm(std::vector<Task*> _tasks, std::vector<Edge> _edges) {
  tasks = _tasks;
  edges = _edges;
  name="fsm";
}

void Fsm::begin() {
  current_task = tasks[0];
  current_task->reset_state();
  current_task->begin();
}

void Fsm::execute() {
  if(done || current_task == NULL) {
    return;
  }

  current_task->execute();
  if(current_task->is_done()) {
    if(current_task != NULL) {
      set_event(current_task->get_completion_event());
    }
  }
}

void Fsm::set_current_task(const char * name) {
  if( equals(name, current_task->name) ) {
    return;
  }
  Serial.print("Switching to task ");
  Serial.println(name);
  
  if(equals(name,"done")) {
    done = true;
    current_task = this;
    return;
  }

  
  for(auto & task : tasks) {
    if(equals(task->name,name)){
      current_task->end();
      current_task = task;
      current_task->reset_state();
      current_task->begin();
      break;
    }
  }
}

void Fsm::end() {
  if(current_task && current_task != this) {
    current_task->end();
  }
  current_task = this;
  done = true;
}

void Fsm::set_event(const char * event) {
  // move based on event
  for (auto edge : edges) {
    // asterisk matches all tasks, only switch if not already in the to state
    if(equals(edge.from, "*") && equals(edge.event, event) && !equals(edge.to, current_task->name)) {
      set_current_task(edge.to);
      break;
    }
    else if(equals( edge.from, current_task->name ) && equals( edge.event, event )) {
      set_current_task(edge.to);
      break;
    }
  }
}

