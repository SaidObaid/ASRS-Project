/****************************************************************
 * File Name: state_queue.cpp
 * Author:  Said Obaid, University of New Brunswick
 * Date: 01/08/2025
 * Description: Pointer-based, dynamically sized StateCommand queue
 ****************************************************************
 * Modification History:
 * [01-08-2025] - Original File Developed by Said Obaid.
 * [18-08-2025] - Converted to linked-list queue (no max size).
 * 
 ****************************************************************
 * License: MIT License as main.ino
 ****************************************************************
 * Questions/Comments: Please email Said Obaid at sobaid@unb.ca
 * Copyright 2025. Said Obaid, University of New Brunswick
 ****************************************************************/
#include "state_queue.h"

// Internal node for the linked-list queue
struct StateNode {
  StateCommand cmd;
  StateNode*   next;
};

void initQueue(StateQueue &q) {
  q.head  = nullptr;
  q.tail  = nullptr;
  q.count = 0;
}

bool enqueueState(StateQueue &q, const StateCommand &sc) {
  StateNode* node = new StateNode;
  if (!node) {
    Serial.println(F("Queue alloc failed. Command ignored."));
    return false;
  }
  node->cmd  = sc;
  node->next = nullptr;

  if (!q.tail) {
    // empty queue
    q.head = q.tail = node;
  } else {
    q.tail->next = node;
    q.tail       = node;
  }
  q.count++;
  return true;
}

bool enqueueState(StateQueue &q,
                  long stepDuration, long xCoord, long yCoord,
                  float xVel, float yVel,
                  bool gripper1State, bool piston1State,
                  bool gripper2State, bool piston2State)
{
  StateCommand sc{ stepDuration, xCoord, yCoord, xVel, yVel,
                   gripper1State, piston1State,
                   gripper2State, piston2State};
  return enqueueState(q, sc);
}

bool dequeueState(StateQueue &q, StateCommand &out) {
  if (!q.head) return false;

  StateNode* node = q.head;
  out   = node->cmd;
  q.head = node->next;
  if (!q.head) q.tail = nullptr;  // became empty

  delete node;
  q.count--;
  return true;
}

bool peekState(const StateQueue &q, StateCommand &out) {
  if (!q.head) return false;
  out = q.head->cmd;   // **no modification** to the queue
  return true;
}

void clearStateQueue(StateQueue &q) {
  while (q.head) {
    StateNode* next = q.head->next;
    delete q.head;
    q.head = next;
  }
  q.tail  = nullptr;
  q.count = 0;
  Serial.println(F("State queue cleared."));
}

long stateQueueSize(const StateQueue &q) {
  return q.count;
}

bool isStateQueueEmpty(const StateQueue &q) {
  return (q.count == 0);
}
