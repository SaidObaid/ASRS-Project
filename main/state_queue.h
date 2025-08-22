/****************************************************************
 * File Name: state_queue.h
 * Author:  Said Obaid, University of New Brunswick
 * Date: 18/08/2025
 * Description: Public API for a pointer-based StateCommand queue
 ****************************************************************
 * Modification History:
 * [01-08-2025] - Original File Developed by Said Obaid.
 * [18-08-2025] - Switched to linked-list (no max size, no globals).
 * 
 ****************************************************************
 * License: MIT License as main.ino
 * Questions/Comments: Please email Said Obaid at sobaid@unb.ca
 * Copyright 2025. Said Obaid, University of New Brunswick
 ****************************************************************/

#ifndef STATE_QUEUE_H
#define STATE_QUEUE_H

#include <Arduino.h>

struct StateCommand {
  long  stepDuration;
  long  xCoord;
  long  yCoord;
  float xVel;
  float yVel;
  bool  gripper1State;
  bool  piston1State;
  bool  gripper2State;
  bool  piston2State;
};

// Forward-declare internal node type
struct StateNode;

// Instance-based queue (supports many queues at once)
struct StateQueue {
  StateNode* head;
  StateNode* tail;
  long       count;
};

// API
void initQueue(StateQueue &q);

// Enqueue using a ready command
bool enqueueState(StateQueue &q, const StateCommand &sc);

// Enqueue using fields (convenience overload)
bool enqueueState(StateQueue &q,
                  long stepDuration, long xCoord, long yCoord,
                  float xVel, float yVel,
                  bool gripper1State, bool piston1State,
                  bool gripper2State, bool piston2State);

// Dequeue front element into out. Returns false if empty.
bool dequeueState(StateQueue &q, StateCommand &out);

// Peek front element into out (without removing). Returns false if empty.
bool peekState(const StateQueue &q, StateCommand &out);

// Remove all nodes and reset
void clearStateQueue(StateQueue &q);

//  Helpers
long stateQueueSize(const StateQueue &q);
bool isStateQueueEmpty(const StateQueue &q);

#endif // STATE_QUEUE_H

