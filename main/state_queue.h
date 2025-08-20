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

enum Side {
  Right = 0,
  Left  = 1
};

struct StateCommand {
  long  stepDuration;
  long  xCoord;
  long  yCoord;
  float xVel;         // NOTE: velocities are float in the command struct
  float yVel;
  Side  sideSelect;
  bool  gripperState;
  bool  pistonState;
};

// Queue (internals are hidden in state_queue.cpp)
bool enqueueState(long stepDuration, long xCoord, long yCoord,
                  float xVel, float yVel,
                  Side sideSelect, bool gripperState, bool pistonState);

bool dequeueState(StateCommand &cmd);

void clearStateQueue();

long stateQueueSize();
bool isStateQueueEmpty();

#endif // STATE_QUEUE_H
