/****************************************************************
 * File Name: state_queue.h
 * Author:  Said Obaid, University of New Brunswick
 * Date: 01/08/2025
 * Description: -
 ****************************************************************
 * Modification History:
 * [01-08-2025] - Original File Developed by Said Obaid.
 * 
 ****************************************************************
 * License: MIT License as main.ino
 ****************************************************************
 * Questions/Comments: Please email Said Obaid at sobaid@unb.ca
 * Copyright 2025. Said Obaid, University of New Brunswick
 ****************************************************************/

#ifndef STATE_QUEUE_H
#define STATE_QUEUE_H

#define STATE_QUEUE_SIZE 10


enum Side {
  Right = 0,
  Left  = 1
};

struct StateCommand {
  long xCoord;
  long yCoord;
  float xVel;
  float yVel;
  Side sideSelect;
  bool gripperState;
  bool pistonState;
};


extern StateCommand stateQueue[];
extern int queueHead;
extern int queueTail;
extern int queueCount;

bool enqueueState(long xCoord, long yCoord, long xVel, long yVel, Side sideSelect, bool gripperState, bool pistonState);
bool dequeueState(StateCommand &cmd);
void clearStateQueue();

#endif
