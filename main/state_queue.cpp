/****************************************************************
 * File Name: state_queue.cpp
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
#include "state_queue.h"
#include <Arduino.h>

StateCommand stateQueue[STATE_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

bool enqueueState(long xCoord, long yCoord, long xVel, long yVel, Side sideSelect, bool gripperState, bool pistonState) {
  if (queueCount >= STATE_QUEUE_SIZE) {
    Serial.println("Queue full. Command ignored.");
    return false;
  }
  stateQueue[queueTail] = {xCoord, yCoord, xVel, yVel, sideSelect, gripperState, pistonState};
  queueTail = (queueTail + 1);
  queueCount++;
  return true;
}

bool dequeueState(StateCommand &state) {
  if (queueCount == 0) return false;
  state = stateQueue[queueHead];
  queueHead = (queueHead + 1) % STATE_QUEUE_SIZE;
  queueCount--;
  return true;
}

void clearStateQueue() {
  queueHead = queueTail = queueCount = 0;
  Serial.println("State queue cleared.");
}