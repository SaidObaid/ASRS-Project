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
#include <Arduino.h>

// Internal node for the linked-list queue
struct StateNode {
  StateCommand cmd;
  StateNode* next;
};

// Queue state (head = front/dequeue, tail = back/enqueue)
static StateNode* headNode = nullptr;
static StateNode* tailNode = nullptr;
static long     queueCount = 0;

bool enqueueState(long stepDuration, long xCoord, long yCoord, float xVel,float yVel, 
                  Side sideSelect, bool gripperState, bool pistonState) {
  // Prepare the command
  StateCommand sc = { stepDuration, xCoord, yCoord, xVel, yVel, sideSelect, gripperState, pistonState };

  // Allocate a new node
  StateNode* node = new StateNode;
  if (!node) {
    Serial.println("Queue alloc failed. Command ignored.");
    return false;
  }
  node->cmd = sc;
  node->next = nullptr;

  // Link into list
  if (!tailNode) {
    // Empty queue case
    headNode = tailNode = node;
  } else {
    tailNode->next = node;
    tailNode = node;
  }

  queueCount++;
  return true;
}

bool dequeueState(StateCommand &state) {
  if (!headNode) {
    return false; // empty
  }

  // Pop from front
  StateNode* node = headNode;
  state = node->cmd;
  headNode = node->next;

  if (!headNode) {
    // Became empty
    tailNode = nullptr;
  }

  delete node;
  queueCount--;
  return true;
}

void clearStateQueue() {
  // Free all nodes
  while (headNode) {
    StateNode* next = headNode->next;
    delete headNode;
    headNode = next;
  }
  tailNode = nullptr;
  queueCount = 0;
  Serial.println("State queue cleared.");
}

long stateQueueSize() {
  return queueCount;
}

bool isStateQueueEmpty() {
  return (queueCount == 0);
}
