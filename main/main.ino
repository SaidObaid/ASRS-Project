/****************************************************************
 * File Name: main.ino
 * Author:  Said Obaid, University of New Brunswick
 *          <Said.Obaid@UNB.ca> 
 *          (Current Maintainer)
 *          Alejandro Guarin, National University of Colombia 
 *          <aguarin@unal.edu.co> 
 *          (Previous Maintainer; ORIGINAL CODE)                
 * Date: 21/07/2025
 * Description: General movement program via RFID Card input or 
 * manual serial input.
 ****************************************************************
 * Modification History:
 * [2024] - Original File Developed by Alejandro Guarin.
 * [2025] - Said Obaid became continues works.
 * [21-07-2025] - Reversed direction for vertical axis to
 *                correct for new 1:5 Gearbox.
 * [22-07-2025] - Moved proximity alerts to ISR.
 *              - Collapsed if statements for motor
 *                halting upon sensor alert.
 *              - Changed "else if" to "if", becausing halting
 *                motors is not mutually exclusive.
 *              - haltHorizontal & haltVertical added so 
 *                halting motors can be stopped faster 
 *                following configured halting acceleration.
 *              - Removed generalSensor bool, no functionality
 *                was observed.
 *              - Added serial debugging command 'R' to reset
 *                sensors to 0.
 *              - Magic numbers switched with variables and
 *                moveToPosition renamed to moveJ.
 *              - moveJ now accounts for sensor triggers by
 *                halting motors and correcting current
 *                postions variable, to avoid accidental
 *                collisions if calibration was not
 *                initiated.
 * [23-07-2025] - Recalculated vertical step/mm to ±0.5 mm.  
 *                Previously was 216.72162 for old 1:16 gearbox.
 *                Currently is 66.87584 for new 1:5 gearbox.
 * [31-07-2025] - Added Coded travel limit with reference 
 *                to corner calibration datum x1500, y900.
 * [01-08-2025] - Restructured how movement is handle to avoid 
 *                blocking code.
 *              - Added on-the-go serial motor pause/resume.
 * [02-08-2025] - Began process of integrating a queue system to
 *                facilitate trajectory/command planning.
 * [##-08-2025] - Missed update logs but a lot happened in
 *                August.
 *              - Stepper drivers were adjusted using ST
 *                configurator software for the X-Axis and 
 *                header bridge of the G210 driver for Y-Axis.
 *                X-Axis: 2000 Steps/Revolution
 *                Y-Axis: 400  Steps/Revolution (Half Stepping)
 *              - ActionQueue as a separate method of organized
 *                commanding and for future integration with 
 *                FMS central requests through serial. 
 *                Commands only execute once the previous command 
 *                is complete, and pistons/gripper commands apply 
 *                a non-blocking delay (userAdjustable: 
 *                delayActionLength) to avoid collision.
 *              - Old non-queued commands remain and can be
 *                called by including '/' before the command.
 *                Main use is software motor pause or sensor
 *                reset, testing really.
 *              - Changed calibration routine to simultaneously
 *                calibrate X and Y axes. It is faster that way.
 * [27-08-2025] - Calibration sequence modified back to
 *                blocking code for reliabilty.
 * [28-08-2025] - Remade shelving Coordinates to be an actual
 *                3-Dimensionally index positions, 
 *                [Side][Col][Row][X/Y].
 * [2#-08-2025] - Began adding Alejandro's inventory management
 *                code, mainly utlizing the camera. Logs
 *                following are changes/additions I (Said) made.
 ****************************************************************
 * License: MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining 
 * a copy of this software and associated documentation files (the 
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to 
 * permit persons to whom the Software is furnished to do so, subject 
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ****************************************************************
 * Questions/Comments: Please email Said Obaid at sobaid@unb.ca
 * Copyright 2025. Said Obaid, University of New Brunswick
 ****************************************************************/
#include <SPI.h>
#include <MFRC522.h>
#include <TimerOne.h>
#include <AccelStepper.h>
#include <avr/pgmspace.h>

#include "state_queue.h"
#include "trajectory.h"


#define LED_PIN 13

#define SS_PIN 43 // Right: Near=43 Far=53
#define RST_PIN 44

#define PROX_SENSOR_UP 19 // Right and Up pin numbers switch around 
#define PROX_SENSOR_DOWN 2
#define PROX_SENSOR_LEFT 18
#define PROX_SENSOR_RIGHT 3

#define MOTOR1_STEP 6
#define MOTOR1_DIR 7
#define MOTOR1_ENABLE 5

#define MOTOR2_STEP 11
#define MOTOR2_DIR 12
#define MOTOR2_ENABLE 10

#define GRIPPER_PIN 32
#define PISTON_PIN 40

#define STEPS_PER_REVOLUTION_VERTICAL 400.0// This is with the Gecko set to Half Stepping
#define STEPS_PER_REVOLUTION_HORIZONTAL 2000.0 // This is with ST Configurator set to 2000 Step/Rev
#define PULLEY_DIAMETER  47.0
#define GEARBOX_REDUCTION 0.2 // 1:5 Gearbox


#define STEPS_PER_mm_VERTICAL    STEPS_PER_REVOLUTION_VERTICAL/(PULLEY_DIAMETER*M_PI*GEARBOX_REDUCTION)

#define STEPS_PER_mm_HORIZONTAL -STEPS_PER_REVOLUTION_HORIZONTAL/(PULLEY_DIAMETER*M_PI)

#define VERTICALINCREMENT_mm    1.0
#define HORIZONTALINCREMENT_mm  1.0
#define STANDARDINCREMENT_mm 1.0




MFRC522 rfid(SS_PIN, RST_PIN);

AccelStepper horizontalStepper(1, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper verticalStepper(1, MOTOR2_STEP, MOTOR2_DIR);


unsigned long referenceMillis, currentMillis, delayTimer;
unsigned long timeStep = 100, delayActionLength = 2000; //milliseconds

StateQueue trajQueue;
StateQueue actionQueue;

StateCommand trajState, actionState, peekingState;


float horizontalMaxSpeed = 1000,
      horizontalAcceleration = 500,
      verticalMaxSpeed = 1000,
      verticalAcceleration = 1000,
      haltAcceleration = 1000,
      horizontalLimitTestSpeed = 1000,
      verticalLimitTestSpeed = 500;

long xLowerLimit = 0,
     xUpperLimit = 1520,
     yLowerLimit = 0,
     yUpperLimit = 900,
     RFIDPlateLift_mm = 30,
     ShelfLift_mm = 20,
     ShelfDrop_mm = 0;


float yPosCurrent = 0,
      xPosCurrent = 0,
      recentEnqueueYPos = 0,
      recentEnqueueXPos = 0;

long moveMultiplier = 0;


volatile bool isUpTriggered, isDownTriggered, isLeftTriggered, isRightTriggered = false;

volatile bool isMoveComplete = false,
              isActionComplete = false,
              verticalHalted = false,
              horizontalHalted = false,
              motorsPaused = false,
              movementAllowed = false,
              playingQueue = false,
              delayAction = false;

bool piston1State = LOW, 
     gripper1State = LOW,
     piston2State = LOW, 
     gripper2State = LOW;



// Define shelf locations and corresponding RFID UIDs
struct Shelf {
  String uid;
  int xPosition;
  int yPosition;
};

// 3D array of shelfCoords for the storage cells (column, row) in millimeters
int shelfCoords[2][7][3][2] = { //[Side][Row][Col][X/Y]
  // Right Side [0]
  // Col [0]   Col [1]    Col [2]
  {
    {{800,0},  {1200,0},  {1520,0}},   // Row [0]
    {{800,127},{1200,127},{1520,127}}, // Row [1]
    {{800,282},{1200,282},{1520,282}}, // Row [2]
    {{800,433},{1200,433},{1520,433}}, // Row [3]
    {{800,583},{1200,583},{1520,583}}, // Row [4]
    {{800,735},{1200,735},{1520,735}}, // Row [5]
    {{800,887},{1200,887},{1520,887}}, // Row [6]
  },
  // Left Side [1]
  { 
    {{525,0},  {920,0},  {1320,0}},
    {{525,140},{920,140},{1320,140}},
    {{525,295},{920,295},{1320,295}},
    {{525,445},{920,445},{1320,445}},
    {{525,595},{920,595},{1320,595}},
    {{525,745},{920,745},{1320,745}},
    {{525,890},{920,890},{1320,890}},
  }
};

int bufferCoords[2][2][2] = {
    {{280,450}, {460,450}},
    {{5,460}, {185,460}}
};


Shelf shelves[] = {
  {"82f059c", shelfCoords[1][4][0], shelfCoords[1][4][1]},
  {"b3281128", shelfCoords[2][2][0], shelfCoords[2][2][1]},
  {"5ed146c9", shelfCoords[0][4][0], shelfCoords[0][4][1]},
  {"e27065c", shelfCoords[0][0][0], shelfCoords[0][0][1]},
  {"535e4128", shelfCoords[0][6][0], shelfCoords[0][6][1]},
  {"6c29dcb6", shelfCoords[2][0][0], shelfCoords[2][0][1]},
  {"233fa113", shelfCoords[2][6][0], shelfCoords[2][6][1]},
};

void setup() {
  
  pinMode(LED_PIN, OUTPUT); 
  Timer1.initialize(500000);  // 150 ms
  Timer1.attachInterrupt(blinkingLed);

  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
  initQueue(trajQueue);
  initQueue(actionQueue);

  pinMode(PROX_SENSOR_UP, INPUT);
  pinMode(PROX_SENSOR_DOWN, INPUT);
  pinMode(PROX_SENSOR_LEFT, INPUT);
  pinMode(PROX_SENSOR_RIGHT, INPUT);

  horizontalStepper.setSpeed(horizontalLimitTestSpeed);
  horizontalStepper.setMaxSpeed(horizontalMaxSpeed);
  horizontalStepper.setAcceleration(horizontalAcceleration);

  verticalStepper.setSpeed(verticalLimitTestSpeed);
  verticalStepper.setMaxSpeed(verticalMaxSpeed);
  verticalStepper.setAcceleration(verticalAcceleration);
  
  pinMode(GRIPPER_PIN, OUTPUT);
  pinMode(PISTON_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_UP), sensorUp_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_DOWN), sensorDown_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_LEFT), sensorLeft_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_RIGHT), sensorRight_ISR, FALLING);
  delay(1000);
  //calibrateLimits();
  Serial.println("\nSystem initialized. Bring an RFID card close...");
  referenceMillis = millis();
}

void loop() {
  currentMillis = millis();  

  updateActions();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    updateActions();
    if (input.indexOf('>') > 0 || input.indexOf('<') > 0 ) {
      int bufferCommaIndex = input.indexOf(',');
      
      // (sideSelect,bufferIdx)
      int sideSelect = input.substring(1, bufferCommaIndex).toInt();
      int bufferParenIndex = input.indexOf(')');
      int bufferIdx = input.substring(bufferCommaIndex + 1, bufferParenIndex).toInt();

      int stackParen1Index = input.indexOf('(',bufferParenIndex+1);
      int stackParen2Index = input.indexOf(')',bufferParenIndex+1);
      int stackCommaIndex = input.indexOf(',',bufferParenIndex);

      int stackROWIdx = input.substring(stackParen1Index+1, stackCommaIndex).toInt();
      int stackCOLIdx = input.substring(stackCommaIndex+1, stackParen2Index).toInt();

      if (input.indexOf('>') > 0) {
        Serial.println("Side: " + String(sideSelect) + " | [Buffer: " + String(bufferIdx) + "] > [Stack: ROW:"+  String(stackROWIdx) + ", COL:" + String(stackCOLIdx) + "]");
        putAway(sideSelect,bufferIdx,stackROWIdx,stackCOLIdx);
      }
      else {
        Serial.println("Side: " + String(sideSelect) + " | [Stack: ROW:" + String(stackROWIdx) + ", COL:" + String(stackCOLIdx) + "] > [Buffer: " + String(bufferIdx) + "]");
        retrieve(sideSelect,bufferIdx,stackROWIdx,stackCOLIdx);
      }


    }
    updateActions();
    if (input.indexOf(',') > 0 && (input.indexOf('(') == -1)) {
      char delimiter = ',';
      int commaIndex = input.indexOf(delimiter);

      if (commaIndex > 0) { 
        float distanceHorizontal = input.substring(0, commaIndex).toFloat(); // Distancia para el motor vertical
        float distanceVertical = input.substring(commaIndex + 1).toFloat(); // Distancia para el motor horizontal

        moveJ(distanceHorizontal, distanceVertical);
      }
    }
    else if (input.length() >= 1) {
      char command = input.charAt(0);
      char directCommand = input.charAt(1);
      
      if (command == '/'){
        input.remove(0,1);
        moveMultiplier = parseCommandValue(input);
        switch (toupper(directCommand)) {
          case 'P': digitalWrite(PISTON_PIN, piston1State = !piston1State); break;
          case 'G': digitalWrite(GRIPPER_PIN, gripper1State = !gripper1State); break;
          case 'D': moveJ(xPosCurrent - HORIZONTALINCREMENT_mm * moveMultiplier, yPosCurrent); break;
          case 'A': moveJ(xPosCurrent + HORIZONTALINCREMENT_mm * moveMultiplier, yPosCurrent); break;
          case 'S': moveJ(xPosCurrent, yPosCurrent - VERTICALINCREMENT_mm * moveMultiplier); break;
          case 'W': moveJ(xPosCurrent, yPosCurrent + VERTICALINCREMENT_mm * moveMultiplier); break;
          case 'B': pauseMotors(); break;
          case 'H': calibrateLimits(); break;
          case 'O': goToOrigin(); break;
          case 'T': operateGripperAndPiston(1); break;
          case 'F': operateGripperAndPiston(2); break;
          case 'Q': enqueueTrajectory(); break;
          case 'R': sensorReset(); break;
          default : Serial.println("Unknown Direct '/' Command"); break;
        } 
        recentEnqueueXPos = xPosCurrent;
        recentEnqueueYPos = yPosCurrent;
      }
      else {
        moveMultiplier = parseCommandValue(input);
        switch (toupper(command)) {
          case 'P': enqueueActuator(moveMultiplier, true, false);
                    break;
          case 'G': enqueueActuator(moveMultiplier, false, true);
                    break;
          case 'X': recentEnqueueXPos = xPosCurrent + HORIZONTALINCREMENT_mm * moveMultiplier;
                    recentEnqueueYPos = yPosCurrent;
                    enqueueAction(recentEnqueueXPos, recentEnqueueYPos,
                                  LOW,LOW,LOW,LOW);
                                  break;
          case 'Y': recentEnqueueXPos = xPosCurrent;
                    recentEnqueueYPos = yPosCurrent + VERTICALINCREMENT_mm * moveMultiplier;
                    enqueueAction(recentEnqueueXPos, recentEnqueueYPos,
                                  LOW,LOW,LOW,LOW);
                                  break;
          case 'H': calibrateLimits(); break;
          default : Serial.println("Unknown Command"); break;
        }
      }
      updateActions();
      
    }
      
  }

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String uidStr = getUIDString();
    Serial.print("Detected UID: ");
    Serial.println(uidStr);

    for (int i = 0; i < sizeof(shelves)/sizeof(Shelf); i++) {
      if (shelves[i].uid == uidStr) {
        Serial.println("Moving to shelf location...");
        operateGripperAndPiston(1);
        moveJ(shelves[i].xPosition, shelves[i].yPosition);
        Serial.println(shelves[i].xPosition);
        Serial.println(shelves[i].yPosition);
        operateGripperAndPiston(0);
        goToOrigin();
        break;
      }
    }
    rfid.PICC_HaltA();
  }

  if ((isUpTriggered || isDownTriggered) && !verticalHalted){
    yPosCurrent = haltVertical(haltAcceleration);
    verticalStepper.setCurrentPosition(yPosCurrent*STEPS_PER_mm_VERTICAL);
  }
  if ((isRightTriggered || isLeftTriggered) && !horizontalHalted){
    xPosCurrent = haltHorizontal(haltAcceleration);
    horizontalStepper.setCurrentPosition(xPosCurrent*STEPS_PER_mm_HORIZONTAL);
  }
  
}

void moveJ(float xPos, float yPos) {
  isMoveComplete = false;
  
  xPos = constrain(xPos, xLowerLimit, xUpperLimit);
  yPos = constrain(yPos, yLowerLimit, yUpperLimit);

  // Move to the specified position
  if (!(isRightTriggered || isLeftTriggered)){
    horizontalStepper.moveTo(xPos * STEPS_PER_mm_HORIZONTAL);
    xPosCurrent = xPos;
    recentEnqueueXPos = xPosCurrent;
  }
  else{Serial.println("A Horizontal Sensor Was Triggered");}

  if (!(isUpTriggered || isDownTriggered)){
    verticalStepper.moveTo(yPos * STEPS_PER_mm_VERTICAL);
    yPosCurrent = yPos;
    recentEnqueueYPos = yPosCurrent;
  }
  else{Serial.println("A Vertical Sensors Was Triggered");}

}

void pauseMotors(){
  motorsPaused = !motorsPaused;

  if (motorsPaused) {
    Serial.println("Motors PAUSED");
    long xPosTemp = xPosCurrent;
    long yPosTemp = yPosCurrent;
    xPosCurrent = haltHorizontal(haltAcceleration);
    yPosCurrent = haltVertical(haltAcceleration);
    stateCurrentPosition();
    moveJ(xPosTemp,yPosTemp);
  }
  else {
    Serial.println("Motors RESUMED");
  }
}

void updateActions(){
  movementAllowed = isMovementAllowed();

  if (delayAction && (millis() - delayTimer >= delayActionLength)){
    delayAction = false;
  }

  if ((stateQueueSize(actionQueue) != 0 ) && (!delayAction && isDistanceToPosZero()) && !allSensors()){
    if (isDistanceToPosZero()){ // I FIX THIS YESTERDAY
      dequeueState(actionQueue, actionState);
      Serial.println("Dequeued Action");
      moveJ(actionState.xCoord, actionState.yCoord);
      if (actionState.piston1State == HIGH || actionState.gripper1State == HIGH){
        if (actionState.gripper1State == HIGH){
          digitalWrite(GRIPPER_PIN, gripper1State = !gripper1State);
        }
        if (actionState.piston1State == HIGH ){
          digitalWrite(PISTON_PIN, piston1State = !piston1State);
        }
        delayAction = true;
        delayTimer = millis();
      }
    }
  }

  if ((!isDistanceToPosZero() || (stateQueueSize(trajQueue) > 0) ) && (!motorsPaused)) {
    while (movementAllowed || (stateQueueSize(trajQueue) > 0) || playingQueue){
      currentMillis = millis();
      if (playingQueue && (stateQueueSize(trajQueue) == 0)){
        if(currentMillis - referenceMillis >= timeStep){
          playingQueue = false;
          horizontalStepper.setSpeed(horizontalLimitTestSpeed);
          verticalStepper.setSpeed(verticalLimitTestSpeed);
          horizontalStepper.setMaxSpeed(horizontalMaxSpeed);
          verticalStepper.setMaxSpeed(verticalMaxSpeed);
          referenceMillis = currentMillis;
        }
      }

      if (stateQueueSize(trajQueue) > 0){
        playingQueue = true;
        horizontalStepper.setMaxSpeed(1000);
        verticalStepper.setMaxSpeed(1000);
        if(currentMillis - referenceMillis >= timeStep){
          dequeueState(trajQueue, trajState);
          moveJ(trajState.xCoord, trajState.yCoord);
          
          timeStep = trajState.stepDuration;
          referenceMillis = currentMillis;
        }
        while (currentMillis - referenceMillis < timeStep) {
          horizontalStepper.setSpeed(trajState.xVel*STEPS_PER_mm_HORIZONTAL);
          verticalStepper.setSpeed(trajState.yVel*STEPS_PER_mm_VERTICAL); //
          horizontalStepper.runSpeedToPosition();
          verticalStepper.runSpeedToPosition();
          currentMillis = millis();
        }
      }
      else{
        horizontalStepper.run();
        verticalStepper.run();
        movementAllowed = isMovementAllowed() && !isDistanceToPosZero();
      }
    }

    if (!motorsPaused && isDistanceToPosZero()){
      Serial.println("Move Complete");
      stateCurrentPosition();
    }
  }
}

String getUIDString() {
  String uidStr = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uidStr += String(rfid.uid.uidByte[i], HEX);
  }
  return uidStr;
}

void goToOrigin(){
  moveJ(215,460);
}

void syncVerticalStepperToState(){
  verticalStepper.setCurrentPosition(yPosCurrent*STEPS_PER_mm_VERTICAL);
}

void syncHorizontalStepperToState(){
  horizontalStepper.setCurrentPosition(xPosCurrent*STEPS_PER_mm_HORIZONTAL);
}

void calibrateLimits(){
  Serial.println("CALIBRATION IN PROGRESS");
  actuatorReset();
  while (!isDownTriggered || !isLeftTriggered) {
    if(!isDownTriggered){
      verticalStepper.setSpeed(-verticalLimitTestSpeed);
      verticalStepper.runSpeed();
    }
    
    if(!isLeftTriggered){
      horizontalStepper.setSpeed(horizontalLimitTestSpeed);
      horizontalStepper.runSpeed();
    }
  }
  
  verticalStepper.setCurrentPosition(0);
  horizontalStepper.setCurrentPosition(0);
  verticalStepper.moveTo(5 * STEPS_PER_mm_VERTICAL);
  horizontalStepper.moveTo(5 * STEPS_PER_mm_HORIZONTAL);
  while (!isDistanceToPosZero()) {
    verticalStepper.run();
    horizontalStepper.run();
  }
  verticalStepper.setCurrentPosition(0);
  horizontalStepper.setCurrentPosition(0);
  positionReset();
  sensorReset();
  Serial.println("CALIBRATION COMPLETE");
}

void operateGripperAndPiston(int action){
  //Take from home
  if(action == 1){
    moveJ(0, 390);
    digitalWrite(PISTON_PIN, HIGH);
    delay(1600);
    digitalWrite(GRIPPER_PIN, HIGH);
    delay(700);
    moveJ(0, 400);
    digitalWrite(PISTON_PIN, LOW);
    delay(1000);
  }
  // Leave in home
  else if(action == 2){
    moveJ(0, 400);
    digitalWrite(PISTON_PIN, HIGH);
    delay(1600);
    digitalWrite(GRIPPER_PIN, LOW);
    delay(700);
    digitalWrite(PISTON_PIN, LOW);
    delay(1000);
  }
  //Leave in place
  else{
    digitalWrite(PISTON_PIN, HIGH);
    delay(1600);
    digitalWrite(GRIPPER_PIN, LOW);
    delay(700);
    digitalWrite(PISTON_PIN, LOW);
    delay(1000);
  }
}

void enqueueTrajectory(){
  TrajectoryPoint tp;
  for (size_t i = 0; i < trajectoryLen; i++) {
    memcpy_P(&tp, &trajectoryData[i], sizeof(tp));
    enqueueState(trajQueue, tp.step_ms, tp.x_mm, tp.y_mm,
                 tp.vx_mmps, tp.vy_mmps,
                 tp.gripper1State, tp.piston1State,
                 tp.gripper2State, tp.piston2State);
  }
}

void enqueueAction(long xPosCMD, long yPosCMD, bool gripper1CMD, bool piston1CMD, bool gripper2CMD, bool piston2CMD){
  xPosCMD = constrain(xPosCMD, xLowerLimit, xUpperLimit);
  yPosCMD = constrain(yPosCMD, yLowerLimit, yUpperLimit);
  enqueueState(actionQueue, -1, xPosCMD, yPosCMD, -1, -1,
              gripper1CMD, piston1CMD, gripper2CMD, piston2CMD);
  Serial.println("Enqueued Postion: X: " + String(xPosCMD) + ", Y: " + String(yPosCMD));
}

void enqueueActuator(int sideSelect, bool togglePiston, bool toggleGripper){
  bool toggleP1 = LOW;
  bool toggleP2 = LOW;
  bool toggleG1 = LOW;
  bool toggleG2 = LOW;
  
  if (togglePiston == true){
    if (sideSelect == 0) {
      toggleP1 = HIGH;
    }
    else{
      toggleP2 = HIGH;
    }
  }

  if (toggleGripper == true){
    if (sideSelect == 0) {
      toggleG1 = HIGH;
    }
    else{
      toggleG2 = HIGH;
    }
  }
  enqueueAction(recentEnqueueXPos, recentEnqueueYPos,
                toggleG1, toggleP1, toggleG2, toggleP2);
}

void retrieve(int sideSelect, long bufferIdx, long stackROWIdx, long stackCOLIdx) {
  actuatorReset();
  enqueueAction(recentEnqueueXPos = shelfCoords[sideSelect][stackROWIdx][stackCOLIdx][0],
                recentEnqueueYPos = shelfCoords[sideSelect][stackROWIdx][stackCOLIdx][1],
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,true,false);
  enqueueActuator(sideSelect,false,true);
  enqueueAction(recentEnqueueXPos = recentEnqueueXPos,
                recentEnqueueYPos = recentEnqueueYPos + ShelfLift_mm,
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,true,false);

  enqueueAction(recentEnqueueXPos = bufferCoords[sideSelect][bufferIdx][0],
                recentEnqueueYPos = bufferCoords[sideSelect][bufferIdx][1] + RFIDPlateLift_mm,
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,true,false);
  enqueueAction(recentEnqueueXPos = recentEnqueueXPos,
                recentEnqueueYPos = recentEnqueueYPos - RFIDPlateLift_mm,
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,false,true);
  enqueueActuator(sideSelect,true,false);
}

void putAway(int sideSelect, long bufferIdx, long stackROWIdx, long stackCOLIdx) {
  actuatorReset();
  recentEnqueueXPos = bufferCoords[sideSelect][bufferIdx][0];
  recentEnqueueYPos = bufferCoords[sideSelect][bufferIdx][1];
  enqueueAction(recentEnqueueXPos = bufferCoords[sideSelect][bufferIdx][0],
                recentEnqueueYPos = bufferCoords[sideSelect][bufferIdx][1],
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,true,false);
  enqueueActuator(sideSelect,false,true);
  enqueueAction(recentEnqueueXPos = recentEnqueueXPos,
                recentEnqueueYPos = recentEnqueueYPos + RFIDPlateLift_mm,
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,true,false);

  enqueueAction(recentEnqueueXPos = shelfCoords[sideSelect][stackROWIdx][stackCOLIdx][0],
                recentEnqueueYPos = shelfCoords[sideSelect][stackROWIdx][stackCOLIdx][1]+ShelfLift_mm,
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,true,false);
  enqueueAction(recentEnqueueXPos = recentEnqueueXPos,
                recentEnqueueYPos = recentEnqueueYPos-ShelfLift_mm,
                LOW,LOW,LOW,LOW);
  enqueueActuator(sideSelect,false,true);
  enqueueActuator(sideSelect,true,false);
}

void blinkingLed() {
  static bool ledState = LOW; // Estado del LED (LOW = apagado, HIGH = encendido)
  ledState = !ledState; // Alternar el estado del LED
  digitalWrite(LED_PIN, ledState); // Escribir el nuevo estado del LED
}

void sensorUp_ISR()     { Serial.println("UP SENSOR PROXIMITY ACTIVATED"); isUpTriggered = true; }
void sensorDown_ISR()   { Serial.println("DOWN SENSOR PROXIMITY ACTIVATED"); isDownTriggered = true; }
void sensorLeft_ISR()   { Serial.println("LEFT SENSOR PROXIMITY ACTIVATED"); isLeftTriggered = true; }
void sensorRight_ISR()  { Serial.println("RIGHT SENSOR PROXIMITY ACTIVATED"); isRightTriggered = true; }
bool allSensors()       { return isUpTriggered && isDownTriggered && isLeftTriggered && isRightTriggered;}
void sensorReset()      { isUpTriggered = isDownTriggered = isLeftTriggered = isRightTriggered = verticalHalted = horizontalHalted = false; }

float haltHorizontal(float haltAcceleration){
  horizontalStepper.setAcceleration(haltAcceleration);
  horizontalStepper.stop();
  horizontalStepper.setAcceleration(horizontalAcceleration);
  horizontalHalted = true;
  return ((float)horizontalStepper.currentPosition())/((float)STEPS_PER_mm_HORIZONTAL);
}

float haltVertical(float haltAcceleration){
  verticalStepper.setAcceleration(haltAcceleration);
  verticalStepper.stop();
  verticalStepper.setAcceleration(verticalAcceleration);
  verticalHalted = true;
  
  return ((float)verticalStepper.currentPosition())/((float)STEPS_PER_mm_VERTICAL);
}

void stateCurrentPosition(){
  Serial.println("X: " + String(xPosCurrent) + ", Y: " + String(yPosCurrent));
}

void positionReset() {
  yPosCurrent       = 0;
  xPosCurrent       = 0;
  recentEnqueueYPos = 0;
  recentEnqueueXPos = 0;
}

void actuatorReset() {
  piston1State  = LOW; 
  gripper1State = LOW;
  piston2State  = LOW; 
  gripper2State = LOW;
  digitalWrite(PISTON_PIN, piston1State);
  digitalWrite(GRIPPER_PIN, gripper1State);
}

long parseCommandValue(const String& input){
  return (input.length() <= 1) ? 1 : input.substring(1).toFloat();
}

bool isMovementAllowed(){
  return((Serial.available() <= 0)
         && (!(isUpTriggered || isDownTriggered))
         && (!(isRightTriggered || isLeftTriggered)));
}

bool isDistanceToPosZero(){
  return (horizontalStepper.distanceToGo() == 0 && verticalStepper.distanceToGo() == 0);
}
