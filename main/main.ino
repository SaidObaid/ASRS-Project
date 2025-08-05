/****************************************************************
 * File Name: main.ino
 * Author:  Said Obaid, University of New Brunswick
 *          Alejandro Guarin, <Your Institution> (ORIGINAL CODE)     
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

#include "state_queue.h"

#define LED_PIN 13

#define SS_PIN 53
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

#define STEPS_PER_mm_VERTICAL   66.87584
#define STEPS_PER_mm_HORIZONTAL -13.5451

#define VERTICALINCREMENT_mm    1.0
#define HORIZONTALINCREMENT_mm  1.0

#define TIMESTEP 50.0


MFRC522 rfid(SS_PIN, RST_PIN);

AccelStepper horizontalStepper(1, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper verticalStepper(1, MOTOR2_STEP, MOTOR2_DIR);

String msgQueued;

unsigned long referenceMillis;
unsigned long currentMillis;

float horizontalMaxSpeed = 1000;
float horizontalAcceleration = 300;
const float verticalMaxSpeed = 3000;
float verticalAcceleration = 1000;
const float haltAcceleration = 1000;

float horizontalLimitTestSpeed = 1000;
float verticalLimitTestSpeed = 2000;

long xLowerLimit = 0;
long xUpperLimit = 1500;
long yLowerLimit = 0;
long yUpperLimit = 900;


long yPos = 0;
long xPos = 0;

long yPosTemp = 0;
long xPosTemp = 0;

long yPosCurrent = 0;
long xPosCurrent = 0;


volatile bool isUpTriggered = false;
volatile bool isDownTriggered = false;
volatile bool isLeftTriggered = false;
volatile bool isRightTriggered = false;

volatile bool isMoveComplete = false;
volatile bool isActionComplete = false;
volatile bool isPaused = false;
volatile bool isMoving = false;

bool PistonState = LOW;
bool GripperState = LOW;



// Define shelf locations and corresponding RFID UIDs
struct Shelf {
  String uid;
  int xPosition;
  int yPosition;
};

// Matriz de coordenadas del estante (columna, fila) en mm
int coordinates[3][7][2] = {
//     FILA 1      FILA 2       FILA 3       FILA 4      FILA 5        FILA 6        FILA 7
    {{450,  10}, {450,  160},  {450,  280},  {450,  480},  {450,  640},  {450,  800},  {450,  960}},  // COLUMNA 1
    {{850,  10}, {850,  160},  {850,  280},  {850,  480},  {850,  640},  {850,  800},  {850,  960}},  // COLUMNA 2
    {{1300, 10}, {1300, 160},  {1300, 280},  {1300, 480},  {1300, 640},  {1300, 800},  {1300, 960}}  // COLUMNA 3
};

Shelf shelves[] = {
  {"82f059c", coordinates[1][4][0], coordinates[1][4][1]},
  {"b3281128", coordinates[2][2][0], coordinates[2][2][1]},
  {"5ed146c9", coordinates[0][4][0], coordinates[0][4][1]},
  {"e27065c", coordinates[0][0][0], coordinates[0][0][1]},
  {"535e4128", coordinates[0][6][0], coordinates[0][6][1]},
  {"6c29dcb6", coordinates[2][0][0], coordinates[2][0][1]},
  {"233fa113", coordinates[2][6][0], coordinates[2][6][1]},
};

void setup() {

  pinMode(LED_PIN, OUTPUT); 
  Timer1.initialize(500000);  // 150 ms
  Timer1.attachInterrupt(blinkingLed);

  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  pinMode(PROX_SENSOR_UP, INPUT);
  pinMode(PROX_SENSOR_DOWN, INPUT);
  pinMode(PROX_SENSOR_LEFT, INPUT);
  pinMode(PROX_SENSOR_RIGHT, INPUT);

  horizontalStepper.setMaxSpeed(horizontalMaxSpeed);
  horizontalStepper.setAcceleration(horizontalAcceleration);

  verticalStepper.setMaxSpeed(verticalMaxSpeed);
  verticalStepper.setAcceleration(verticalAcceleration);
  
  pinMode(GRIPPER_PIN, OUTPUT);
  pinMode(PISTON_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_UP), sensorUp_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_DOWN), sensorDown_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_LEFT), sensorLeft_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PROX_SENSOR_RIGHT), sensorRight_ISR, FALLING);
  delay(1000);
  calibrateLimits();
  Serial.println("\nSystem initialized. Bring an RFID card close...");
  referenceMillis = millis();
}

void loop() {
  
  currentMillis = millis();  
  if (isMoveComplete){
    // continue here
    isActionComplete = true;
  }
  if ((currentMillis - referenceMillis >= TIMESTEP) && (queueCount > 0) && isActionComplete){
    referenceMillis = currentMillis;
    dequeueState(state);
    horizontalStepper.setSpeed(state.xVel);
    verticalStepper.setSpeed(state.yVel);
    moveJ(state.xCoord,state.yCoord);
    isActionComplete = false;
  }
  updatePosition();

  if (msgQueued.length() > 0) {
    Serial.print(msgQueued);
    msgQueued = "";
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Procesar comandos de movimiento en milímetros
    if (input.indexOf(',') > 0) {
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
      long moveMultiplier = parseCommandValue(input);
      switch (toupper(command)) {
        case 'P': digitalWrite(PISTON_PIN, PistonState = !PistonState); break;
        case 'G': digitalWrite(GRIPPER_PIN, GripperState = !GripperState); break;
        case 'D': moveJ(xPosCurrent - HORIZONTALINCREMENT_mm * moveMultiplier, yPosCurrent); break;
        case 'A': moveJ(xPosCurrent + HORIZONTALINCREMENT_mm * moveMultiplier, yPosCurrent); break;
        case 'S': moveJ(xPosCurrent, yPosCurrent - VERTICALINCREMENT_mm * moveMultiplier); break;
        case 'W': moveJ(xPosCurrent, yPosCurrent + VERTICALINCREMENT_mm * moveMultiplier); break;
        case 'B': pauseMotors(); break;
        case 'H': calibrateLimits(); break;
        case 'O': goToOrigin(); break;
        case 'T': operateGripperAndPiston(1); break;
        case 'F': operateGripperAndPiston(2); break;
        case 'R': sensorReset(); break;
        default : Serial.println("Unknown Command"); break;
      }
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

  if (isUpTriggered || isDownTriggered){
    yPosCurrent = haltVertical(haltAcceleration);
    verticalStepper.setCurrentPosition(yPosCurrent*STEPS_PER_mm_VERTICAL);
  }
  if (isRightTriggered || isLeftTriggered){
    xPosCurrent = haltHorizontal(haltAcceleration);
    horizontalStepper.setCurrentPosition(xPosCurrent*STEPS_PER_mm_HORIZONTAL);
  }

  if (isMoveComplete 
      && (xPosCurrent*STEPS_PER_mm_HORIZONTAL != horizontalStepper.currentPosition()
      || yPosCurrent*STEPS_PER_mm_VERTICAL != verticalStepper.currentPosition())) {
    horizontalStepper.setCurrentPosition(xPosCurrent*STEPS_PER_mm_HORIZONTAL);
    verticalStepper.setCurrentPosition(yPosCurrent*STEPS_PER_mm_VERTICAL);
  }
  
}

void moveJ(long xPos, long yPos) {
  isMoveComplete = false;
  
  xPos = constrain(xPos, xLowerLimit, xUpperLimit);
  yPos = constrain(yPos, yLowerLimit, yUpperLimit);

  // Move to the specified position
  if (!(isRightTriggered || isLeftTriggered)){
    horizontalStepper.moveTo(xPos * STEPS_PER_mm_HORIZONTAL);
    xPosCurrent = xPos;
  }
  if (!(isUpTriggered || isDownTriggered)){
    verticalStepper.moveTo(yPos * STEPS_PER_mm_VERTICAL);
    yPosCurrent = yPos;
  }
  
}

void pauseMotors(){
  isPaused = !isPaused;

  if (isPaused) {
    Serial.println("Motors PAUSED");
    xPosTemp = xPosCurrent;
    yPosTemp = yPosCurrent;
    xPosCurrent = haltHorizontal(haltAcceleration);
    yPosCurrent = haltVertical(haltAcceleration);
    stateCurrentPosition();
    moveJ(xPosTemp,yPosTemp);
  }
  else {
    Serial.println("Motors RESUMED");
  }
}

void updatePosition(){
  isMoving = isMovementAllowed();

  if ((!isMoveComplete) & (!isPaused)) {
    while (isMoving){
      horizontalStepper.run();
      verticalStepper.run();
      isMoving = isMovementAllowed();
    }

    if (!isPaused){
      isMoveComplete = true;
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
  moveJ(0, 121.875);
}

void calibrateLimits(){
  Serial.println("CALIBRATION IN PROGRESS");
  while(!isDownTriggered){
    verticalStepper.setSpeed(-verticalLimitTestSpeed);
    verticalStepper.runSpeed();
  }
  sensorReset();
  verticalStepper.setCurrentPosition(0);

  while(!isLeftTriggered){
    horizontalStepper.setSpeed(horizontalLimitTestSpeed);
    horizontalStepper.runSpeed();
  }
  sensorReset();
  horizontalStepper.setCurrentPosition(0);
  delay(1000);
  moveJ(5,5);
  xPosCurrent = 0;
  yPosCurrent = 0;
  
  msgQueued = "CALIBRATION COMPLETE\n";
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

void blinkingLed() {
  static bool ledState = LOW; // Estado del LED (LOW = apagado, HIGH = encendido)
  ledState = !ledState; // Alternar el estado del LED
  digitalWrite(LED_PIN, ledState); // Escribir el nuevo estado del LED
}

void sensorUp_ISR()     { Serial.println("UP SENSOR PROXIMITY ACTIVATED"); isUpTriggered = true; }
void sensorDown_ISR()   { Serial.println("DOWN SENSOR PROXIMITY ACTIVATED"); isDownTriggered = true; }
void sensorLeft_ISR()   { Serial.println("LEFT SENSOR PROXIMITY ACTIVATED"); isLeftTriggered = true; }
void sensorRight_ISR()  { Serial.println("RIGHT SENSOR PROXIMITY ACTIVATED"); isRightTriggered = true; }
void sensorReset()      { isUpTriggered = isDownTriggered = isLeftTriggered = isRightTriggered = false; }

long haltHorizontal(float haltAcceleration){
  horizontalStepper.setAcceleration(haltAcceleration);
  horizontalStepper.stop();
  horizontalStepper.setAcceleration(horizontalAcceleration);
  xPosCurrent = horizontalStepper.currentPosition()/STEPS_PER_mm_HORIZONTAL;
  return xPosCurrent;
}
long haltVertical(float haltAcceleration){
  verticalStepper.setAcceleration(haltAcceleration);
  verticalStepper.stop();
  verticalStepper.setAcceleration(verticalAcceleration);
  yPosCurrent = verticalStepper.currentPosition()/STEPS_PER_mm_VERTICAL;
  return yPosCurrent;
}

void stateCurrentPosition(){
  Serial.println("X: " + String(xPosCurrent) + ", Y: " + String(yPosCurrent));
}

long parseCommandValue(const String& input) {
  return (input.length() <= 1) ? 1 : input.substring(1).toFloat();
}

bool isMovementAllowed(){
  return (horizontalStepper.distanceToGo() != 0 || verticalStepper.distanceToGo() != 0) 
              && (Serial.available() <= 0)
              && (!(isUpTriggered || isDownTriggered))
              && (!(isRightTriggered || isLeftTriggered));
}
