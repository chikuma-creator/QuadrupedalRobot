#include "RobotServo.h"

#define SERVO_FREQ        50      /* Servo Driver Frequency */
#define MG90S_SERVOMIN    1000
#define MG90S_SERVOMID    1500
#define MG90S_SERVOMAX    2000

const uint8_t   LEG1_SERVO1_PIN   = 10; // 右前足の根本
const uint8_t   LEG1_SERVO2_PIN   = 11; // 右前足の第１関節
const uint8_t   LEG1_SERVO3_PIN   = 12; // 右前足の第２関節
const uint8_t   LEG2_SERVO1_PIN   = 13; // 右後足の根本
const uint8_t   LEG2_SERVO2_PIN   = 14; // 右前後の第１関節
const uint8_t   LEG2_SERVO3_PIN   = 15; // 右前後の第２関節
const uint8_t   LEG3_SERVO1_PIN   =  5; // 左前足の根本
const uint8_t   LEG3_SERVO2_PIN   =  4; // 左前足の第１関節
const uint8_t   LEG3_SERVO3_PIN   =  3; // 左前足の第２関節
const uint8_t   LEG4_SERVO1_PIN   =  2; // 左後足の根本
const uint8_t   LEG4_SERVO2_PIN   =  1; // 左前後の第１関節
const uint8_t   LEG4_SERVO3_PIN   =  0; // 左前後の第２関節

RobotServoController controller = RobotServoController(0x40);

const uint8_t SERVO_PIN[12] = {
  LEG1_SERVO1_PIN, LEG1_SERVO2_PIN, LEG1_SERVO3_PIN,
  LEG2_SERVO1_PIN, LEG2_SERVO2_PIN, LEG2_SERVO3_PIN,
  LEG3_SERVO1_PIN, LEG3_SERVO2_PIN, LEG3_SERVO3_PIN,
  LEG4_SERVO1_PIN, LEG4_SERVO2_PIN, LEG4_SERVO3_PIN
};

// 配列のサイズを取得しておく
size_t sizeX = sizeof(SERVO_PIN) / sizeof(SERVO_PIN[0]);

const uint16_t SERVO_POS[][12] = {
  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}, 
  {1150, 1500, 1500, 1850, 1500, 1500, 1850, 1500, 1500, 1150, 1500, 1500}, 
  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}, 
  {1850, 1500, 1500, 1150, 1500, 1500, 1150, 1500, 1500, 1850, 1500, 1500}, 
  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}, 
  {1500, 1150, 1850, 1500, 1150, 1850, 1500, 1150, 1850, 1500, 1150, 1850},
  {1500,  800, 2200, 1500,  800, 2200, 1500,  800, 2200, 1500,  800, 2200},
  {1500, 1150, 1850, 1500, 1150, 1850, 1500, 1150, 1850, 1500, 1150, 1850},
  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}, 
  {1500, 1850, 1150, 1500, 1850, 1150, 1500, 1850, 1150, 1500, 1850, 1150},
  {1500, 2200,  800, 1500, 2200,  800, 1500, 2200,  800, 1500, 2200,  800},
  {1500, 1850, 1150, 1500, 1850, 1150, 1500, 1850, 1150, 1500, 1850, 1150},
  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500} 
};

size_t sizeY = sizeof(SERVO_POS) / sizeof(SERVO_POS[0]);
size_t goal_count = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Quadrupedal move start.");
  controller.initPWMServoDriver(SERVO_FREQ);
  delay(500);
  Serial.print("Initial servo moter: ");
  Serial.print(sizeX);
  Serial.println(" moters.");
  for (int x=0; x<sizeX; x++) {
    controller.attach(SERVO_PIN[x], ST_MG90S, MG90S_SERVOMID);
  }
  delay(1000);
}

void loop() {
  if (controller.isReachGoal()) {
    if (goal_count == sizeY) {
      Serial.println("Quadrupedal move end.");
      delay(1000);
      exit(0);
    } else {
      for (int x=0; x<sizeX; x++) {
        controller.setGoal(SERVO_PIN[x], SERVO_POS[goal_count][x]);
      }
      delay(500);
      goal_count++;
    }
  } else {
    controller.run();
  }
  delay(4);
}