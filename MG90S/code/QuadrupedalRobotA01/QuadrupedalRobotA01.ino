#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define MG90S_SERVOMIN     700
#define MG90S_SERVOMID    1500
#define MG90S_SERVOMAX    2300

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

const uint8_t SERVO_PIN[][3] = {
  {LEG1_SERVO1_PIN, LEG1_SERVO2_PIN, LEG1_SERVO3_PIN},
  {LEG2_SERVO1_PIN, LEG2_SERVO2_PIN, LEG2_SERVO3_PIN},
  {LEG3_SERVO1_PIN, LEG3_SERVO2_PIN, LEG3_SERVO3_PIN},
  {LEG4_SERVO1_PIN, LEG4_SERVO2_PIN, LEG4_SERVO3_PIN},
};

// 配列のサイズを取得しておく
size_t sizeX = sizeof(SERVO_PIN) / sizeof(SERVO_PIN[0]);
size_t sizeY = sizeof(SERVO_PIN[0]) / sizeof(SERVO_PIN[0][0]);

const uint16_t SERVO_POS[][4][3] = {
  {{1500, 1500, 1500}, {1500, 1500, 1500}, {1500, 1500, 1500,}, {1500, 1500, 1500}}, 
  {{1150, 1500, 1500}, {1850, 1500, 1500}, {1850, 1500, 1500,}, {1150, 1500, 1500}}, 
  {{1500, 1500, 1500}, {1500, 1500, 1500}, {1500, 1500, 1500,}, {1500, 1500, 1500}}, 
  {{1850, 1500, 1500}, {1150, 1500, 1500}, {1150, 1500, 1500,}, {1850, 1500, 1500}}, 
  {{1500, 1500, 1500}, {1500, 1500, 1500}, {1500, 1500, 1500,}, {1500, 1500, 1500}}, 
  {{1500, 1150, 1850}, {1500, 1150, 1850}, {1500, 1150, 1850,}, {1500, 1150, 1850}},
  {{1500,  700, 2300}, {1500,  700, 2300}, {1500,  700, 2300,}, {1500,  700, 2300}},
  {{1500, 1150, 1850}, {1500, 1150, 1850}, {1500, 1150, 1850,}, {1500, 1150, 1850}},
  {{1500, 1500, 1500}, {1500, 1500, 1500}, {1500, 1500, 1500,}, {1500, 1500, 1500}}, 
  {{1500, 1850, 1150}, {1500, 1850, 1150}, {1500, 1850, 1150,}, {1500, 1850, 1150}},
  {{1500, 2300,  700}, {1500, 2300,  700}, {1500, 2300,  700,}, {1500, 2300,  700}},
  {{1500, 1850, 1150}, {1500, 1850, 1150}, {1500, 1850, 1150,}, {1500, 1850, 1150}},
  {{1500, 1500, 1500}, {1500, 1500, 1500}, {1500, 1500, 1500,}, {1500, 1500, 1500}} 
};

size_t sizeZ = sizeof(SERVO_POS) / sizeof(SERVO_POS[0]);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);
  Serial.println("Quadrupedal move start.");
  Serial.print("Motions: "); Serial.println(sizeZ);
  // 初期位置をセット
  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      pwm.writeMicroseconds(SERVO_PIN[x][y], MG90S_SERVOMID);
    }
  }
  delay(700);
}

void loop() {
  for (int z=0; z<sizeZ; z++) {
    Serial.print("{ ");
    for (int x=0; x<sizeX; x++) {
      Serial.print("{ ");
      for (int y=0; y<sizeY; y++) {
        pwm.writeMicroseconds(SERVO_PIN[x][y], SERVO_POS[z][x][y]);
        if (y>0) { Serial.print(", "); }
        Serial.print(SERVO_POS[z][x][y]);
      }
      Serial.print(" }");
    }
    Serial.println(" }");
    delay(500);
  }
  Serial.println("Quadrupedal move end.");
  delay(1000);
  exit(0);
}
