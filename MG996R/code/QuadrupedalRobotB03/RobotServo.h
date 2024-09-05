/************************************************************************************************
 * RobotServo.h - Library for PWM Servo Moter Controll
 * Created by Chikuma, 2024/02/04
 * Copylight https://chikuma-creative.com/
 ************************************************************************************************/

#ifndef RobotServo_h
#define RobotServo_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo Types
#define ST_MG90S        1002      /**< TowerPro MG90S */
#define ST_MG90D        1003      /**< TowerPro MG90D 360 degree */
#define ST_MG996R       1010      /**< TowerPro MG996R */
#define ST_MG996R_CR    1011      /**< TowerPro MG996R 360 degree */

#define PWM_CHANNELS    16        /**< PWM Servo Driver Channel Count */
#define DEF_VEL_COEF    0.992     /**< Default velocity coefficient (%) */
#define DEF_UNIT_DIST   10        /**< Distance traveled in one movement */

const bool debug        = false;   /**< Debug print to display (Serial.begin(<baud>) is required) */

// Servo properties
struct ServoProperties {
  uint16_t    SERVO_TYPE;         // Servo motor model number. Specify from a constant (e.g. ST_MG996R)
  char*       SERVO_NAME;         // Servo motor model name.
  uint16_t    MS_MIN;             // Minimum value of servo motor (micro sec)
  uint16_t    MS_MID;             // Intermediate value of servo motor (micro sec)
  uint16_t    MS_MAX;             // Maximum value of servo motor (micro sec)
  float       INITIAL_SPEED;      // Initial Speed
  boolean     CONTINUOUS;         // Continuous rotation.
};

// RobotServo Class.
class RobotServo {
  public:
    RobotServo();
    RobotServo(Adafruit_PWMServoDriver *pwm);
    RobotServo(Adafruit_PWMServoDriver *pwm, uint8_t pin);
    ~RobotServo();
    void setPWMServoDriver(Adafruit_PWMServoDriver *pwm);
    void setServoType(uint8_t pin, uint16_t servo_type);
    void setNeutralPosition(uint16_t npos);
    void setPositionRange(uint16_t min_pos, uint16_t max_pos, uint16_t mid_pos = 0);
    void moveNeutralPosition();
    void setGoal(uint16_t goal_pos, float vel_coef, float initial_speed = 0.0, uint16_t move_count = 0);
    void setReverse(float vel_coef = 1.0);
    void run(float vel_coef = 1.0);
    void stop();
    boolean isReachGoal();
    uint16_t getGoalPosition();
    uint16_t getCurrentPosition();
    float getSpeed();
  private:
    uint16_t          m_servo_type;
    ServoProperties*  m_servo_props = NULL;
    uint8_t           m_servo_pin;
    Adafruit_PWMServoDriver*  m_pwm = NULL;
    uint16_t        m_ms_min;
    uint16_t        m_ms_mid;
    uint16_t        m_ms_max;
    uint16_t        m_goal_pos;
    uint16_t        m_start_pos;
    uint16_t        m_neutral_pos;
    uint16_t        m_current_pos;
    float           m_speed;                      // speed
    float           m_prev_speed;                 // previous speed
    float           m_pos;                        // current position (float)
    int8_t          m_dir_rotation;               // +1 clockwise / -1 counterclockwise / 0 stop
    uint16_t        m_move_count;                 // move count
};

// RobotServoController
class RobotServoController {
  public:
    RobotServoController();
    RobotServoController(const uint8_t addr);
    ~RobotServoController();
    void initPWMServoDriver(float freq);
    void attach(uint8_t pin, uint16_t servo_type);
    void attach(uint8_t pin, uint16_t servo_type, uint16_t npos);
    void detach(uint8_t pin);
    void setGoal(uint8_t pin, uint16_t goal_pos, float initial_speed = 0.0, float vel_coef = 0.0);
    boolean isReachGoal();
    void run(float vel_coef = 0.0);
    RobotServo *getRobotServo(uint8_t pin);
  private:
    Adafruit_PWMServoDriver *m_pwm = NULL;    // PCA9685 PWM Controller board driver.
    uint8_t       m_pwm_addr;                 // PWM Controller Address
    RobotServo*   m_servos[PWM_CHANNELS];     // Servo moter list
    float         m_interval = 5.0;           // Servo position control call interval (ms)
    float         m_vel_coef = DEF_VEL_COEF;  // velocity coefficient (%)
};

#endif
