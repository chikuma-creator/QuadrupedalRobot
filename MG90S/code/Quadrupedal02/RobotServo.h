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
#define ST_SG90         1000      /**< TowerPro SG90 */
#define ST_SG90_CR      1001      /**< TowerPro SG90 360 degree */
#define ST_MG90S        1002      /**< TowerPro MG90S */
#define ST_MG90D        1003      /**< TowerPro MG90D 360 degree */
#define ST_MG92B        1004      /**< TowerPro MG92B */
#define ST_MG92B_CR     1005      /**< TowerPro MG92B 360 degree */
#define ST_MG996R       1010      /**< TowerPro MG996R */
#define ST_MG996R_CR    1011      /**< TowerPro MG996R 360 degree */
#define ST_MG995        1012      /**< TowerPro MG995 */
#define ST_MG945        1013      /**< TowerPro MG945 */
#define ST_SG5010       1012      /**< TowerPro SG5010 */
#define ST_MG938        1013      /**< TowerPro MG938 */
#define ST_MG946R       1014      /**< TowerPro MG946R */
#define ST_MG959        1020      /**< TowerPro MG959 */
#define ST_MG968        1021      /**< TowerPro MG968 */
#define ST_MG958        1022      /**< TowerPro MG958 */
#define ST_PID1145BW    1145      /**< TowerPro PID-1145BW */
#define ST_HS805BB      2000      /**< Hitec HS-805BB */
#define ST_FS5106R      3001      /**< FEETECH FS5106R 360 degree */

#define PWM_CHANNELS    16        /**< PWM Servo Driver Channel Count */

// Servo properties
struct ServoProperties {
  uint16_t    SERVO_TYPE;         // Servo motor model number. Specify from a constant (e.g. ST_MG996R)
  char*       SERVO_NAME;         // Servo motor model name.
  uint16_t    MS_MIN;             // Minimum value of servo motor (micro sec)
  uint16_t    MS_MID;             // Intermediate value of servo motor (micro sec)
  uint16_t    MS_MAX;             // Maximum value of servo motor (micro sec)
  int16_t     DEGREE_MIN;         // Minimum operating angle (degrees)
  int16_t     DEGREE_MAX;         // Maximum operating angle (degrees)
  boolean     CONTINUOUS;         // Continuous rotation.
};

// RobotServo Class.
class RobotServo {
  public:
    RobotServo();
    RobotServo(const Adafruit_PWMServoDriver *pwm, uint8_t pin = 0);
    ~RobotServo();
    void setPWMServoDriver(const Adafruit_PWMServoDriver *pwm);
    void setServoType(uint8_t pin, uint16_t servo_type);
    void setNeutralPosition(uint16_t npos);
    void setPositionRange(uint16_t min_pos, uint16_t max_pos, uint16_t mid_pos = 0);
    void setAngleRange(int16_t min_degree, int16_t max_degree);
    void setPartName(const char* part_name);
    void moveNeutralPosition();
    void setGoal(uint16_t goal_pos);
    void setReverse();
    void run(float vel_coef = 1.0);
    void stop();
    boolean isReachGoal();
    uint16_t getGoalPosition();
    uint16_t getCurrentPosition();
    float getSpeed();
  private:
    uint16_t        m_servo_type;
    char*           m_servo_name = NULL;
    char*           m_part_name = NULL;
    ServoProperties m_servo_props;
    uint16_t        m_ms_min;
    uint16_t        m_ms_mid;
    uint16_t        m_ms_max;
    uint16_t        m_degree_min;
    uint16_t        m_degree_max;
    uint8_t         m_servo_pin;
    Adafruit_PWMServoDriver   *m_pwm = NULL;
    uint16_t        m_goal_pos;
    uint16_t        m_start_pos;
    uint16_t        m_neutral_pos;
    uint16_t        m_current_pos;
    float           m_initial_speed = 20.0;       // Initial speed
    float           m_speed;                      // speed
    float           m_prev_speed;                 // previous speed
    float           m_pos;                        // current position (float)
    int8_t          m_dir_rotation;               // +1 clockwise / -1 counterclockwise / 0 stop
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
    void setGoal(uint8_t pin, uint16_t goal_pos);
    boolean isReachGoal();
    void run();
    RobotServo *getRobotServo(uint8_t pin);
  private:
    Adafruit_PWMServoDriver *m_pwm = NULL;    // PCA9685 PWM Controller board driver.
    uint8_t       m_pwm_addr;                 // PWM Controller Address
    RobotServo*   m_servos[PWM_CHANNELS];     // Servo moter list
    float         m_interval = 5.0;           // Servo position control call interval (ms)
    float         m_vel_coef = 0.98;          // velocity coefficient (%)
};

#endif
