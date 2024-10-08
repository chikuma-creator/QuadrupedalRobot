/************************************************************************************************
 * RobotServo.cpp - Library for PWM Servo Moter Controll
 * Created by Chikuma, 2024/02/04
 * Copylight https://chikuma-creative.com/
 ************************************************************************************************/

#include "RobotServo.h"

// Predefined servo motor list
ServoProperties AvailableServos[] = {
  {ST_MG90S,     "MG90S",       500, 1500, 2500, 20.0f, false},
  {ST_MG90D,     "MG90D",       500, 1500, 2500, 20.0f, true },
  {ST_MG996R,    "MG996R",      500, 1500, 2500, 10.0f, false},
  {ST_MG996R_CR, "MG996R",      500, 1500, 2500, 10.0f, true }
};

// RobotServo Class Method
RobotServo::RobotServo()
  : m_pwm(NULL) {}

RobotServo::RobotServo(Adafruit_PWMServoDriver *pwm)
  : m_pwm(pwm), m_servo_pin(0) {}

RobotServo::RobotServo(Adafruit_PWMServoDriver *pwm, uint8_t pin)
  : m_pwm(pwm), m_servo_pin(pin) {}

RobotServo::~RobotServo() {}

void RobotServo::setServoType(uint8_t pin, uint16_t servo_type) {
  boolean available = false;
  m_servo_pin   = pin;
  m_servo_type  = servo_type;
  for (int i=0; i<(sizeof(AvailableServos) / sizeof(AvailableServos[0])); i++) {
    if (m_servo_type == AvailableServos[i].SERVO_TYPE) {
      m_servo_props = &AvailableServos[i];
      available = true;
      break;
    }
  }
  if (available) {
    setNeutralPosition(m_servo_props->MS_MID);
    setPositionRange(m_servo_props->MS_MIN, m_servo_props->MS_MAX, m_servo_props->MS_MIN);
  }
}

void RobotServo::setNeutralPosition(uint16_t npos) {
  m_neutral_pos   = npos;
  m_goal_pos      = npos;
  m_start_pos     = npos;
  m_current_pos   = npos;
  m_speed         = 0;
  m_prev_speed    = 0;
  m_pos           = (float)m_current_pos;
  m_dir_rotation  = 0;
}

void RobotServo::setPositionRange(uint16_t min_pos, uint16_t max_pos, uint16_t mid_pos) {
  m_ms_min      = min_pos;
  m_ms_max      = max_pos;
  if (mid_pos == 0) {
    m_ms_mid    = (m_ms_min + m_ms_max) / 2;
  } else {
    m_ms_mid    = mid_pos;
  }
}

void RobotServo::moveNeutralPosition() {
  setNeutralPosition(m_neutral_pos);
  if (m_pwm) {
    m_pwm->writeMicroseconds(m_servo_pin, m_neutral_pos);
  }
}

void RobotServo::setGoal(uint16_t goal_pos, float vel_coef, float initial_speed, uint16_t move_count) {
  m_start_pos = m_current_pos;
  if (goal_pos <= m_ms_min) {
    m_goal_pos = m_ms_min;
  } else if (goal_pos >= m_ms_max) {
    m_goal_pos = m_ms_max;
  } else {
    m_goal_pos = goal_pos;
  }
  // cluclate move count
  uint16_t move_dist = abs((int)m_goal_pos - (int)m_start_pos);
  uint16_t unit_dist = DEF_UNIT_DIST;
  if (vel_coef == 1.0 && initial_speed != 0.0) {
    unit_dist = uint16_t(initial_speed);
  }
  if (move_count == 0) {
    m_move_count = uint16_t(move_dist / unit_dist);
  } else {
    m_move_count = move_count;
  }
  if (initial_speed == 0.0 && vel_coef < 1.0) {
    m_speed = float((double(1.0) - double(vel_coef)) / ((double)1.0 - pow(double(vel_coef), double(m_move_count))) * double(move_dist));
  } else if (initial_speed == 0.0 && vel_coef >= 1.0) {
    m_speed = m_servo_props->INITIAL_SPEED;
  } else {
    m_speed = initial_speed;
  }
  if (m_goal_pos < m_start_pos) {
    m_speed = m_speed * -1.0;
  } else if (m_goal_pos > m_start_pos) {
    m_speed = m_speed;
  } else {
    m_speed = 0;
  }
  m_prev_speed = 0;
  Serial.print(" Start: "); Serial.print((int)m_start_pos);
  Serial.print(" Goal: "); Serial.print((int)m_goal_pos);
  Serial.print(" Move Count: "); Serial.print((int)m_move_count);
  Serial.print(" Distance: "); Serial.print((int)move_dist);
  Serial.print(" Initial speed: "); Serial.println((double)m_speed);
}

void RobotServo::setReverse(float vel_coef) {
  setGoal(m_start_pos, vel_coef);
}

boolean RobotServo::isReachGoal() {
  if (m_current_pos == m_goal_pos) return true;
  return false;
}

void RobotServo::run(float vel_coef) {
  if (m_prev_speed != 0) {
    m_speed = m_prev_speed * vel_coef;
//  if (m_speed < 0 && m_speed > -1) m_speed = -1.0;
//  if (m_speed > 0 && m_speed <  1) m_speed =  1.0;
  }
  m_pos += m_speed;
  m_current_pos = (uint16_t)m_pos;
  m_prev_speed = m_speed;
  if (m_speed < 0.0 && m_current_pos <= m_goal_pos) {
    m_current_pos = m_goal_pos;
    m_pos = (float)m_current_pos;
  }
  else if (m_speed > 0.0 && m_current_pos >= m_goal_pos) {
    m_current_pos = m_goal_pos;
    m_pos = (float)m_current_pos;
  }
  m_pwm->writeMicroseconds(m_servo_pin, m_current_pos);
}

void RobotServo::stop() {
  m_speed = 0;
  m_prev_speed = 0;
  m_pwm->writeMicroseconds(m_servo_pin, m_current_pos);
}

uint16_t RobotServo::getGoalPosition() {
  return m_goal_pos;
}

uint16_t RobotServo::getCurrentPosition() {
  return m_current_pos;
}

float RobotServo::getSpeed() {
  return m_speed;
}

// RobotServoController Class Method
RobotServoController::RobotServoController()
  : m_pwm_addr(PCA9685_I2C_ADDRESS)
  {
    for(int i=0; i<PWM_CHANNELS; i++) {
      m_servos[i] = NULL;
    }
  }

RobotServoController::RobotServoController(const uint8_t addr)
  : m_pwm_addr(addr)
{
    for(int i=0; i<PWM_CHANNELS; i++) {
      m_servos[i] = NULL;
    }
}

RobotServoController::~RobotServoController() {
  if (m_pwm) delete m_pwm;
  for (int i=0; i<PWM_CHANNELS; i++) {
    if (m_servos[i]) {
      delete m_servos[i];
      m_servos[i] = NULL;
    }
  }
}

void RobotServoController::initPWMServoDriver(float freq) {
  if (m_pwm) delete m_pwm;
  m_pwm = new Adafruit_PWMServoDriver(m_pwm_addr);
  m_pwm->begin();
  m_pwm->setPWMFreq(freq);
}

void RobotServoController::attach(uint8_t pin, uint16_t servo_type) {
  // This method only works if the PIN number is between 0 and 15.
  if (pin >= 0 && pin < 16) {
    detach(pin);
    m_servos[pin] = new RobotServo(m_pwm);
    m_servos[pin]->setServoType(pin, servo_type);
  }
}


void RobotServoController::attach(uint8_t pin, uint16_t servo_type, uint16_t npos) {
  attach(pin, servo_type);
  if (m_servos[pin]) {
    m_servos[pin]->setNeutralPosition(npos);
  }
}

void RobotServoController::detach(uint8_t pin) {
  // This method only works if the PIN number is between 0 and 15.")
  if (pin >= 0 && pin < 16) {
    if (m_servos[pin]) {
      delete m_servos[pin];
      m_servos[pin] = NULL;
    }
  }
}

void RobotServoController::setGoal(uint8_t pin, uint16_t goal_pos, float initial_speed, float vel_coef) {
  float _vel_coef = vel_coef;
  if (_vel_coef == 0.0) {
    _vel_coef = m_vel_coef;
  }
  if (m_servos[pin]) {
    m_servos[pin]->setGoal(goal_pos, _vel_coef, initial_speed);
  }
}

// Returns true if all servo motors have reached the goal
boolean RobotServoController::isReachGoal() {
  for (int i=0; i<PWM_CHANNELS; i++) {
    if (m_servos[i]) {
      if (!m_servos[i]->isReachGoal()) {
        return false;
      }
    }
  }
  return true;
}

void RobotServoController::run(float vel_coef) {
  for (int i=0; i<PWM_CHANNELS; i++) {
    float _vel_coef = vel_coef;
    if (_vel_coef == 0.0) {
      _vel_coef = m_vel_coef;
    }
    if (m_servos[i]) {
      m_servos[i]->run(_vel_coef);
    }
  }
}

RobotServo *RobotServoController::getRobotServo(uint8_t pin) {
  if (pin >= 0 && pin < 16) {
    if (m_servos[pin]) {
      return m_servos[pin];
    }
  }
  return NULL;
}
