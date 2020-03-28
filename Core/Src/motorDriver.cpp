#include "motorDriver.h"


MotorDriver::MotorDriver() {
  m_actualLeftMotorPercentage = 0;
  m_actualRightMotorPercentage = 0;
}


void MotorDriver::driveMotor(uint8_t leftMotorPercentage, uint8_t rightMotorPercentage) {
  // Drive PWM with given percentage
  m_actualLeftMotorPercentage = leftMotorPercentage;
  m_actualRightMotorPercentage = rightMotorPercentage;
}


uint8_t MotorDriver::getLeftMotorPercentage() {
  return m_actualLeftMotorPercentage;
}


uint8_t MotorDriver::getRightMotorPercentage() {
  return m_actualRightMotorPercentage;
}
