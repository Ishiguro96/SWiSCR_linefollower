#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <stdint.h>

class MotorDriver {
private:
  uint8_t m_actualLeftMotorPercentage;
  uint8_t m_actualRightMotorPercentage;

  // Max PWM divided by 100 [%]
  uint16_t m_PWMfactor = 1000 / 100;
public:
  MotorDriver();
  void driveMotor(uint8_t leftMotorPercentage, uint8_t rightMotorPercentage);
  uint8_t getLeftMotorPercentage();
  uint8_t getRightMotorPercentage();
};

#endif
