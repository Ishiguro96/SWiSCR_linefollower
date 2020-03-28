#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <stdint.h>

// Behaviors needed by motors in LineFollower
enum class MotorBehavior {
  MoveForward,
  MoveBackward,
  TurnLeftGently,
  TurnRightGently,
  TurnLeftSharply,
  TurnRightSharply,
};

class MotorDriver {
private:
  static constexpr int16_t forwardPower[2] = {100, 100};
  static constexpr int16_t backwardPower[2] = {-100, -100};
  static constexpr int16_t leftGentlyPower[2] = {100, 40};
  static constexpr int16_t rightGentlyPower[2] = {40, 100};
  static constexpr int16_t leftSharplyPower[2] = {100, -80};
  static constexpr int16_t rightSharplyPower[2] = {-80, 100};

  uint8_t m_actualLeftMotorPercentage;
  uint8_t m_actualRightMotorPercentage;

  // Max PWM divided by 100 [%]
  uint16_t m_PWMfactor = 1000 / 100;
public:
  MotorDriver();
  void driveMotor(uint8_t leftMotorPercentage, uint8_t rightMotorPercentage);
  void driveMotor(MotorBehavior behaviors);
  uint8_t getLeftMotorPercentage();
  uint8_t getRightMotorPercentage();
};

#endif
