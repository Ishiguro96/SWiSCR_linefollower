#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <cstdint>
#include <cstdlib>
#include "stm32f4xx_hal.h"

// Defines for PWM and motors
#define MOTORA_PWM_CHANNEL TIM_CHANNEL_1
#define MOTORB_PWM_CHANNEL TIM_CHANNEL_2
#define MOTORA_ENB GPIO_PIN_4
#define MOTORB_ENB GPIO_PIN_5
#define MAX_PWM_REGISTER 999

// Behaviors needed by motors in LineFollower
enum class MotorBehavior {
  MoveForward,
  MoveBackward,
  TurnLeftGently,
  TurnRightGently,
  TurnLeftSharply,
  TurnRightSharply,
};

enum class Motors {
  MotorA,
  MotorB
};

class MotorDriver {
private:
  static constexpr int16_t forwardPower[2] = {80, 80};
  static constexpr int16_t backwardPower[2] = {-100, -100};
  static constexpr int16_t leftGentlyPower[2] = {80, 40};
  static constexpr int16_t rightGentlyPower[2] = {40, 80};
  static constexpr int16_t leftSharplyPower[2] = {100, -80};
  static constexpr int16_t rightSharplyPower[2] = {-80, 100};

  uint8_t m_actualLeftMotorPercentage;
  uint8_t m_actualRightMotorPercentage;

  // Max PWM divided by 100 [%]
  uint16_t m_PWMfactor = 1000 / 100;

  void _driveMotors(const int16_t power[2]);
  void _setMotorSpeed(uint16_t speed, Motors motor);

  TIM_HandleTypeDef& m_htim;
public:
  MotorDriver(TIM_HandleTypeDef& htim);
  //void driveMotor(uint8_t leftMotorPercentage, uint8_t rightMotorPercentage);
  void driveMotor(const MotorBehavior behaviors);
  uint8_t getLeftMotorPercentage();
  uint8_t getRightMotorPercentage();
};

#endif
