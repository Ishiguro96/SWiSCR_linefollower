#include "motorDriver.h"


void MotorDriver::_driveMotors(const int16_t power[2]) {
  // Set directions of motors
  if(power[0] < 0) HAL_GPIO_WritePin(GPIOB, MOTORA_ENB, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(GPIOB, MOTORA_ENB, GPIO_PIN_RESET);

  if(power[1] < 0) HAL_GPIO_WritePin(GPIOB, MOTORB_ENB, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(GPIOB, MOTORB_ENB, GPIO_PIN_RESET);

  // Set motors powers
  _setMotorSpeed(power[0], Motors::MotorA);
  _setMotorSpeed(power[1], Motors::MotorB);
}


void MotorDriver::_setMotorSpeed(uint16_t speed, Motors motor) {
  if (speed > 100) speed = 100;

  // Calculate new PWM register max value
  uint16_t newSpeed = speed * m_PWMfactor;
  if(newSpeed > MAX_PWM_REGISTER) newSpeed = MAX_PWM_REGISTER;

  switch(motor) {
  case Motors::MotorA:
    __HAL_TIM_SET_COMPARE(&m_htim, MOTORA_PWM_CHANNEL, newSpeed);
    break;
  case Motors::MotorB:
    __HAL_TIM_SET_COMPARE(&m_htim, MOTORB_PWM_CHANNEL, newSpeed);
    break;
  }
}


MotorDriver::MotorDriver(TIM_HandleTypeDef& htim) : m_htim(htim) {
  m_actualLeftMotorPercentage = 0;
  m_actualRightMotorPercentage = 0;

  // Start PWM for motors
  HAL_TIM_PWM_Start(&m_htim, MOTORA_PWM_CHANNEL);
  HAL_TIM_PWM_Start(&m_htim, MOTORB_PWM_CHANNEL);

  // Set PWM output to 0%
  __HAL_TIM_SET_COMPARE(&m_htim, MOTORA_PWM_CHANNEL, 0);
  __HAL_TIM_SET_COMPARE(&m_htim, MOTORB_PWM_CHANNEL, 0);
}


uint8_t MotorDriver::getLeftMotorPercentage() {
  return m_actualLeftMotorPercentage;
}


uint8_t MotorDriver::getRightMotorPercentage() {
  return m_actualRightMotorPercentage;
}

/*
void MotorDriver::driveMotor(uint8_t leftMotorPercentage, uint8_t rightMotorPercentage) {
  // Drive PWM with given percentage
  m_actualLeftMotorPercentage = leftMotorPercentage;
  m_actualRightMotorPercentage = rightMotorPercentage;
}
*/

void MotorDriver::driveMotor(const MotorBehavior behaviors) {
  switch(behaviors) {
  case MotorBehavior::MoveForward:
    _driveMotors(forwardPower);
    break;
  case MotorBehavior::MoveBackward:
    _driveMotors(backwardPower);
    break;

  case MotorBehavior::TurnLeftGently:
    _driveMotors(leftGentlyPower);
    break;
  case MotorBehavior::TurnRightGently:
    _driveMotors(rightGentlyPower);
    break;

  case MotorBehavior::TurnLeftSharply:
    _driveMotors(leftSharplyPower);
    break;
  case MotorBehavior::TurnRightSharply:
    _driveMotors(rightSharplyPower);
    break;

  case MotorBehavior::Stop:
    _driveMotors(stopPower);
    break;
  }
}
