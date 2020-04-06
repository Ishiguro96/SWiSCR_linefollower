#include "lineFollower.h"


LineFollower::LineFollower(TIM_HandleTypeDef& htim, uint16_t threshold, bool invertedColors)
  : m_colorReader(threshold, invertedColors),
    m_motorDriver(htim)
{
  m_prevState = 0;
  m_currentState = 0;
}


void LineFollower::_prepareStruct() {
  m_colors = 0;
  for(uint8_t i = 0; i < NUMBER_OF_OPTOCOUPLERS; i++) {
    m_colors |= (((uint8_t)m_colorsReadStruct.colors[i]) << i);
  }
}


void LineFollower::run() {
  //[TODO]
  /*
   * 1. Read all colors
   * 2. Check which situation we have with algorithm
   * 3. Drive motors
   *
   * No infinite loop here - we have in main
   */

  // 1. Read all colors
  m_colorsReadStruct = m_colorReader.getColors();

  _prepareStruct();

  m_prevState = m_currentState;

  switch(m_colors) {
  case 0b0110:
    m_motorDriver.driveMotor(MotorBehavior::MoveForward);
    break;

  case 0b0100:
    m_motorDriver.driveMotor(MotorBehavior::TurnRightGently);
    m_currentState = m_colors;
    break;

  case 0b0010:
    m_motorDriver.driveMotor(MotorBehavior::TurnLeftGently);
    m_currentState = m_colors;
    break;

  case 0b1000:
  case 0b1010:
  case 0b1100:
  case 0b1110:
    m_motorDriver.driveMotor(MotorBehavior::TurnRightSharply);
    m_currentState = m_colors;
    break;

  case 0b0001:
  case 0b0011:
  case 0b0101:
  case 0b0111:
    m_motorDriver.driveMotor(MotorBehavior::TurnLeftSharply);
    m_currentState = m_colors;
    break;

  case 0b0000:
    if(m_prevState == 0b0100 || m_colors & 0b1000) {
      m_motorDriver.driveMotor(MotorBehavior::TurnRightSharply);
    }
    else if(m_prevState == 0b0010 || m_colors & 0b0001) {
      m_motorDriver.driveMotor(MotorBehavior::TurnLeftSharply);
    }
    else {
      m_motorDriver.driveMotor(MotorBehavior::Stop);
    }
    break;

  default:
    m_motorDriver.driveMotor(MotorBehavior::Stop);
    break;
  }

}


uint16_t* LineFollower::getVariableForMeasurements() {
  return m_colorReader.getVariableForMeasurements();
}


