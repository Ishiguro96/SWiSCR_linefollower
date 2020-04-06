#ifndef _LINE_FOLLOWER_
#define _LINE_FOLLOWER_

#include "motorDriver.h"
#include "colorReader.h"

class LineFollower {
public:
  LineFollower(TIM_HandleTypeDef& htim, uint16_t threshold, bool invertedColors = false);
  void run();

  uint16_t* getVariableForMeasurements();
private:
  void _prepareStruct();

  ColorReader m_colorReader;
  MotorDriver m_motorDriver;

  ColorsReadStruct m_colorsReadStruct;
  uint8_t m_colors;

  uint8_t m_prevState;
  uint8_t m_currentState;
};

#endif
