#ifndef _LINE_FOLLOWER_
#define _LINE_FOLLOWER_

#include "motorDriver.h"
#include "colorReader.h"

class LineFollower {
public:
  LineFollower(TIM_HandleTypeDef& htim, uint16_t threshold, bool invertedColors = false);
  void run();

  //Debug functions
  ColorsReadStruct& __getColors();

  uint16_t* getVariableForMeasurements();
private:
  ColorReader m_colorReader;
  MotorDriver m_motorDriver;

  ColorsReadStruct m_colorsReadStruct;
};

#endif
