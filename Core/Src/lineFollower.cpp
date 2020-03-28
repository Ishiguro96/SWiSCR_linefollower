#include "lineFollower.h"


LineFollower::LineFollower(uint16_t threshold, bool invertedColors)
  : m_colorReader(threshold, invertedColors),
    m_motorDriver()
{

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


}


uint16_t* LineFollower::getVariableForMeasurements() {
  return m_colorReader.getVariableForMeasurements();
}


// Debug functions
ColorsReadStruct& LineFollower::__getColors() {
  return m_colorsReadStruct;
}
