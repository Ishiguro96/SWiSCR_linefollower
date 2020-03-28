#ifndef _COLOR_READER_
#define _COLOR_READER_

#include <stdint.h>

#define NUMBER_OF_OPTOCOUPLERS 4

enum class ColorsEnum {
  Line, Ambient
};

// Structure to hold read colors - read from left to right (top view)
struct ColorsReadStruct {
  ColorsEnum colors[NUMBER_OF_OPTOCOUPLERS];
};

class ColorReader {
public:
  ColorReader(uint16_t threshold, bool invertedColors = false);
  ColorsReadStruct getColors();

  uint16_t* getVariableForMeasurements();

private:
  uint16_t m_threshold;
  bool m_invertedColors;

  static uint16_t adc_data[NUMBER_OF_OPTOCOUPLERS];
};

#endif
