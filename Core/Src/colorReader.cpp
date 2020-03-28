#include "colorReader.h"

uint16_t ColorReader::adc_data[NUMBER_OF_OPTOCOUPLERS];

ColorReader::ColorReader(uint16_t threshold, bool invertedColors) {
  m_threshold = threshold;
  m_invertedColors = invertedColors;
}


ColorsReadStruct ColorReader::getColors() {
  // Read colors from ADC - maybe use DMA and continuous reading over all optocouplers
  // Done - color reading is made by direct DMA interrupt with ADCs
  ColorsReadStruct _colorsReadStruct;

  for (int i = 0; i < NUMBER_OF_OPTOCOUPLERS; i++) {
    // Read colors of i-th optocoupler
    if (ColorReader::adc_data[i] >= m_threshold) {
      if(!m_invertedColors)
        _colorsReadStruct.colors[i] = ColorsEnum::Ambient;
      else
        _colorsReadStruct.colors[i] = ColorsEnum::Line;
    }
    else {
      if(!m_invertedColors)
        _colorsReadStruct.colors[i] = ColorsEnum::Line;
      else
        _colorsReadStruct.colors[i] = ColorsEnum::Ambient;
    }
  }

  return _colorsReadStruct;
}


uint16_t* ColorReader::getVariableForMeasurements() {
  return ColorReader::adc_data;
}
