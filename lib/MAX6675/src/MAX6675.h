#ifndef ADAFRUIT_MAX6675_H
#define ADAFRUIT_MAX6675_H

#include "Arduino.h"

/**************************************************************************/
/*!  @brief  Class for communicating with thermocouple sensor*/
/**************************************************************************/
class MAX6675 {
public:
  MAX6675(int8_t SCLK, int8_t CS, int8_t MISO);

  float readCelsius(void);

private:
  int8_t sclk, miso, cs;
  uint8_t spiread(void);
};

#endif