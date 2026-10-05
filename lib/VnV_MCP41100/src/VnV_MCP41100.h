#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <VnV_PCF8574.h>

// MCP41100 is a single 8-bit SPI digital potentiometer.
class VnV_MCP41100 {
public:
  VnV_MCP41100(SPIClass &spi, VnV_PCF8574 &chipSelectPcf,
               uint8_t chipSelectPin);

  bool begin(uint8_t initialWiper = 0x00);
  bool setWiper(uint8_t value);
  uint8_t wiper() const { return _wiper; }

private:
  SPIClass *_spi;
  VnV_PCF8574 *_chipSelectPcf;
  uint8_t _chipSelectPin;
  uint8_t _wiper;
  bool _initialized;
};
