#include "VnV_MCP41100.h"

namespace {
constexpr uint8_t MCP41100_WRITE_WIPER = 0x11;
constexpr uint32_t MCP41100_SPI_HZ = 1000000;
}

VnV_MCP41100::VnV_MCP41100(SPIClass &spi, VnV_PCF8574 &chipSelectPcf,
                           uint8_t chipSelectPin)
    : _spi(&spi), _chipSelectPcf(&chipSelectPcf),
      _chipSelectPin(chipSelectPin), _wiper(0), _initialized(false) {}

bool VnV_MCP41100::begin(uint8_t initialWiper) {
  if (!_chipSelectPcf->pinMode(_chipSelectPin, OUTPUT) ||
      !_chipSelectPcf->digitalWrite(_chipSelectPin, HIGH)) {
    return false;
  }
  _initialized = true;
  return setWiper(initialWiper);
}

bool VnV_MCP41100::setWiper(uint8_t value) {
  if (!_initialized) {
    return false;
  }

  _spi->beginTransaction(
      SPISettings(MCP41100_SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!_chipSelectPcf->digitalWrite(_chipSelectPin, LOW)) {
    _spi->endTransaction();
    return false;
  }
  _spi->transfer(MCP41100_WRITE_WIPER);
  _spi->transfer(value);
  _spi->endTransaction();

  // MCP41100 latches the 16-bit command when CS rises.
  if (!_chipSelectPcf->digitalWrite(_chipSelectPin, HIGH)) {
    return false;
  }

  _wiper = value;
  return true;
}
