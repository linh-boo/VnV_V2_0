#pragma once

#include <Arduino.h>
#include <Wire.h>

// PCF8574 is a quasi-bidirectional I/O expander. It has no direction
// register: writing 0 drives a pin LOW and writing 1 releases it HIGH.
class VnV_PCF8574 {
public:
  VnV_PCF8574(TwoWire &wire, uint8_t address);

  bool begin(uint8_t initialState = 0xFF);
  bool pinMode(uint8_t pin, uint8_t mode);
  bool digitalWrite(uint8_t pin, uint8_t value);
  int digitalRead(uint8_t pin);
  bool write8(uint8_t value);
  int read8();

  uint8_t address() const { return _address; }
  uint8_t outputState() const { return _state; }
  bool isInitialized() const { return _initialized; }

private:
  bool validPin(uint8_t pin) const;
  bool validAddress() const;

  TwoWire *_wire;
  uint8_t _address;
  uint8_t _state;
  uint8_t _inputMask;
  bool _initialized;
};
