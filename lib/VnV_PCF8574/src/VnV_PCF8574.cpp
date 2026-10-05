#include "VnV_PCF8574.h"

VnV_PCF8574::VnV_PCF8574(TwoWire &wire, uint8_t address)
    : _wire(&wire), _address(address), _state(0xFF), _inputMask(0xFF),
      _initialized(false) {}

bool VnV_PCF8574::validPin(uint8_t pin) const {
  return pin < 8;
}

bool VnV_PCF8574::validAddress() const {
  return (_address >= 0x20 && _address <= 0x27) ||
         (_address >= 0x38 && _address <= 0x3F);
}

bool VnV_PCF8574::begin(uint8_t initialState) {
  _initialized = false;
  if (!validAddress()) {
    return false;
  }

  _wire->beginTransmission(_address);
  if (_wire->endTransmission() != 0) {
    return false;
  }

  _state = initialState;
  _inputMask = 0xFF;
  if (!write8(_state)) {
    return false;
  }

  _initialized = true;
  return true;
}

bool VnV_PCF8574::write8(uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return false;
  }
  _state = value;
  return true;
}

int VnV_PCF8574::read8() {
  if (_wire->requestFrom(static_cast<int>(_address), 1) != 1 ||
      !_wire->available()) {
    return -1;
  }
  return _wire->read();
}

bool VnV_PCF8574::pinMode(uint8_t pin, uint8_t mode) {
  if (!validPin(pin)) {
    return false;
  }

  if (mode == INPUT || mode == INPUT_PULLUP) {
    _inputMask |= static_cast<uint8_t>(1U << pin);
    return digitalWrite(pin, HIGH); // release the pin for input
  }

  if (mode == OUTPUT) {
    _inputMask &= static_cast<uint8_t>(~(1U << pin));
    return true;
  }

  return false;
}

bool VnV_PCF8574::digitalWrite(uint8_t pin, uint8_t value) {
  if (!validPin(pin)) {
    return false;
  }

  const uint8_t mask = static_cast<uint8_t>(1U << pin);
  if (value == LOW) {
    _state &= static_cast<uint8_t>(~mask);
  } else {
    // HIGH is a released/quasi-pull-up state, not a strong push-pull HIGH.
    _state |= mask;
  }
  return write8(_state);
}

int VnV_PCF8574::digitalRead(uint8_t pin) {
  if (!validPin(pin)) {
    return -1;
  }
  const int value = read8();
  if (value < 0) {
    return -1;
  }
  return (value & (1U << pin)) ? HIGH : LOW;
}
