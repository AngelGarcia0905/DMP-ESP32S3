#pragma once
#include <Arduino.h>

// Digital input with a simple "N identical samples in a row" debounce.
// Call update() on every loop pass / every millisecond.
class DebouncedInput {
 public:
  void begin(uint8_t pin, uint8_t activeLevel, bool pullup) {
    _pin = pin;
    _activeLevel = activeLevel;
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    _stable = _last = digitalRead(pin);
    _count = 0;
  }

  void update() {
    int v = digitalRead(_pin);
    if (v == _last) {
      if (_count < kSamples) _count++;
      if (_count >= kSamples) _stable = v;
    } else {
      _last = v;
      _count = 0;
    }
  }

  bool active() const { return _stable == _activeLevel; }
  int raw() const { return digitalRead(_pin); }

 private:
  static constexpr uint8_t kSamples = 3;
  uint8_t _pin = 0;
  uint8_t _activeLevel = LOW;
  volatile int _stable = HIGH;
  int _last = HIGH;
  uint8_t _count = 0;
};
