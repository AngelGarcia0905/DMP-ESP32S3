#pragma once
#include <Arduino.h>

// AS5600 magnetic angle sensor (I2C 0x36, shares the bus with the IMUs).
// Mide el angulo REAL del laser para la compensacion angular automatica: con el laser
// perpendicular a la hoja se guarda el cero (setZeroHere, queda en flash) y desde entonces
// relativeDeg() da el angulo real respecto a la perpendicular. No se usa para el home.
// Everything is non-blocking: call update() on every loop() pass, from the same task as Wire.
class As5600 {
 public:
  enum class Magnet : uint8_t { None, Weak, Strong, Ok };

  void begin();
  void update();

  bool present() const { return _present; }
  Magnet magnet() const { return _magnet; }
  bool live() const { return _present && _magnet != Magnet::None; }  // angle is meaningful
  uint16_t raw() const { return _raw; }                               // 0..4095
  float angleDeg() const { return _raw * (360.0f / 4096.0f); }
  uint8_t agc() const { return _agc; }       // 0..128 at 3.3 V; lower = stronger field
  uint16_t magnitude() const { return _mag; }

  // Cero del sensor (laser perpendicular a la hoja). Se guarda en flash (namespace "as5600").
  bool hasZero() const { return _hasZero; }
  bool setZeroHere();  // false si el sensor no esta listo (sin iman / no responde)
  // Angulo real del laser en grados, -180..180, respecto al cero. Solo valido si live() y hasZero().
  float relativeDeg() const;

  const char* magnetName() const;
  // Magnet distance quality, from the AGC: too_close, close, ideal, far, too_far
  // (or the magnet state when it is missing / weak / strong).
  const char* zone() const;

  static constexpr uint8_t ADDR = 0x36;

 private:
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);

  void loadZero();

  bool _hasZero = false;
  uint16_t _zeroRaw = 0;
  bool _present = false;
  Magnet _magnet = Magnet::None;
  uint16_t _raw = 0, _mag = 0;
  uint8_t _agc = 0;
  uint8_t _fails = 0;
  uint32_t _lastRead = 0;
};
