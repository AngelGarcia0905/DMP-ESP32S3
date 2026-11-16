#include "As5600.h"

#include <Preferences.h>
#include <Wire.h>

#include "dmp_config.h"

namespace {
constexpr uint8_t REG_STATUS = 0x0B;     // bit5 MD (magnet detected), bit4 ML (too weak), bit3 MH (too strong)
constexpr uint8_t REG_RAW_ANGLE = 0x0C;  // 12 bits
constexpr uint8_t REG_AGC = 0x1A;
constexpr uint8_t REG_MAGNITUDE = 0x1B;  // 12 bits
constexpr uint32_t READ_MS = 100;
constexpr uint32_t PROBE_MS = 2000;
constexpr uint8_t MAX_FAILS = 3;
}  // namespace

bool As5600::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void As5600::loadZero() {
  Preferences p;
  if (!p.begin("as5600", true)) return;
  _hasZero = p.isKey("zero");
  _zeroRaw = p.getUShort("zero", 0) & 0x0FFF;
  p.end();
}

bool As5600::setZeroHere() {
  if (!live()) return false;
  Preferences p;
  if (!p.begin("as5600", false)) return false;
  p.putUShort("zero", _raw);
  p.end();
  _zeroRaw = _raw;
  _hasZero = true;
  return true;
}

float As5600::relativeDeg() const {
  int d = (int)_raw - (int)_zeroRaw;  // cuentas, -4095..4095
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  float deg = d * (360.0f / 4096.0f) / Cfg::ENC_RATIO;
  return Cfg::ANGLE_SIGN * (Cfg::ENC_INVERT ? -deg : deg);  // mismo sentido de angulos que el motor
}

void As5600::begin() {
  loadZero();
  _lastRead = 0;
  update();
  Serial.printf("[as5600] %s\n", _present ? "found at 0x36" : "not found at 0x36 (will keep probing)");
  if (_hasZero) Serial.printf("[as5600] cero del angulo guardado: raw=%u (%.2f deg)\n", _zeroRaw, _zeroRaw * (360.0f / 4096.0f));
  else Serial.println("[as5600] sin cero guardado");
}

void As5600::update() {
  uint32_t now = millis();
  uint32_t period = _present ? READ_MS : PROBE_MS;
  if (_lastRead != 0 && now - _lastRead < period) return;
  _lastRead = now ? now : 1;

  uint8_t st, agc, a[2], m[2];
  if (!readRegs(REG_STATUS, &st, 1) || !readRegs(REG_RAW_ANGLE, a, 2) ||
      !readRegs(REG_AGC, &agc, 1) || !readRegs(REG_MAGNITUDE, m, 2)) {
    if (++_fails >= MAX_FAILS) {
      _present = false;
      _magnet = Magnet::None;
    }
    return;
  }
  _fails = 0;
  _present = true;
  _raw = ((a[0] & 0x0F) << 8) | a[1];
  _mag = ((m[0] & 0x0F) << 8) | m[1];
  _agc = agc;

  if (!(st & 0x20)) _magnet = Magnet::None;
  else if (st & 0x10) _magnet = Magnet::Weak;
  else if (st & 0x08) _magnet = Magnet::Strong;
  else _magnet = Magnet::Ok;
}

const char* As5600::magnetName() const {
  switch (_magnet) {
    case Magnet::Ok: return "ok";
    case Magnet::Weak: return "weak";
    case Magnet::Strong: return "strong";
    default: return "none";
  }
}

const char* As5600::zone() const {
  if (_magnet != Magnet::Ok) return magnetName();
  if (_agc <= 10) return "too_close";
  if (_agc < 30) return "close";
  if (_agc <= 70) return "ideal";
  if (_agc <= 100) return "far";
  return "too_far";
}
