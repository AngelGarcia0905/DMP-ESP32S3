#pragma once
#include <Arduino.h>
#include <Wire.h>

// Raw-register driver for the MPU6050 family (MPU6050 / MPU6500 / MPU9250) on I2C.
// Call Wire.begin() first. Ranges: accel +-2 g (16384 LSB/g), gyro +-250 deg/s (131 LSB/(deg/s)).
struct MpuSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  float tempC;
};

class Mpu {
 public:
  static constexpr float ACCEL_LSB_PER_G = 16384.0f;
  static constexpr float GYRO_LSB_PER_DPS = 131.0f;

  Mpu(const char* name, uint8_t addr) : _name(name), _addr(addr) {}

  bool begin() {
    if (!readRegs(0x75, &_who, 1)) return false;
    // Same sequence for every chip of the family, so all IMUs end up configured identically.
    writeReg(0x6B, 0x80);  // device reset
    delay(100);
    writeReg(0x68, 0x07);  // reset the gyro, accel and temp signal paths
    delay(100);
    writeReg(0x6B, 0x01);  // wake up, clock from the gyro PLL (steadier than the internal oscillator)
    writeReg(0x19, 0x00);  // sample rate divider 0 (1 kHz with the low-pass filter on)
    writeReg(0x1A, 0x03);  // gyro/temp low-pass ~44 Hz
    writeReg(0x1B, 0x00);  // gyro  +-250 deg/s
    writeReg(0x1C, 0x00);  // accel +-2 g
    writeReg(0x1D, 0x03);  // accel low-pass ~44 Hz (MPU6500/9250 only; ignored by MPU6050)
    delay(50);             // let the first conversions settle
    _ok = true;
    return true;
  }

  bool read(MpuSample& s) {
    uint8_t b[14];
    if (!readRegs(0x3B, b, 14)) return false;
    auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
    s.ax = s16(0); s.ay = s16(2); s.az = s16(4);
    int16_t t = s16(6);
    s.gx = s16(8); s.gy = s16(10); s.gz = s16(12);
    s.tempC = (_who == 0x68) ? t / 340.0f + 36.53f : t / 333.87f + 21.0f;
    return true;
  }

  const char* name() const { return _name; }
  uint8_t addr() const { return _addr; }
  uint8_t whoAmI() const { return _who; }
  bool ok() const { return _ok; }
  const char* model() const {
    switch (_who) {
      case 0x68: return "MPU6050";
      case 0x70: return "MPU6500";
      case 0x71: return "MPU9250";
      case 0x73: return "MPU9255";
      default: return "unknown";
    }
  }

 private:
  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(_addr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
  }

  const char* _name;
  uint8_t _addr;
  uint8_t _who = 0;
  bool _ok = false;
};
