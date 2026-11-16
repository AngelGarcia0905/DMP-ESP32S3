#pragma once
#include <Arduino.h>
#include "Mpu.h"

// Stored per IMU. The IMUs stay fixed on their base, so there is one calibration pose:
// resting flat with +Z up. Calibrating makes that pose read exactly (0, 0, +1 g) and the
// gyro read 0 deg/s. Scale cannot be found from a single pose, so it stays at 1 g = 16384.
struct ImuCal {
  bool valid = false;
  float off[3] = {0, 0, 0};    // accel offset, counts
  float gbias[3] = {0, 0, 0};  // gyro zero-rate bias, counts
  char when[20] = "";          // "YYYY-MM-DDTHH:MM:SS", empty when the clock was not synced
};

// One MPU on the I2C bus. Everything is non-blocking: call update() on every loop() pass.
// All calls must come from the same task as Wire (the loop task); there is no locking.
class Imu {
 public:
  enum class State : uint8_t { Idle, Running, Done, Failed };

  // id is "white" / "black" (used in the API and as the flash namespace "imu_<id>").
  Imu(const char* id, const char* label, uint8_t addr) : _id(id), _mpu(label, addr) {}

  void begin();   // probes the sensor and loads the stored calibration
  void update();  // live reads + calibration sampling + reconnect attempts

  bool canStart(const char** err) const;
  bool startCalibration(const char** err);

  const char* id() const { return _id; }
  const char* name() const { return _mpu.name(); }
  uint8_t addr() const { return _mpu.addr(); }
  const char* model() const { return _mpu.model(); }
  bool ok() const { return _mpu.ok(); }
  State state() const { return _state; }
  int progress() const;
  const char* error() const { return _error; }
  const ImuCal& cal() const { return _cal; }
  bool liveValid() const { return _liveValid; }
  float tempC() const { return _live.tempC; }

  // Latest reading with the stored calibration applied: accel in g, gyro in deg/s.
  void corrected(float a[3], float g[3]) const;

  // Inclinacion del chip en grados respecto al "nivel" en que se calibro (la calibracion deja ax = ay = 0 y az = 1 g en esa
  // posicion, asi que hay que calibrar sobre una superficie realmente nivelada). incX / incY = inclinacion a lo largo del
  // eje X / Y del chip (el + de cada eje apuntando hacia arriba da signo +). Con promedio suave (se llama cada cuadro).
  // false si no hay lectura valida.
  bool tilt(float* incXDeg, float* incYDeg) const;

 private:
  void load();
  void save() const;
  void accumulate(const MpuSample& s);
  void finish();
  void fail(const char* msg);

  const char* _id;
  Mpu _mpu;
  ImuCal _cal;

  MpuSample _live{};
  bool _liveValid = false;
  mutable float _tiltX = 0, _tiltY = 0;  // promedio suave de la inclinacion
  mutable bool _tiltInit = false;
  uint32_t _lastRead = 0, _lastProbe = 0;

  State _state = State::Idle;
  const char* _error = "";
  char _errBuf[128] = "";  // backing store for errors that carry numbers
  int _n = 0, _fails = 0;
  double _sum[6] = {}, _sq[6] = {};
};
