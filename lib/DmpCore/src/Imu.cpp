#include "Imu.h"

#include <Preferences.h>
#include <math.h>

#include "Net.h"

namespace {

constexpr int WIN = 100;                  // samples per calibration window (~1 s)
constexpr double STILL_ACCEL_STD = 100;   // counts (~0.006 g)
constexpr double STILL_GYRO_STD = 40;     // counts (~0.3 deg/s)
constexpr double LEVEL_Z_MIN = 0.80 * Mpu::ACCEL_LSB_PER_G;   // +Z must carry most of gravity
constexpr double LEVEL_XY_MAX = 0.20 * Mpu::ACCEL_LSB_PER_G;  // ~11 deg of tilt at most

constexpr uint32_t CAL_PERIOD_MS = 10;     // sample period while calibrating
constexpr uint32_t LIVE_PERIOD_MS = 100;   // sample period otherwise
constexpr uint32_t PROBE_PERIOD_MS = 2000; // retry period while the sensor does not answer
constexpr int MAX_READ_FAILS = 20;

}  // namespace

void Imu::begin() {
  _mpu.begin();
  _lastProbe = millis();
  load();
}

void Imu::load() {
  Preferences p;
  char ns[16];
  snprintf(ns, sizeof(ns), "imu_%s", _id);
  if (!p.begin(ns, true)) return;
  if (p.getBytesLength("cal") == sizeof(ImuCal)) p.getBytes("cal", &_cal, sizeof(ImuCal));
  p.end();
}

void Imu::save() const {
  Preferences p;
  char ns[16];
  snprintf(ns, sizeof(ns), "imu_%s", _id);
  if (!p.begin(ns, false)) return;
  p.putBytes("cal", &_cal, sizeof(ImuCal));
  p.end();
}

void Imu::update() {
  const uint32_t now = millis();

  if (!_mpu.ok()) {
    _liveValid = false;
    if (now - _lastProbe >= PROBE_PERIOD_MS) {
      _lastProbe = now;
      _mpu.begin();
    }
    return;
  }

  const uint32_t period = (_state == State::Running) ? CAL_PERIOD_MS : LIVE_PERIOD_MS;
  if (now - _lastRead < period) return;
  _lastRead = now;

  MpuSample s;
  if (!_mpu.read(s)) {
    _liveValid = false;
    if (_state == State::Running && ++_fails >= MAX_READ_FAILS) fail("I2C read failed");
    return;
  }
  _fails = 0;
  _live = s;
  _liveValid = true;
  if (_state == State::Running) accumulate(s);
}

bool Imu::canStart(const char** err) const {
  if (!_mpu.ok()) { *err = "not responding"; return false; }
  if (_state == State::Running) { *err = "already calibrating"; return false; }
  return true;
}

bool Imu::startCalibration(const char** err) {
  if (!canStart(err)) return false;
  memset(_sum, 0, sizeof(_sum));
  memset(_sq, 0, sizeof(_sq));
  _n = 0;
  _fails = 0;
  _error = "";
  _state = State::Running;
  return true;
}

int Imu::progress() const {
  if (_state == State::Running) return _n * 100 / WIN;
  return _state == State::Done ? 100 : 0;
}

void Imu::accumulate(const MpuSample& s) {
  const double v[6] = {(double)s.ax, (double)s.ay, (double)s.az,
                       (double)s.gx, (double)s.gy, (double)s.gz};
  for (int i = 0; i < 6; i++) {
    _sum[i] += v[i];
    _sq[i] += v[i] * v[i];
  }
  if (++_n >= WIN) finish();
}

void Imu::finish() {
  double mu[6], sd[6];
  for (int i = 0; i < 6; i++) {
    mu[i] = _sum[i] / WIN;
    sd[i] = sqrt(max(0.0, _sq[i] / WIN - mu[i] * mu[i]));
  }

  for (int i = 0; i < 3; i++) {
    if (sd[i] > STILL_ACCEL_STD || sd[i + 3] > STILL_GYRO_STD) {
      snprintf(_errBuf, sizeof(_errBuf),
               "not still: std accel %.0f %.0f %.0f (max %.0f), gyro %.0f %.0f %.0f (max %.0f) counts",
               sd[0], sd[1], sd[2], STILL_ACCEL_STD, sd[3], sd[4], sd[5], STILL_GYRO_STD);
      Serial.printf("[imu] %s %s\n", name(), _errBuf);
      fail(_errBuf);
      return;
    }
  }
  if (mu[2] < LEVEL_Z_MIN || fabs(mu[0]) > LEVEL_XY_MAX || fabs(mu[1]) > LEVEL_XY_MAX) {
    fail("not resting flat with +Z up");
    return;
  }

  ImuCal c;
  c.valid = true;
  c.off[0] = (float)mu[0];
  c.off[1] = (float)mu[1];
  c.off[2] = (float)(mu[2] - Mpu::ACCEL_LSB_PER_G);
  for (int i = 0; i < 3; i++) c.gbias[i] = (float)mu[i + 3];
  Net::isoNow().toCharArray(c.when, sizeof(c.when));

  _cal = c;
  save();
  _state = State::Done;
}

void Imu::fail(const char* msg) {
  _error = msg;
  _state = State::Failed;
}

bool Imu::tilt(float* incXDeg, float* incYDeg) const {
  if (!_mpu.ok() || !_liveValid) { _tiltInit = false; return false; }
  float a[3], g[3];
  corrected(a, g);
  // pendiente a lo largo de cada eje del chip: asin de la componente de la gravedad en ese eje
  const float nx = constrain(a[0], -1.0f, 1.0f), ny = constrain(a[1], -1.0f, 1.0f);
  const float tx = asinf(nx) * 57.29578f, ty = asinf(ny) * 57.29578f;
  if (!_tiltInit) { _tiltX = tx; _tiltY = ty; _tiltInit = true; }
  else { _tiltX += 0.2f * (tx - _tiltX); _tiltY += 0.2f * (ty - _tiltY); }
  *incXDeg = _tiltX;
  *incYDeg = _tiltY;
  return true;
}

void Imu::corrected(float a[3], float g[3]) const {
  const float ra[3] = {(float)_live.ax, (float)_live.ay, (float)_live.az};
  const float rg[3] = {(float)_live.gx, (float)_live.gy, (float)_live.gz};
  for (int i = 0; i < 3; i++) {
    a[i] = (ra[i] - _cal.off[i]) / Mpu::ACCEL_LSB_PER_G;
    g[i] = (rg[i] - _cal.gbias[i]) / Mpu::GYRO_LSB_PER_DPS;
  }
}
