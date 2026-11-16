#include "Stepper28.h"

#include "dmp_config.h"

namespace {
// Half-step sequence for IN1..IN4 (bit0 = IN1).
constexpr uint8_t SEQ[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
constexpr int64_t RELEASE_AFTER_US = (int64_t)Cfg::MOTOR_HOLD_MS * 1000;  // bobinas sueltas tras este tiempo sin pasos
constexpr float V_START = Cfg::RAMP_START_SPS;  // velocidad de arranque (donde el motor cargado no se salta pasos)
// Rampa: la velocidad sube ACCEL pasos/s en cada paso y baja igual al acercarse al destino, para frenar llegando a
// V_START (valores y razon en dmp_config.h).
constexpr float ACCEL = Cfg::RAMP_ACCEL_SPS;
constexpr float V_MIN = 10.0f, V_MAX = 2000.0f;  // rango que acepta setSpeed
}  // namespace

Stepper28::Stepper28(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, bool invert)
    : _pin{in1, in2, in3, in4}, _invert(invert) {}

void Stepper28::begin() {
  for (uint8_t p : _pin) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  esp_timer_create_args_t args = {};
  args.callback = &Stepper28::onTimer;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;  // corre en la tarea de esp_timer (no en una interrupcion): seguro con la flash
  args.name = "step";
  esp_timer_create(&args, &_timer);
}

void Stepper28::setSpeed(float sps) {
  portENTER_CRITICAL(&_mux);
  _cruise = constrain(sps, V_MIN, V_MAX);
  portEXIT_CRITICAL(&_mux);
}

void Stepper28::moveTo(long target) {
  portENTER_CRITICAL(&_mux);
  _target = target;
  portEXIT_CRITICAL(&_mux);
  wake();
}

void Stepper28::moveBy(long delta) {
  portENTER_CRITICAL(&_mux);
  _target = _pos + delta;
  portEXIT_CRITICAL(&_mux);
  wake();
}

void Stepper28::stop() {
  portENTER_CRITICAL(&_mux);
  _target = _pos;
  portEXIT_CRITICAL(&_mux);
}

void Stepper28::setPosition(long p) {
  portENTER_CRITICAL(&_mux);
  _pos = p;
  _target = p;
  portEXIT_CRITICAL(&_mux);
}

void Stepper28::release() {
  portENTER_CRITICAL(&_mux);
  coilsOff();
  portEXIT_CRITICAL(&_mux);
}

void Stepper28::service() {
  portENTER_CRITICAL(&_mux);
  if (_energized && !_armed && _pos == _target && esp_timer_get_time() - _lastUs >= RELEASE_AFTER_US) coilsOff();
  portEXIT_CRITICAL(&_mux);
}

void Stepper28::coilsOff() {
  for (uint8_t p : _pin) digitalWrite(p, LOW);
  _energized = false;
}

void Stepper28::energize(uint8_t phase) {
  uint8_t bits = SEQ[phase & 7];
  for (uint8_t i = 0; i < 4; i++) digitalWrite(_pin[i], (bits >> i) & 1);
  _energized = true;
}

// Si el temporizador estaba dormido y hay pasos pendientes, programa el primero. Respeta el intervalo de arranque
// desde el ultimo paso (por si el motor acaba de detenerse).
void Stepper28::wake() {
  if (!_timer) return;
  bool start = false;
  int64_t delay = 50;
  portENTER_CRITICAL(&_mux);
  if (!_armed && _pos != _target) {
    _armed = true;
    start = true;
    const int64_t since = esp_timer_get_time() - _lastUs;
    const int64_t startUs = (int64_t)(1e6f / V_START);
    if (since < startUs) delay = startUs - since;
  }
  portEXIT_CRITICAL(&_mux);
  if (start) esp_timer_start_once(_timer, delay);
}

void Stepper28::onTimer(void* arg) {
  Stepper28* s = static_cast<Stepper28*>(arg);
  const uint32_t next = s->tick();
  if (next) esp_timer_start_once(s->_timer, next);
}

uint32_t Stepper28::tick() {
  uint32_t next = 0;
  portENTER_CRITICAL(&_mux);
  if (_pos == _target) {  // llego (o lo detuvieron): el siguiente arranque empieza lento
    _v = 0.0f;
    _dir = 0;
    _armed = false;
  } else {
    const int dir = (_target > _pos) ? 1 : -1;
    if (dir != _dir) {  // arranque o inversion de giro: rampa desde V_START
      _dir = (int8_t)dir;
      _v = 0.0f;
    }
    // Holgura de los engranes: _slack va de 0 (engranes apoyados del lado +) a _backlash (apoyados del lado -). Al
    // invertir el sentido, los primeros pasos solo recorren la holgura: el motor gira pero la posicion no cuenta.
    if (dir > 0 && _slack > 0) _slack--;
    else if (dir < 0 && _slack < _backlash) _slack++;
    else _pos += dir;
    _phase = (uint8_t)((_phase + 8 + (_invert ? -dir : dir)) & 7);
    energize(_phase);
    _lastUs = esp_timer_get_time();
    _total++;

    // Velocidad para el siguiente paso: sube ACCEL por paso, sin pasar del crucero, y baja al acercarse al destino.
    const float vLow = _cruise < V_START ? _cruise : V_START;
    float v = (_v <= 0.0f) ? vLow : _v + ACCEL;
    const float brake = vLow + labs(_target - _pos) * ACCEL;
    if (v > brake) v = brake;
    if (v > _cruise) v = _cruise;
    _v = v;
    next = (uint32_t)(1e6f / v);  // aunque ya llego: la siguiente llamada solo marca que termino
  }
  portEXIT_CRITICAL(&_mux);
  return next;
}
