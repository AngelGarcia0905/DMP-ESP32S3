#pragma once
#include <Arduino.h>
#include <esp_timer.h>

// Non-blocking half-step driver for a 28BYJ-48 behind a ULN2003 board.
// Los PASOS los da un temporizador de hardware (esp_timer, resolucion de microsegundos): cada motor tiene su propio
// temporizador de un disparo que se reprograma para el instante exacto del siguiente paso. Asi la velocidad puede ser
// cualquier valor (800, 850... pasos/s, no solo 1000, 500, 333...) y la rampa de arranque y frenado es suave.
// Todos los metodos se pueden llamar desde otra tarea mientras el motor se mueve. Llamar service() seguido (cada 1 ms):
// apaga las bobinas cuando el motor lleva un rato quieto.
// Position is counted in half-steps; positive = increasing phase index.
class Stepper28 {
 public:
  Stepper28(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, bool invert = false);

  void begin();                         // pines y temporizador
  void setSpeed(float stepsPerSecond);  // velocidad de crucero (cualquier valor de 10 a 2000 pasos/s)
  void setInvert(bool v) { _invert = v; }  // cambia el sentido de giro (la calibracion de limits lo corrige)
  bool invert() const { return _invert; }
  // Holgura de los engranes en pasos (0 = sin compensar): al cambiar de sentido el motor da esos pasos extra, que no se
  // cuentan en la posicion, antes de volver a contar. Al encender se supone apoyado del lado + (el primer movimiento hacia
  // el lado - recorre la holgura).
  void setBacklash(int steps) { _backlash = steps < 0 ? 0 : steps; if (_slack > _backlash) _slack = _backlash; }

  void moveTo(long target);
  void moveBy(long delta);
  void stop();     // se queda donde esta
  void release();  // all coils off (no holding torque, no heat)
  void service();  // llamar seguido: apaga las bobinas tras RELEASE_AFTER_US sin moverse

  void setPosition(long p);
  long position() const { return _pos; }
  long target() const { return _target; }
  bool isRunning() const { return _pos != _target; }
  uint32_t totalSteps() const { return _total; }  // accumulated, direction ignored

 private:
  static void onTimer(void* arg);
  uint32_t tick();  // da un paso si toca; devuelve los microsegundos hasta el siguiente (0 = nada pendiente)
  void wake();      // arranca el temporizador si estaba dormido
  void energize(uint8_t phase);
  void coilsOff();

  uint8_t _pin[4];
  volatile bool _invert;
  volatile long _pos = 0;
  volatile long _target = 0;
  uint8_t _phase = 0;
  volatile int _backlash = 0;  // holgura total (pasos)
  int _slack = 0;              // donde va dentro de la holgura: 0 = apoyado del lado +, _backlash = del lado -
  float _cruise = 333.0f;  // velocidad de crucero (pasos/s)
  float _v = 0.0f;         // velocidad actual (0 = detenido: el siguiente arranque empieza lento)
  int8_t _dir = 0;
  int64_t _lastUs = 0;     // instante del ultimo paso
  bool _energized = false;
  bool _armed = false;     // el temporizador tiene un paso programado
  volatile uint32_t _total = 0;
  esp_timer_handle_t _timer = nullptr;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};
