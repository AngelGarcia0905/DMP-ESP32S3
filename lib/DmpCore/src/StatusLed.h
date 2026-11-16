#pragma once
#include <Arduino.h>

// LED RGB (WS2812) de la placa ESP32-S3 como indicador de estado (el color lo decide main.cpp):
//   AZUL  = ejecutando algo (laser, movimiento, home, calibraciones, marcado de puntos); tiene prioridad
//   ROJO  = nada en curso y ningun equipo conectado a la red Wi-Fi del ESP32
//   VERDE = nada en curso y conectado, o con el punto de acceso apagado
// El core de Arduino solo inicializa un pin en neopixelWrite(), por eso aqui se usa el RMT
// directamente para poder manejar los dos pines candidatos (Pins::RGB_A y Pins::RGB_B).
class StatusLed {
 public:
  enum class Color : uint8_t { Off, Red, Blue, Green };

  void begin();
  void set(Color c);  // solo reescribe el LED cuando el color cambia
  Color color() const { return _color; }

 private:
  void write(uint8_t r, uint8_t g, uint8_t b);

  Color _color = Color::Off;
  bool _first = true;
};
