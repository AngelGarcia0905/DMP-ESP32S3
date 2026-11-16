#pragma once
#include <Arduino.h>

// Mapa de pines SOLO para ESP32-S3 (placa tipo DevKitC-1, N16R8: 16 MB flash, 8 MB PSRAM,
// programada por el USB-C marcado UART/COM, chip CH343).
//
// Pines que se EVITAN a proposito (no usar para nada mas):
//   GPIO 0, 3, 45, 46   pines de arranque (strapping): un nivel mal puesto impide arrancar
//   GPIO 19, 20         USB nativo D-/D+
//   GPIO 26 a 32        flash y PSRAM internas (usarlos cuelga el chip)
//   GPIO 33 a 37        PSRAM octal de las placas N16R8
//   GPIO 43, 44         UART0 (puente USB serie, es el monitor serie)
//   GPIO 38, 48         LED RGB de la placa (segun la version de la placa es uno u otro)
//
// Todos los pines elegidos abajo son GPIO normales, sin funcion especial al arrancar,
// con pull-up interno y salida limpia (sin pulsos) durante el reset. Cableado y colores:
// ver README.md. Si agregas algo, anota el pin aqui primero.
namespace Pins {

// Motor del eje X (modulo ULN2003 -> 28BYJ-48)
constexpr uint8_t X_IN1 = 4;
constexpr uint8_t X_IN2 = 5;
constexpr uint8_t X_IN3 = 6;
constexpr uint8_t X_IN4 = 7;

// Motor del eje Y (modulo ULN2003 -> 28BYJ-48)
constexpr uint8_t Y_IN1 = 15;
constexpr uint8_t Y_IN2 = 16;
constexpr uint8_t Y_IN3 = 17;
constexpr uint8_t Y_IN4 = 18;

// Senal del laser (KY-008), directo del pin. El modulo ya trae ~20k a GND en S.
constexpr uint8_t LASER = 14;

// Finales de carrera del riel X (un lado a GND, pull-up interno)
// Final de carrera X- (inicio del riel) en el GPIO 47 y X+ (final del riel) en el GPIO 1. (X- estaba en el GPIO 2: se
// movio a un pin libre.)
constexpr uint8_t LIMIT_X_MIN = 47;
constexpr uint8_t LIMIT_X_MAX = 1;

// Bus I2C (IMUs 0x68/0x69 y AS5600 0x36) - pines I2C por defecto del S3
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;

// LED RGB de la placa (WS2812). Se manejan los dos candidatos con la misma senal:
// el que tenga el LED se enciende y el otro pin queda sin nada conectado.
constexpr uint8_t RGB_A = 48;  // DevKitC-1 v1.0
constexpr uint8_t RGB_B = 38;  // DevKitC-1 v1.1

// Pantalla TFT ST7735 (opcional, aun no se usa en el firmware): SPI2 por hardware
constexpr uint8_t TFT_SCK  = 12;
constexpr uint8_t TFT_MOSI = 11;
constexpr uint8_t TFT_CS   = 10;
constexpr uint8_t TFT_DC   = 13;
constexpr uint8_t TFT_RST  = 21;

// Encoder rotatorio KY-040 (aun sin cablear, sin firmware): cuadratura CLK/DT + boton SW.
// El modulo ya trae resistencias pull-up a su pin "+", asi que alimentado a 3V3 las senales son de 3.3 V.
// Cualquier GPIO del S3 sirve para interrupciones.
constexpr uint8_t KNOB_CLK = 39;
constexpr uint8_t KNOB_DT  = 40;
constexpr uint8_t KNOB_SW  = 41;

// Libres: 2, 42

}  // namespace Pins
