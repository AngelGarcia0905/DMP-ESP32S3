// DMP - Posicionador laser, firmware para ESP32-S3.
// LED RGB de la placa (el azul tiene prioridad sobre el rojo):
//   AZUL  = ejecutando algo: laser encendido, motores moviendose, HOME, CALIBRAR LIMITS, calibrar IMUs, todo el
//           marcado de los puntos de la hoja, o editando un eje con la perilla
//   ROJO  = nada en curso y ningun equipo conectado a la red Wi-Fi del ESP32 (red DMPS3)
//   VERDE = nada en curso y conectado, o con el punto de acceso apagado en la pantalla RED
#include <Arduino.h>
#include <Wire.h>

#include "As5600.h"
#include "Imu.h"
#include "Machine.h"
#include "Net.h"
#include "StatusLed.h"
#include "Ui.h"

Machine machine;
Imu imuWhite("white", "IMU blanco", 0x68);
Imu imuBlack("black", "IMU negro", 0x69);
As5600 encoder;
StatusLed led;
Ui ui(machine, imuWhite, imuBlack, encoder);  // pantalla TFT + perilla: desde aqui se hace todo

// Hay alguna accion en curso?
static bool actionRunning() {
  return machine.moving() || machine.homing() || machine.laser() ||
         imuWhite.state() == Imu::State::Running || imuBlack.state() == Imu::State::Running;
}

// Decide el color. El azul se mantiene al menos 400 ms para que se vea incluso en acciones cortas.
static void updateLed() {
  static uint32_t blueUntil = 0;
  const uint32_t now = millis();

  if (actionRunning() || ui.editing() || ui.marking()) blueUntil = now + 400;

  if (static_cast<int32_t>(blueUntil - now) > 0) {
    led.set(StatusLed::Color::Blue);  // algo en curso: azul, haya o no equipos conectados
  } else if (Net::enabled() && Net::clients() == 0) {
    led.set(StatusLed::Color::Red);   // punto de acceso encendido y nadie conectado
  } else {
    led.set(StatusLed::Color::Green); // terminado (conectado, o con el punto de acceso apagado)
  }
}

// Monitor serie (115200): L = calibrar finales de carrera (Cfg::LIMIT_CAL_SAMPLES muestras), C = cancelar, Z = marcar el
// HOME de Y donde este el laser. Imprime el avance.
static void serialLimits() {
  static int printed = 0;
  static bool wasTesting = false;
  static uint32_t lastInfo = 0;
  while (Serial.available()) {
    const int c = Serial.read();
    if (c == 'L' || c == 'l') {
      const char* err = "";
      if (machine.startLimitCal(Cfg::LIMIT_CAL_SAMPLES, &err))
        Serial.printf("[limits] calibracion iniciada (X-:%d X+:%d al empezar)\n", machine.limitXMin(), machine.limitXMax());
      else Serial.printf("[limits] no se pudo iniciar: %s\n", err);
    } else if (c == 'C' || c == 'c') {
      machine.cancelHome();
      Serial.println("[limits] cancelado");
    } else if (c == 'Z' || c == 'z') {  // marcar el HOME de Y aqui (igual que CALIBRAR > HOME (MARCAR) > MARCAR Y)
      const char* err = "";
      const float antes = encoder.relativeDeg();
      if (machine.markHome(false, true, &err))
        Serial.printf("[home] HOME de Y marcado aqui: el AS5600 marcaba %+.2f grd y ahora es 0 (cero raw=%u)\n", antes,
                      encoder.raw());
      else Serial.printf("[home] no se pudo marcar el HOME de Y: %s\n", err);
    }
  }
  // Cada cambio de los finales de carrera, con la posicion de X en pasos (sirve para ver donde toca cada uno).
  static int pMin = -1, pMax = -1;
  const int a = machine.limitXMin(), b = machine.limitXMax();
  if (a != pMin || b != pMax) {
    Serial.printf("[limits] interruptores: X-:%d X+:%d  (X = %ld pasos, %s)\n", a, b,
                  machine.xSteps(), machine.testing() ? machine.homePhase() : "sin prueba");
    pMin = a;
    pMax = b;
  }
  if (machine.legCount() < printed) printed = 0;
  while (printed < machine.legCount()) {
    Serial.printf("[limits] muestra %d: %ld pasos (%s)\n", printed + 1, machine.legSteps(printed),
                  machine.legDir(printed) > 0 ? "X- -> X+" : "X+ -> X-");
    printed++;
  }
  if (machine.testing() && millis() - lastInfo > 2000) {
    lastInfo = millis();
    Serial.printf("[limits] %s | X=%ld pasos | X-:%d X+:%d\n", machine.homePhase(), machine.xSteps(),
                  machine.limitXMin(), machine.limitXMax());
  }
  if (wasTesting && !machine.testing()) {
    if (machine.avgSteps() > 0) {
      Serial.printf("[limits] LISTO: promedio %ld pasos = %.2f cm | mitad = %ld pasos | X ahora = %.2f cm\n", machine.avgSteps(),
                    machine.railCm(), machine.middleSteps(), machine.xCm());
      if (machine.xFlipped())
        Serial.printf("[limits] el sentido del motor X estaba al reves: corregido y guardado (invertido = %d)\n", machine.xInvertEffective());
    }
    else
      Serial.printf("[limits] TERMINO CON ERROR: %s\n", machine.lastError());
  }
  wasTesting = machine.testing();
}

void setup() {
  led.begin();  // rojo desde el primer momento
  Serial.begin(115200);
  delay(300);
  Serial.println("\nDMP laser positioner - ESP32-S3");

  machine.begin();  // el laser se fuerza a OFF aqui
  machine.startMotionTask();

  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(400000);
  imuWhite.begin();
  imuBlack.begin();
  encoder.begin();
  machine.attachEncoder(&encoder);  // el HOME de Y va al cero del AS5600
  // El contador de Y arranca donde marca el AS5600 (respecto a su cero guardado, que NO se toca aqui), para que los
  // puntos de la hoja guardados sigan valiendo despues de apagar.
  if (encoder.live() && encoder.hasZero()) {
    machine.setYFromDeg(encoder.relativeDeg());
    Serial.printf("[y] contador de Y tomado del AS5600: %+.2f deg\n", machine.angleDeg());
  } else {
    Serial.println("[y] AS5600 sin iman o sin cero: Y arranca en 0 donde este (sin referencia)");
  }
  if (machine.gridPoints()) Serial.printf("[hoja] calibrada con %d puntos (MOVER con limites y mapa)\n", machine.gridPoints());
  else Serial.println("[hoja] sin calibrar");

  Net::begin();
  ui.begin();
}

void loop() {
  imuWhite.update();
  imuBlack.update();
  encoder.update();
  ui.update();
  serialLimits();
  updateLed();
  delay(1);
}
