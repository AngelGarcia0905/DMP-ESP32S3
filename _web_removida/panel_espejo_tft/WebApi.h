#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "As5600.h"
#include "Imu.h"
#include "Machine.h"

// Servidor del panel web (web/index.html, embebido en la flash). El panel tiene los MISMOS modulos que la pantalla TFT
// (MOVER, HOME, CALIBRAR, SENSORES, RED) y usa la misma Machine, asi que lo que se hace en uno se ve en el otro.
// Todas las respuestas son JSON ({"ok":true} o {"ok":false,"error":"..."}):
//   GET /                         el panel
//   GET /status                   todo el estado (posiciones, limites, HOME, calibraciones, sensores, red)
//   GET /move?x={cm}&angle={grd}  mover dentro de los TOPES (como MOVER; sin topes, libre)
//   GET /jog?x={cm}&angle={grd}   mover SIN limites (todo lo de CALIBRAR)
//   GET /sheet?h={mm}&v={mm}      ir a un punto de la hoja con el mapa de los puntos marcados
//   GET /home?axis=both|x|y       HOME de los dos, solo X o solo Y, al HOME marcado (Y por el lado corto con el AS5600)
//   GET /home/mark?axis=both|x|y  la posicion actual pasa a ser el HOME (X = 0 / Y = 0; Y sobrescribe el cero del AS5600)
//   GET /refx                     REFERENCIAR X con los finales de carrera
//   GET /cancel                   cancela un HOME o una calibracion de limits
//   GET /stop                     paro: detiene los motores (NO apaga el laser)
//   GET /laser?state=on|off
//   GET /imu/calibrate            calibra las IMUs conectadas
//   GET /limits/start?n={1-5}     calibracion de los finales de carrera con n corridas
//   GET /grid/begin?n=4|6|9       empieza a marcar los puntos de la hoja
//   GET /grid/mark?i={0..n-1}     la posicion actual pasa a ser el punto i
//   GET /grid/commit              termina el marcado: se guarda en la flash
//   GET /tope/set?i={0-3}         TOPES de MOVER: la posicion actual pasa a ser X min, X max, Y min o Y max
//   GET /tope/clear               borra los topes min/max (MOVER queda limitado por los puntos de la hoja, o libre)
//   GET /speed?x={p/s}&y={p/s}    velocidad de cada eje (se guarda en la flash)
//   GET /ap?state=on|off          punto de acceso (apagarlo corta la conexion con el panel)
class WebApi {
 public:
  WebApi(Machine& m, Imu& white, Imu& black, As5600& enc) : _m(m), _imus{&white, &black}, _enc(enc) {}
  void begin();   // registra las rutas (el servidor arranca solo cuando el punto de acceso esta encendido)
  void handle();  // llamar seguido desde loop()

 private:
  void sendJson(const String& body);
  void sendOk() { sendJson("{\"ok\":true}"); }
  void sendError(const char* msg);
  void reply(bool ok, const char* err) { if (ok) sendOk(); else sendError(err); }
  bool argF(const char* name, float* v);  // false (y responde el error) si falta el argumento
  void onRoot();
  void onStatus();

  WebServer _srv{80};
  bool _started = false;
  Machine& _m;
  Imu* _imus[2];
  As5600& _enc;
};
