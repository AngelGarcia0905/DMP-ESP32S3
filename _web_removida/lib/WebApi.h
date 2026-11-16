#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "As5600.h"
#include "Imu.h"
#include "Machine.h"

// HTTP API consumed by the panel (web/index.html, which this server also serves):
//   GET /                    the panel itself
//   GET /status
//   GET /move?x={cm}&angle={deg}&speed={1-100}
//   GET /turn?revs={-5..5}&speed={1-100}[&axis=x|y]   bench test: turn a motor (negative = reverse, default X)
//   GET /home
//   GET /stop
//   GET /laser?state=on|off
//   GET /calibrate?offset={deg}[&auto=1]   auto=1: offset from the AS5600 (laser height is fixed at 15 cm)
//   GET /imu/status          live readings + calibration state of both IMUs
//   GET /imu/calibrate?which=white|black|both
//   GET /encoder/status      AS5600: magnet state, angle, angle relative to the zero, suggested compensation
//   GET /limits/start?n={1-10}   calibrar los finales de carrera: n tramos X- <-> X+ contando pasos, promedio y al medio
//   GET /limits/legs         pasos de cada tramo, promedio y mitad
//   GET /encoder/zero        store the current AS5600 angle as the zero (laser perpendicular to the sheet)
// Every reply is JSON with permissive CORS so the panel can also be opened from another origin.
class WebApi {
 public:
  WebApi(Machine& m, Imu& white, Imu& black, As5600& enc) : _m(m), _imus{&white, &black}, _enc(enc) {}
  void begin();
  void handle() { _srv.handleClient(); }

 private:
  void sendJson(const String& body);
  void sendError(const char* msg);
  void onRoot();
  void onStatus();
  void onMove();
  void onTurn();
  void onHome();
  void onStop();
  void onLaser();
  void onCalibrate();
  void onImuStatus();
  void onImuCalibrate();
  void onEncoderStatus();
  void onEncoderZero();
  void onLimitsStart();
  void onLimitsLegs();

  WebServer _srv{80};
  Machine& _m;
  Imu* _imus[2];
  As5600& _enc;
  String _lastCalibration;
};
