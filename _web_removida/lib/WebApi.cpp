#include "WebApi.h"

#include "Net.h"

// The panel, embedded in flash from web/index.html (board_build.embed_txtfiles).
extern const char indexHtmlStart[] asm("_binary_web_index_html_start");
extern const char indexHtmlEnd[] asm("_binary_web_index_html_end");

void WebApi::begin() {
  _srv.on("/", HTTP_GET, [this] { onRoot(); });
  _srv.on("/index.html", HTTP_GET, [this] { onRoot(); });
  _srv.on("/imu/status", HTTP_GET, [this] { onImuStatus(); });
  _srv.on("/imu/calibrate", HTTP_GET, [this] { onImuCalibrate(); });
  _srv.on("/encoder/status", HTTP_GET, [this] { onEncoderStatus(); });
  _srv.on("/encoder/zero", HTTP_GET, [this] { onEncoderZero(); });
  _srv.on("/status", HTTP_GET, [this] { onStatus(); });
  _srv.on("/move", HTTP_GET, [this] { onMove(); });
  _srv.on("/turn", HTTP_GET, [this] { onTurn(); });
  _srv.on("/home", HTTP_GET, [this] { onHome(); });
  _srv.on("/stop", HTTP_GET, [this] { onStop(); });
  _srv.on("/laser", HTTP_GET, [this] { onLaser(); });
  _srv.on("/calibrate", HTTP_GET, [this] { onCalibrate(); });
  _srv.on("/limits/start", HTTP_GET, [this] { onLimitsStart(); });
  _srv.on("/limits/legs", HTTP_GET, [this] { onLimitsLegs(); });

  // CORS preflight (also the Private Network Access preflight Chrome sends
  // when a page from the internet talks to a device on the LAN).
  _srv.onNotFound([this] {
    if (_srv.method() == HTTP_OPTIONS) {
      _srv.sendHeader("Access-Control-Allow-Origin", "*");
      _srv.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
      _srv.sendHeader("Access-Control-Allow-Headers", "*");
      _srv.sendHeader("Access-Control-Allow-Private-Network", "true");
      _srv.send(204);
    } else if (_srv.hostHeader() != Net::ip() && _srv.hostHeader() != "dmp-laser.local") {
      // Captive portal: the phone/PC connectivity probes ask for some other host
      // (e.g. connectivitycheck.gstatic.com); send them to the panel.
      _srv.sendHeader("Location", String("http://") + Net::ip() + "/", true);
      _srv.sendHeader("Cache-Control", "no-store");
      _srv.send(302, "text/plain", "");
    } else {
      sendError("Unknown endpoint");
    }
  });
  _srv.begin();
}

void WebApi::sendJson(const String& body) {
  _srv.sendHeader("Access-Control-Allow-Origin", "*");
  _srv.sendHeader("Cache-Control", "no-store");
  _srv.send(200, "application/json", body);
}

void WebApi::sendError(const char* msg) {
  sendJson(String("{\"ok\":false,\"error\":\"") + msg + "\"}");
}

void WebApi::onRoot() {
  // The embedded text file carries a trailing NUL that must not be sent.
  _srv.sendHeader("Cache-Control", "no-cache");
  _srv.send_P(200, "text/html; charset=utf-8", indexHtmlStart, indexHtmlEnd - indexHtmlStart - 1);
}

void WebApi::onStatus() {
  char buf[1536];
  snprintf(buf, sizeof(buf),
           "{\"tx\":%.2f,\"ty\":%.2f,\"seq\":%lu,\"src\":\"%c\","  // objetivo vigente y quien lo movio (W web, S pantalla, M maquina)
           "\"yStep\":%.4f,\"xMin\":%.1f,\"xMax\":%.1f,\"yMin\":%.1f,\"yMax\":%.1f,\"xHomed\":%s,\"railCm\":%.1f,\"canMove\":%s,\"testMode\":%s,"
           "\"testing\":%s,\"phase\":\"%s\",\"legs\":%d,"  // paso del motor Y, limites y referencia de X: los mismos que usa la pantalla
           "\"connected\":true,\"x\":%.2f,\"angle\":%.2f,\"laser\":%s,\"calibrated\":%s,"
           "\"lastCalibration\":\"%s\",\"laserHeight\":%.1f,\"limit\":%s,"
           "\"stepsX\":%lu,\"revsX\":%.3f,\"stepsAngle\":%lu,\"revsAngle\":%.3f,"
           "\"limitXMin\":%s,\"limitXMax\":%s,\"homeSensor\":%s,\"angleEncoderOk\":%s,"
           "\"angleOffset\":%.2f,\"moving\":%s,\"homing\":%s,\"error\":\"%s\"}",
           _m.targetXCm(), _m.targetAngleDeg(), (unsigned long)_m.seq(), (char)_m.lastSource(),
           Cfg::Y_DEG_PER_STEP, _m.xMinCm(), _m.xMaxCm(), Cfg::Y_MIN_DEG, Cfg::Y_MAX_DEG,
           _m.xHomed() ? "true" : "false", _m.railCm(),
           _m.canMove() ? "true" : "false", _m.testMode() ? "true" : "false",
           _m.testing() ? "true" : "false", _m.homePhase(), _m.legCount(),
           _m.xCm(), _m.angleDeg(), _m.laser() ? "true" : "false",
           _m.calibrated() ? "true" : "false", _lastCalibration.c_str(), _m.laserHeightCm(),
           (_m.limitXMin() || _m.limitXMax()) ? "true" : "false",
           (unsigned long)_m.stepsX(), _m.revsX(), (unsigned long)_m.stepsY(), _m.revsY(),
           _m.limitXMin() ? "true" : "false", _m.limitXMax() ? "true" : "false",
           _m.homeSensor() ? "true" : "false", _enc.live() ? "true" : "false", _m.angleOffsetDeg(),
           _m.moving() ? "true" : "false",
           _m.homing() ? "true" : "false", _m.lastError());
  sendJson(buf);
}

void WebApi::onMove() {
  if (!_srv.hasArg("x") || !_srv.hasArg("angle")) { sendError("Missing x or angle"); return; }
  int speed = _srv.hasArg("speed") ? _srv.arg("speed").toInt() : 50;
  const char* err = "";
  if (_m.moveTo(_srv.arg("x").toFloat(), _srv.arg("angle").toFloat(), speed, &err)) {
    sendJson("{\"ok\":true}");
  } else {
    sendError(err);
  }
}

void WebApi::onTurn() {
  if (!_srv.hasArg("revs")) { sendError("Missing revs"); return; }
  int speed = _srv.hasArg("speed") ? _srv.arg("speed").toInt() : 50;
  const char* err = "";
  float revs = _srv.arg("revs").toFloat();
  bool axisY = _srv.arg("axis") == "y";
  bool ok = axisY ? _m.turnY(revs, speed, &err) : _m.turnX(revs, speed, &err);
  if (ok) sendJson("{\"ok\":true}");
  else sendError(err);
}

void WebApi::onHome() {
  const char* err = "";
  if (_m.startHome(&err)) sendJson("{\"ok\":true}");
  else sendError(err);
}

void WebApi::onStop() {
  _m.emergencyStop();
  sendJson("{\"ok\":true}");
}

void WebApi::onLaser() {
  String s = _srv.arg("state");
  if (s != "on" && s != "off") { sendError("state must be on or off"); return; }
  _m.setLaser(s == "on");
  sendJson(String("{\"ok\":true,\"laser\":") + (_m.laser() ? "true" : "false") + "}");
}

void WebApi::onCalibrate() {
  float o = _srv.hasArg("offset") ? _srv.arg("offset").toFloat() : _m.angleOffsetDeg();
  if (_srv.hasArg("auto") && _srv.arg("auto") == "1") {
    // Compensacion automatica: offset = angulo del motor - angulo REAL medido por el AS5600.
    if (!_enc.live()) { sendError("AS5600 no listo (sin iman o no responde)"); return; }
    if (!_enc.hasZero()) { sendError("Fije primero el cero del AS5600 con el laser perpendicular"); return; }
    o = _m.angleDeg() - _enc.relativeDeg();
  }
  _m.setCalibration(o);  // si llega "height" se ignora: la altura es fija
  _lastCalibration = Net::isoNow();
  char buf[120];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"lastCalibration\":\"%s\",\"angleOffset\":%.2f}",
           _lastCalibration.c_str(), _m.angleOffsetDeg());
  sendJson(buf);
}

static const char* stateName(Imu::State s) {
  switch (s) {
    case Imu::State::Running: return "running";
    case Imu::State::Done: return "done";
    case Imu::State::Failed: return "failed";
    default: return "idle";
  }
}

void WebApi::onImuStatus() {
  String out = "{\"imus\":[";
  for (int i = 0; i < 2; i++) {
    const Imu& m = *_imus[i];
    float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
    if (m.liveValid()) m.corrected(a, g);
    const ImuCal& c = m.cal();
    char buf[640];
    snprintf(buf, sizeof(buf),
             "%s{\"id\":\"%s\",\"name\":\"%s\",\"addr\":%u,\"ok\":%s,\"model\":\"%s\","
             "\"calibrated\":%s,\"when\":\"%s\",\"state\":\"%s\",\"progress\":%d,\"error\":\"%s\","
             "\"live\":%s,\"a\":[%.3f,%.3f,%.3f],\"g\":[%.2f,%.2f,%.2f],\"tempC\":%.1f,"
             "\"off\":[%.1f,%.1f,%.1f],\"gbias\":[%.1f,%.1f,%.1f]}",
             i ? "," : "", m.id(), m.name(), m.addr(), m.ok() ? "true" : "false", m.model(),
             c.valid ? "true" : "false", c.when, stateName(m.state()), m.progress(), m.error(),
             m.liveValid() ? "true" : "false", a[0], a[1], a[2], g[0], g[1], g[2], m.tempC(),
             c.off[0], c.off[1], c.off[2], c.gbias[0], c.gbias[1], c.gbias[2]);
    out += buf;
  }
  out += "]}";
  sendJson(out);
}

void WebApi::onEncoderStatus() {
  char buf[300];
  snprintf(buf, sizeof(buf),
           "{\"present\":%s,\"addr\":%u,\"magnet\":\"%s\",\"zone\":\"%s\",\"live\":%s,"
           "\"angle\":%.2f,\"raw\":%u,\"agc\":%u,\"mag\":%u,"
           "\"zeroSet\":%s,\"rel\":%.2f,\"comp\":%.2f}",
           _enc.present() ? "true" : "false", As5600::ADDR, _enc.magnetName(), _enc.zone(),
           _enc.live() ? "true" : "false", _enc.angleDeg(), _enc.raw(), _enc.agc(), _enc.magnitude(),
           _enc.hasZero() ? "true" : "false", _enc.relativeDeg(), _m.angleDeg() - _enc.relativeDeg());
  sendJson(buf);
}

void WebApi::onEncoderZero() {
  if (_m.moving()) { sendError("Espere a que termine el movimiento"); return; }
  if (!_enc.setZeroHere()) { sendError("AS5600 no listo (sin iman o no responde)"); return; }
  sendJson("{\"ok\":true}");
}

void WebApi::onImuCalibrate() {
  String which = _srv.arg("which");
  bool pick[2] = {which == "white" || which == "both", which == "black" || which == "both"};
  if (!pick[0] && !pick[1]) { sendError("which must be white, black or both"); return; }

  // Check everything first so "both" never starts only one of them.
  static char msg[64];
  for (int i = 0; i < 2; i++) {
    const char* err = "";
    if (pick[i] && !_imus[i]->canStart(&err)) {
      snprintf(msg, sizeof(msg), "%s %s", _imus[i]->name(), err);
      sendError(msg);
      return;
    }
  }
  for (int i = 0; i < 2; i++) {
    const char* err = "";
    if (pick[i]) _imus[i]->startCalibration(&err);
  }
  sendJson("{\"ok\":true}");
}

// ---- Calibracion de los finales de carrera ----------------------------------------
void WebApi::onLimitsStart() {
  int n = _srv.hasArg("n") ? _srv.arg("n").toInt() : Cfg::LIMIT_CAL_SAMPLES;
  const char* err = "";
  if (_m.startLimitCal(n, &err)) sendJson("{\"ok\":true}");
  else sendError(err);
}

// Pasos contados en cada tramo (dir +1 = hacia X+, -1 = hacia X-), promedio y mitad del riel.
void WebApi::onLimitsLegs() {
  String s = "{\"ok\":true,\"mmPerStep\":";
  s += String(Cfg::X_MM_PER_STEP, 4);
  s += ",\"avg\":";
  s += String(_m.avgSteps());
  s += ",\"middle\":";
  s += String(_m.middleSteps());
  s += ",\"testing\":";
  s += _m.testing() ? "true" : "false";
  s += ",\"legs\":[";
  for (int i = 0; i < _m.legCount(); i++) {
    if (i) s += ",";
    s += "{\"dir\":";
    s += String(_m.legDir(i));
    s += ",\"steps\":";
    s += String(_m.legSteps(i));
    s += "}";
  }
  s += "]}";
  sendJson(s);
}
