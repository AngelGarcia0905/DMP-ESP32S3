#include "WebApi.h"

#include "Net.h"
#include "dmp_config.h"

// El panel, embebido en la flash desde web/index.html (board_build.embed_txtfiles en platformio.ini).
extern const char indexHtmlStart[] asm("_binary_web_index_html_start");
extern const char indexHtmlEnd[] asm("_binary_web_index_html_end");

static const char* tf(bool b) { return b ? "true" : "false"; }

static const char* imuState(const Imu& m) {
  switch (m.state()) {
    case Imu::State::Running: return "running";
    case Imu::State::Failed: return "failed";
    default: return m.cal().valid ? "calibrated" : "idle";
  }
}

void WebApi::begin() {
  _srv.on("/", HTTP_GET, [this] { onRoot(); });
  _srv.on("/index.html", HTTP_GET, [this] { onRoot(); });
  _srv.on("/status", HTTP_GET, [this] { onStatus(); });

  _srv.on("/move", HTTP_GET, [this] {
    float x, a;
    const char* err = "";
    if (argF("x", &x) && argF("angle", &a)) reply(_m.moveTo(x, a, 100, &err, 'W'), err);
  });
  _srv.on("/jog", HTTP_GET, [this] {
    float x, a;
    const char* err = "";
    if (argF("x", &x) && argF("angle", &a)) reply(_m.jogTo(x, a, &err, 'W'), err);
  });
  _srv.on("/sheet", HTTP_GET, [this] {
    float h, v;
    const char* err = "";
    if (argF("h", &h) && argF("v", &v)) reply(_m.moveToSheet(h, v, &err, 'W'), err);
  });
  _srv.on("/home", HTTP_GET, [this] {
    const String a = _srv.arg("axis");
    const char* err = "";
    if (a == "x") reply(_m.startHomeX(&err), err);
    else if (a == "y") reply(_m.startHomeY(&err), err);
    else if (a == "both" || a == "") reply(_m.startHome(&err), err);
    else sendError("axis must be both, x or y");
  });
  _srv.on("/home/mark", HTTP_GET, [this] {  // la posicion actual pasa a ser el HOME (X = 0 y/o Y = 0)
    const String a = _srv.arg("axis");
    const bool x = (a == "x" || a == "both" || a == ""), y = (a == "y" || a == "both" || a == "");
    if (!x && !y) { sendError("axis must be both, x or y"); return; }
    const char* err = "";
    reply(_m.markHome(x, y, &err), err);
  });
  _srv.on("/refx", HTTP_GET, [this] {  // REFERENCIAR X con los finales de carrera
    const char* err = "";
    reply(_m.startRefX(&err), err);
  });
  _srv.on("/cancel", HTTP_GET, [this] {
    _m.cancelHome();
    sendOk();
  });
  _srv.on("/stop", HTTP_GET, [this] {
    _m.emergencyStop();
    sendOk();
  });
  _srv.on("/laser", HTTP_GET, [this] {
    const String s = _srv.arg("state");
    if (s != "on" && s != "off") { sendError("state must be on or off"); return; }
    _m.setLaser(s == "on");
    sendOk();
  });
  _srv.on("/imu/calibrate", HTTP_GET, [this] {  // las IMUs conectadas, como en la pantalla
    const char* err = "";
    int started = 0;
    for (Imu* m : _imus)
      if (m->canStart(&err) && m->startCalibration(&err)) started++;
    if (started) sendOk(); else sendError("IMUs no conectadas u ocupadas");
  });
  _srv.on("/limits/start", HTTP_GET, [this] {
    const int n = _srv.hasArg("n") ? _srv.arg("n").toInt() : Cfg::LIMIT_CAL_SAMPLES;
    if (n < 1 || n > Cfg::LIMIT_CAL_MAX_RUNS) { sendError("n fuera de rango"); return; }
    const char* err = "";
    reply(_m.startLimitCal(n, &err), err);
  });
  _srv.on("/grid/begin", HTTP_GET, [this] {
    const char* err = "";
    reply(_m.beginGrid(_srv.arg("n").toInt(), &err), err);
  });
  _srv.on("/grid/mark", HTTP_GET, [this] {
    if (!_srv.hasArg("i")) { sendError("Missing i"); return; }
    const char* err = "";
    reply(_m.markPoint(_srv.arg("i").toInt(), &err), err);
  });
  _srv.on("/grid/commit", HTTP_GET, [this] {
    _m.commitGrid();
    sendOk();
  });
  _srv.on("/tope/set", HTTP_GET, [this] {  // i: 0 X min, 1 X max, 2 Y min, 3 Y max (la posicion actual)
    if (!_srv.hasArg("i")) { sendError("Missing i"); return; }
    const char* err = "";
    reply(_m.setTope(_srv.arg("i").toInt(), &err), err);
  });
  _srv.on("/tope/clear", HTTP_GET, [this] {
    _m.clearTopes();
    sendOk();
  });
  _srv.on("/speed", HTTP_GET, [this] {
    const int x = _srv.hasArg("x") ? _srv.arg("x").toInt() : _m.speedX();
    const int y = _srv.hasArg("y") ? _srv.arg("y").toInt() : _m.speedY();
    _m.setSpeeds(x, y);  // recorta al rango de la pantalla
    sendOk();
  });
  _srv.on("/ap", HTTP_GET, [this] {
    const String s = _srv.arg("state");
    if (s != "on" && s != "off") { sendError("state must be on or off"); return; }
    sendOk();  // se responde antes: al apagarlo se corta la conexion
    delay(50);
    Net::setEnabled(s == "on");
  });

  // CORS preflight y portal cautivo: las pruebas de conexion del telefono/PC piden otro host y se mandan al panel.
  _srv.onNotFound([this] {
    if (_srv.method() == HTTP_OPTIONS) {
      _srv.sendHeader("Access-Control-Allow-Origin", "*");
      _srv.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
      _srv.sendHeader("Access-Control-Allow-Headers", "*");
      _srv.sendHeader("Access-Control-Allow-Private-Network", "true");
      _srv.send(204);
    } else if (_srv.hostHeader() != Net::ip() && _srv.hostHeader() != String(Cfg::HOSTNAME) + ".local") {
      _srv.sendHeader("Location", String("http://") + Net::ip() + "/", true);
      _srv.sendHeader("Cache-Control", "no-store");
      _srv.send(302, "text/plain", "");
    } else {
      sendError("Unknown endpoint");
    }
  });
  // El servidor NO se arranca aqui: necesita el Wi-Fi encendido (con el punto de acceso apagado, arrancarlo reinicia la
  // placa en bucle). handle() lo arranca cuando el punto de acceso esta encendido y lo cierra cuando se apaga.
}

void WebApi::handle() {
  const bool want = Net::enabled();
  if (want && !_started) {
    _srv.begin();
    _started = true;
  } else if (!want && _started) {
    _srv.close();
    _started = false;
  }
  if (_started) _srv.handleClient();
}

void WebApi::sendJson(const String& body) {
  _srv.sendHeader("Access-Control-Allow-Origin", "*");
  _srv.sendHeader("Cache-Control", "no-store");
  _srv.send(200, "application/json", body);
}

void WebApi::sendError(const char* msg) { sendJson(String("{\"ok\":false,\"error\":\"") + msg + "\"}"); }

bool WebApi::argF(const char* name, float* v) {
  if (!_srv.hasArg(name)) {
    sendError((String("Missing ") + name).c_str());
    return false;
  }
  *v = _srv.arg(name).toFloat();
  return true;
}

void WebApi::onRoot() {
  // El archivo embebido trae un NUL al final que no se manda.
  _srv.sendHeader("Cache-Control", "no-cache");
  _srv.send_P(200, "text/html; charset=utf-8", indexHtmlStart, indexHtmlEnd - indexHtmlStart - 1);
}

// Todo el estado en un JSON (el panel lo pide cada ~0.4 s). Mismos datos que muestran MOVER, HOME, CALIBRAR y SENSORES.
void WebApi::onStatus() {
  char b[700];
  String s;
  s.reserve(2048);

  snprintf(b, sizeof(b),
           "{\"ok\":true,\"x\":%.3f,\"angle\":%.3f,\"tx\":%.3f,\"ty\":%.3f,\"seq\":%lu,\"src\":\"%c\","
           "\"moving\":%s,\"homing\":%s,\"testing\":%s,\"phase\":\"%s\",\"error\":\"%s\",\"laser\":%s,"
           "\"xHomeSet\":%s,\"xHomed\":%s,\"yRef\":%s,\"xMin\":%.3f,\"xMax\":%.3f,\"yMin\":%.3f,\"yMax\":%.3f,",
           _m.xCm(), _m.angleDeg(), _m.targetXCm(), _m.targetAngleDeg(), (unsigned long)_m.seq(), _m.lastSource(),
           tf(_m.moving()), tf(_m.homing()), tf(_m.testing()), _m.homePhase(), _m.lastError(), tf(_m.laser()),
           tf(_m.xHomeSet()), tf(_m.xHomed()), tf(_m.yZeroed()), _m.xMinCm(), _m.xMaxCm(), _m.yMinDeg(), _m.yMaxDeg());
  s += b;

  snprintf(b, sizeof(b),
           "\"xMmStep\":%.6f,\"yDegStep\":%.6f,\"xRed\":%.1f,\"yRed\":%.1f,\"stepsRev\":%.0f,"
           "\"xRel\":%ld,\"xSteps\":%ld,\"ySteps\":%ld,\"angleSign\":%.0f,\"totX\":%lu,\"totY\":%lu,\"xInv\":%s,"
           "\"spsX\":%d,\"spsY\":%d,\"spsMin\":%d,\"spsMax\":%d,\"spsStep\":%d,"
           "\"rail\":%ld,\"avg\":%ld,\"runsMax\":%d,\"limMin\":%s,\"limMax\":%s,",
           Cfg::X_MM_PER_STEP, Cfg::Y_DEG_PER_STEP, Cfg::X_REDUCTION, Cfg::Y_REDUCTION, Cfg::STEPS_PER_REV,
           _m.xStepsRel(), _m.xSteps(), _m.ySteps(), Cfg::ANGLE_SIGN, (unsigned long)_m.stepsX(), (unsigned long)_m.stepsY(),
           tf(_m.xInvertEffective()), _m.speedX(), _m.speedY(), Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX, Cfg::SPS_UI_STEP,
           _m.savedRailSteps(), _m.avgSteps(), Cfg::LIMIT_CAL_MAX_RUNS, tf(_m.limitXMin()), tf(_m.limitXMax()));
  s += b;

  s += "\"legs\":[";
  for (int i = 0; i < _m.legCount(); i++) {
    if (i) s += ",";
    s += String(_m.legSteps(i));
  }
  s += "],";

  s += "\"topes\":[";  // X min, X max (cm), Y min, Y max (grd); null = sin fijar
  for (int i = 0; i < 4; i++) {
    if (i) s += ",";
    s += _m.topeSet(i) ? String(_m.topeValue(i), 3) : String("null");
  }
  s += "],\"limSrc\":";
  s += String(_m.limitSource());  // 0 libre, 1 topes min/max, 2 puntos de la hoja
  s += ",";

  snprintf(b, sizeof(b),
           "\"grid\":%d,\"sheetW\":%.1f,\"sheetH\":%.1f,"
           "\"enc\":{\"present\":%s,\"live\":%s,\"zone\":\"%s\",\"zero\":%s,\"rel\":%.2f},",
           _m.gridPoints(), Cfg::SHEET_W_MM, Cfg::SHEET_H_MM, tf(_enc.present()), tf(_enc.live()),
           _enc.present() ? (_enc.live() ? _enc.zone() : "sin iman") : "--", tf(_enc.hasZero()),
           (_enc.live() && _enc.hasZero()) ? _enc.relativeDeg() : 0.0f);
  s += b;

  s += "\"imus\":[";
  for (int i = 0; i < 2; i++) {
    const Imu& m = *_imus[i];
    snprintf(b, sizeof(b), "%s{\"name\":\"%s\",\"ok\":%s,\"state\":\"%s\",\"progress\":%d}", i ? "," : "", m.name(),
             tf(m.ok()), imuState(m), m.progress());
    s += b;
  }
  snprintf(b, sizeof(b), "],\"ap\":%s,\"clients\":%d,\"ssid\":\"%s\",\"ip\":\"%s\"}", tf(Net::enabled()),
           Net::clients(), Net::ssid(), Net::ip().c_str());
  s += b;
  sendJson(s);
}
