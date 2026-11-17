#include "Machine.h"

#include "As5600.h"
#include "SheetMap.h"

#ifdef ARDUINO_ARCH_ESP32
#include <Preferences.h>
#endif

namespace Cfg {
float X_MM_PER_STEP = X_MM_PER_STEP_DEFAULT;
float Y_DEG_PER_STEP = Y_DEG_PER_STEP_DEFAULT;
}  // namespace Cfg

static long homeSpanSteps();  // recorrido maximo de busqueda de un final de carrera

// Sentido del motor X guardado en la flash por la calibracion de limits (si hubo que corregirlo); si no, el de la config.
static bool xInvertLoad(bool def) {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (!p.begin("dmp", true)) return def;
  const bool v = p.isKey("xinv") ? p.getBool("xinv", def) : def;
  p.end();
  return v;
#else
  return def;
#endif
}

static void xInvertSave(bool v) {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putBool("xinv", v);
    p.end();
  }
#else
  (void)v;
#endif
}

void Machine::begin() {
  pinMode(Pins::LASER, OUTPUT);
  digitalWrite(Pins::LASER, !Cfg::LASER_ACTIVE);  // laser OFF before anything else
  _laser = false;

  _x.begin();
  _y.begin();
  _x.setInvert(xInvertLoad(Cfg::X_INVERT));  // sentido de X: config, o la correccion que guardo la calibracion de limits
  _y.setBacklash(Cfg::Y_BACKLASH_STEPS);     // holgura de los engranes de Y al cambiar de sentido
  _limMin.begin(Pins::LIMIT_X_MIN, Cfg::LIMIT_ACTIVE, true);
  _limMax.begin(Pins::LIMIT_X_MAX, Cfg::LIMIT_ACTIVE, true);
  loadGrid();
  loadPoints();
  loadSettings();
}

// Velocidad de cada eje y largo del riel guardados en la flash (espacio "dmp": spsx, spsy, rail).
void Machine::loadSettings() {
  int sx = lroundf(Cfg::SPS_MAX), sy = sx;
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", true)) {
    sx = p.getInt("spsx", sx);
    sy = p.getInt("spsy", sy);
    _savedRail = p.getLong("rail", 0);
    _xHomeSet = p.getBool("hxs", false);  // HOME de X marcado en CALIBRAR > HOME
    _xOff = _xHomeSet ? p.getLong("hxo", 0) : 0;
    // escalas corregidas desde la pantalla (si estan guardadas y son razonables, mandan sobre las de fabrica)
    const float xmm = p.getFloat("xmm", 0.0f), ydg = p.getFloat("ydg", 0.0f);
    if (xmm > 0.0001f && xmm < 0.05f) Cfg::X_MM_PER_STEP = xmm;
    if (ydg > 0.001f && ydg < 0.5f) Cfg::Y_DEG_PER_STEP = ydg;
    p.end();
  }
#endif
  _spsX = constrain(sx, Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX);
  _spsY = constrain(sy, Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX);
  _x.setSpeed(_spsX);
  _y.setSpeed(_spsY);
  if (_savedRail > 0) _railSteps = _savedRail;  // largo conocido; X sigue sin referencia hasta el HOME
}

void Machine::setSpeeds(int spsX, int spsY) {
  _spsX = constrain(spsX, Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX);
  _spsY = constrain(spsY, Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX);
  _x.setSpeed(_spsX);
  _y.setSpeed(_spsY);
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putInt("spsx", _spsX);
    p.putInt("spsy", _spsY);
    p.end();
  }
#endif
}

// Corrige las escalas con lo que se midio (0 = no tocar ese eje) y las guarda. Devuelve false si el valor no es razonable.
// Solo cambian los mm y grados que se muestran y se piden: todo lo guardado (topes, puntos, riel) esta en pasos.
bool Machine::setScales(float xMmPerStep, float yDegPerStep) {
  if ((xMmPerStep != 0 && (xMmPerStep < 0.0001f || xMmPerStep > 0.05f)) ||
      (yDegPerStep != 0 && (yDegPerStep < 0.001f || yDegPerStep > 0.5f)))
    return false;
  if (xMmPerStep > 0) Cfg::X_MM_PER_STEP = xMmPerStep;
  if (yDegPerStep > 0) Cfg::Y_DEG_PER_STEP = yDegPerStep;
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putFloat("xmm", Cfg::X_MM_PER_STEP);
    p.putFloat("ydg", Cfg::Y_DEG_PER_STEP);
    p.end();
  }
#endif
  _src = 'M';
  _seq = _seq + 1;  // los valores en pantalla se vuelven a leer
  return true;
}

// Vuelve a las escalas de fabrica (las de dmp_config.h) y borra las guardadas.
void Machine::resetScales() {
  Cfg::X_MM_PER_STEP = Cfg::X_MM_PER_STEP_DEFAULT;
  Cfg::Y_DEG_PER_STEP = Cfg::Y_DEG_PER_STEP_DEFAULT;
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.remove("xmm");
    p.remove("ydg");
    p.end();
  }
#endif
  _src = 'M';
  _seq = _seq + 1;
}

void Machine::saveRail(long steps) {
  _savedRail = steps;
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putLong("rail", steps);
    p.end();
  }
#endif
}

// ---- Calibracion de la hoja (espacio "dmp" de la flash: gpts, gx0..gx8, gy0..gy8) ----
static bool validGridN(int n) { return n == 4 || n == 6 || n == 9; }

const char* Machine::pointName(int n, int i) {
  static const char* const k4[4] = {"SUP IZQ", "SUP DER", "INF IZQ", "INF DER"};
  static const char* const k6[6] = {"SUP IZQ", "SUP CENTRO", "SUP DER", "INF IZQ", "INF CENTRO", "INF DER"};
  static const char* const k9[9] = {"SUP IZQ", "SUP CENTRO", "SUP DER", "CENTRO IZQ", "CENTRO",
                                    "CENTRO DER", "INF IZQ", "INF CENTRO", "INF DER"};
  if (i < 0 || i >= n) return "";
  return n == 4 ? k4[i] : (n == 6 ? k6[i] : k9[i]);
}

void Machine::pointSheetMm(int n, int i, float* h, float* v) {
  const int nc = gridCols(n), nr = gridRows(n), r = i / nc, c = i % nc;
  *h = -Cfg::SHEET_W_MM / 2 + c * Cfg::SHEET_W_MM / (nc - 1);
  *v = Cfg::SHEET_H_MM / 2 - r * Cfg::SHEET_H_MM / (nr - 1);  // fila 0 = arriba
}

void Machine::loadGrid() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (!p.begin("dmp", true)) return;
  const int n = p.getUChar("gpts", 0);
  if (validGridN(n)) {
    char k[4] = "gx0";
    for (int i = 0; i < n; i++) {
      k[2] = '0' + i;
      k[1] = 'x'; _gx[i] = p.getLong(k, 0);
      k[1] = 'y'; _gy[i] = p.getLong(k, 0);
    }
    _gN = n;
  }
  _tMask = p.getUChar("tmask", 0) & 0x0F;  // topes de MOVER
  _tx[0] = p.getLong("tx0", 0);
  _tx[1] = p.getLong("tx1", 0);
  _ty[0] = p.getLong("ty0", 0);
  _ty[1] = p.getLong("ty1", 0);
  p.end();
#endif
}

// ---- Topes de MOVER (CALIBRAR > TOPES > X/Y MIN Y MAX) ----
bool Machine::setTope(int i, const char** err) {
  if (i < 0 || i > 3) { *err = "Tope invalido"; return false; }
  if (homing()) { *err = "HOME en curso"; return false; }
  if (moving()) { *err = "Motor en movimiento"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }  // sin referencia, el tope no serviria al reiniciar
  if (!_yZeroed) { *err = "Y sin referencia"; return false; }
  if (i < 2) _tx[i] = _x.position();
  else _ty[i - 2] = lroundf(Cfg::ANGLE_SIGN * _y.position());
  _tMask |= (1 << i);
  saveTopes();
  return true;
}

// ---- Puntos guardados (modulo PUNTOS) ----
void Machine::loadPoints() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (!p.begin("dmp", true)) return;
  _sMask = p.getUChar("smask", 0);
  char k[4] = "sx0";
  for (int i = 0; i < kMaxSaved; i++) {
    k[2] = '0' + i;
    k[1] = 'x'; _sx[i] = p.getLong(k, 0);
    k[1] = 'y'; _sy[i] = p.getLong(k, 0);
  }
  p.end();
#endif
}

void Machine::savePoints() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    char k[4] = "sx0";
    for (int i = 0; i < kMaxSaved; i++) {
      k[2] = '0' + i;
      k[1] = 'x'; p.putLong(k, _sx[i]);
      k[1] = 'y'; p.putLong(k, _sy[i]);
    }
    p.putUChar("smask", _sMask);
    p.end();
  }
#endif
}

bool Machine::savePoint(int i, const char** err) {
  if (i < 0 || i >= kMaxSaved) { *err = "Punto invalido"; return false; }
  if (homing()) { *err = "HOME en curso"; return false; }
  if (moving()) { *err = "Motor en movimiento"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }  // sin referencia, el punto no serviria al reiniciar
  if (!_yZeroed) { *err = "Y sin referencia"; return false; }
  _sx[i] = _x.position();
  _sy[i] = lroundf(Cfg::ANGLE_SIGN * _y.position());
  _sMask |= (1 << i);
  savePoints();
  return true;
}

bool Machine::gotoPoint(int i, const char** err, char src) {
  if (!pointSaved(i)) { *err = "Punto libre"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }
  if (!_yZeroed) { *err = "Y sin referencia"; return false; }
  return moveCore((_sx[i] - _xOff) * Cfg::X_MM_PER_STEP / 10.0f, _sy[i] * Cfg::Y_DEG_PER_STEP, 100, err, src, true);
}

void Machine::clearPoint(int i) {
  if (i < 0 || i >= kMaxSaved) return;
  _sMask &= ~(1 << i);
  savePoints();
}

void Machine::mapRange(long* x0, long* x1, long* y0, long* y1) const {
  if (topesReady()) {
    *x0 = min(_tx[0], _tx[1]); *x1 = max(_tx[0], _tx[1]);
    *y0 = min(_ty[0], _ty[1]); *y1 = max(_ty[0], _ty[1]);
  } else if (_gN) {
    *x0 = gridMin(_gx); *x1 = gridMax(_gx);
    *y0 = gridMin(_gy); *y1 = gridMax(_gy);
  } else {
    *x0 = 0;
    *x1 = _savedRail > 0 ? _savedRail : lroundf(300.0f / Cfg::X_MM_PER_STEP);
    const long ys = lroundf(35.0f / Cfg::Y_DEG_PER_STEP);
    *y0 = -ys;
    *y1 = ys;
  }
  if (*x1 <= *x0) *x1 = *x0 + 1;
  if (*y1 <= *y0) *y1 = *y0 + 1;
}

void Machine::clearTopes() {
  _tMask = 0;
  saveTopes();
}

void Machine::saveTopes() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putUChar("tmask", _tMask);
    p.putLong("tx0", _tx[0]);
    p.putLong("tx1", _tx[1]);
    p.putLong("ty0", _ty[0]);
    p.putLong("ty1", _ty[1]);
    p.end();
  }
#endif
}

float Machine::topeValue(int i) const {  // X desde el HOME marcado (como se muestra)
  return i < 2 ? (_tx[i] - _xOff) * Cfg::X_MM_PER_STEP / 10.0f : _ty[i - 2] * Cfg::Y_DEG_PER_STEP;
}

// Limites de MOVER: topes (si estan los 4), si no el rectangulo de los puntos de la hoja, si no libre. min/max se
// ordenan por si se marcaron al reves. X se devuelve desde el HOME marcado (como se muestra).
float Machine::xMinCm() const {
  if (!limitX()) return -Cfg::X_FREE_CM;
  const long v = topesReady() ? min(_tx[0], _tx[1]) : gridMin(_gx);
  return (v - _xOff) * Cfg::X_MM_PER_STEP / 10.0f;
}

float Machine::xMaxCm() const {
  if (!limitX()) return Cfg::X_FREE_CM;
  const long v = topesReady() ? max(_tx[0], _tx[1]) : gridMax(_gx);
  return (v - _xOff) * Cfg::X_MM_PER_STEP / 10.0f;
}

float Machine::yMinDeg() const {
  if (!limitY()) return Cfg::Y_MIN_DEG;
  return (topesReady() ? min(_ty[0], _ty[1]) : gridMin(_gy)) * Cfg::Y_DEG_PER_STEP;
}

float Machine::yMaxDeg() const {
  if (!limitY()) return Cfg::Y_MAX_DEG;
  return (topesReady() ? max(_ty[0], _ty[1]) : gridMax(_gy)) * Cfg::Y_DEG_PER_STEP;
}

bool Machine::beginGrid(int n, const char** err) {
  if (!validGridN(n)) { *err = "Puntos: 4, 6 o 9"; return false; }
  if (homing()) { *err = "HOME en curso"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }  // sin referencia, la X guardada no serviria al reiniciar
  if (!_yZeroed) { *err = "Y sin referencia"; return false; }
  _pN = n;
  return true;
}

bool Machine::markPoint(int i, const char** err) {
  if (i < 0 || i >= _pN) { *err = "Punto invalido"; return false; }
  if (homing()) { *err = "HOME en curso"; return false; }
  if (moving()) { *err = "Motor en movimiento"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }
  if (!_yZeroed) { *err = "Y sin referencia"; return false; }
  _px[i] = _x.position();
  _py[i] = lroundf(Cfg::ANGLE_SIGN * _y.position());
  return true;
}

void Machine::commitGrid() {
  if (!validGridN(_pN)) return;
  for (int i = 0; i < _pN; i++) { _gx[i] = _px[i]; _gy[i] = _py[i]; }
  _gN = _pN;
  saveGrid();
}

void Machine::saveGrid() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    char k[4] = "gx0";
    for (int i = 0; i < _gN; i++) {
      k[2] = '0' + i;
      k[1] = 'x'; p.putLong(k, _gx[i]);
      k[1] = 'y'; p.putLong(k, _gy[i]);
    }
    p.putUChar("gpts", _gN);
    p.end();
  }
#endif
}

// Punto de la hoja (mm) -> objetivo de los motores. H y V se miden desde los HOME marcados (CALIBRAR > HOME):
//   V -> motor X (carro): V = 0 es el HOME de X; X = V en mm reales, en el sentido de la hoja marcada.
//   H -> motor Y (pivote): H = 0 es el HOME de Y; H en mm se convierte con la tabla medida de SheetMap.h.
// Con H = 0 y V = 0 los ejes van exactamente al HOME, este donde este (no al centro del riel ni de la hoja). De la hoja
// marcada (4, 6 o 9 puntos) solo se usa hacia donde crece cada eje. Lo que cae fuera de los limites de MOVER (topes, o
// los puntos de la hoja si no hay topes) se RECHAZA, no se recorta.
bool Machine::moveToSheet(float hMm, float vMm, const char** err, char src) {
  if (!_gN) { *err = "Hoja sin calibrar"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }
  if (!_yZeroed) { *err = "Y sin referencia"; return false; }
  if (isnan(hMm) || isnan(vMm)) { *err = "H o V invalido"; return false; }
  int dirX, sgnY;
  SheetMap::gridDirs(_gx, _gy, gridCols(_gN), gridRows(_gN), &dirX, &sgnY);
  const SheetMap::Target t = SheetMap::hvFromHome(hMm, vMm, dirX, sgnY);
  const char* bad = SheetMap::checkRange(t, xMinCm() * 10.0f, xMaxCm() * 10.0f, yMinDeg(), yMaxDeg(),
                                         Cfg::X_MM_PER_STEP / 2.0f, Cfg::Y_DEG_PER_STEP * 0.5f);
  if (bad) { *err = bad; return false; }
  return moveCore(t.xMm / 10.0f, t.yDeg, 100, err, src, true);
}

// ---- HOME marcado (CALIBRAR > HOME) ----
// X: el HOME se guarda en pasos desde X- (_xOff) y desde ahi se cuenta X = 0; lo guardado en pasos (topes, puntos, largo
// del riel) no cambia. Y: el angulo actual pasa a ser el cero del AS5600 y el contador de Y = 0; los topes y puntos de
// Y (guardados en pasos del angulo) se recorren lo mismo para que sigan en el mismo lugar fisico.
// HOME de X: la posicion (en pasos desde el final de carrera X-) donde X = 0. Se guarda en la flash.
void Machine::setXHome(long steps) {
  _xOff = steps;
  _xHomeSet = true;
#ifdef ARDUINO_ARCH_ESP32
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putLong("hxo", _xOff);
    p.putBool("hxs", true);
    p.end();
  }
#endif
}

// A donde va X al terminar de referenciarse: el HOME de X (sin marcar es el final X-), dentro del riel.
long Machine::homeDest(long rail) const { return constrain(_xOff, 0L, rail > 0 ? rail : _xOff); }

bool Machine::markHome(bool x, bool y, const char** err) {
  if (homing()) { *err = "HOME en curso"; return false; }
  if (moving()) { *err = "Motor en movimiento"; return false; }
  if (x && !_xHomed) { *err = "Calibra X primero"; return false; }  // sin referencia no serviria al reiniciar
  if (y && !(_enc && _enc->live())) { *err = "AS5600 no listo"; return false; }
  if (x) setXHome(_x.position());
  if (y) {
    // Cuanto se mueve el cero (en pasos del angulo mostrado): lo que marcaba el sensor, o el contador si no tenia cero.
    const long shift = _enc->hasZero() ? lroundf(_enc->relativeDeg() / Cfg::Y_DEG_PER_STEP)
                                       : lroundf(Cfg::ANGLE_SIGN * _y.position());
    if (!_enc->setZeroHere()) { *err = "No se pudo fijar"; return false; }
    for (int i = 0; i < _gN; i++) _gy[i] -= shift;
    for (int i = 0; i < 2; i++) _ty[i] -= shift;
    for (int i = 0; i < kMaxSaved; i++) _sy[i] -= shift;
    if (_gN) saveGrid();
    if (_tMask) saveTopes();
    if (_sMask) savePoints();
    _y.setPosition(0);
    _yZeroed = true;
  }
  _calibrated = _xHomed && _yZeroed;
  _error = "";
  _src = 'M';
  _seq = _seq + 1;
  return true;
}


void Machine::setYFromDeg(float deg) {
  if (homing() || _y.isRunning() || isnan(deg)) return;
  _y.setPosition(lroundf(Cfg::ANGLE_SIGN * deg / Cfg::Y_DEG_PER_STEP));
  _yZeroed = true;
  _calibrated = _xHomed && _yZeroed;
  _src = 'M';
  _seq = _seq + 1;
}

void Machine::update() {
  _limMin.update();
  _limMax.update();

  if (_ts != TestState::None) {  // HOME o calibracion de limits (los dos van por lado)
    updateLimitCal();
  } else {
    guardLimits();
  }
  if (_yh != YHome::None) updateHomeY();  // HOME de Y (al cero del AS5600), a la vez que el de X si es HOME completo

  // Los pasos los da el temporizador de hardware de cada motor (Stepper28); aqui solo se apagan las bobinas en reposo.
  _x.service();
  _y.service();
}

// Cada 1 ms: finales de carrera, HOME / calibracion y seguridad. (Ya no limita la velocidad de los motores.)
static void motionTask(void* arg) {
  Machine* m = static_cast<Machine*>(arg);
  for (;;) {
    m->update();
    vTaskDelay(1);
  }
}

void Machine::startMotionTask() {
  xTaskCreatePinnedToCore(motionTask, "motion", 4096, this, 3, nullptr, 1);
}

bool Machine::moveTo(float xCm, float angleDeg, int speedPct, const char** err, char src) {
  return moveCore(xCm, angleDeg, speedPct, err, src, true);
}

bool Machine::jogTo(float xCm, float angleDeg, const char** err, char src) {
  return moveCore(xCm, angleDeg, 100, err, src, false);
}

bool Machine::moveCore(float xCm, float angleDeg, int speedPct, const char** err, char src, bool checkRange) {
  if (homing()) { *err = "Homing in progress"; return false; }
  if (!canMove()) { *err = "Haga HOME primero"; return false; }  // sin referencia de X no se mueve nada
  if (isnan(xCm) || isnan(angleDeg)) { *err = "Invalid coordinates"; return false; }
  if (fabsf(xCm) > Cfg::X_FREE_CM) { *err = "X out of range"; return false; }  // tope tecnico (los pasos no desbordan)
  if (checkRange) {
    // TOPES de MOVER (sin topes: solo los tecnicos); margen de medio paso por redondeo
    const float hx = Cfg::X_MM_PER_STEP / 20.0f, hy = Cfg::Y_DEG_PER_STEP * 0.5f;
    if (xCm < xMinCm() - hx || xCm > xMaxCm() + hx) { *err = "X out of range"; return false; }
    if (angleDeg < yMinDeg() - hy || angleDeg > yMaxDeg() + hy) { *err = "Angle out of range"; return false; }
  }

  (void)speedPct;  // la velocidad es la de cada eje (CALIBRAR > VELOCIDAD); el porcentaje se ignora
  _error = "";
  _x.moveTo(SheetMap::xStepsFromHome(xCm * 10.0f, Cfg::X_MM_PER_STEP, _xOff));  // X se cuenta desde el HOME marcado
  _y.moveTo(lroundf(Cfg::ANGLE_SIGN * angleDeg / Cfg::Y_DEG_PER_STEP));
  _src = src;
  _seq = _seq + 1;
  return true;
}

bool Machine::turnX(float revs, int speedPct, const char** err) {
  if (!canMove()) { *err = "Haga HOME primero"; return false; }
  return turnAxis(_x, "X", revs, speedPct, err);
}

bool Machine::turnY(float revs, int speedPct, const char** err) {
  return turnAxis(_y, "Y", Cfg::ANGLE_SIGN * revs, speedPct, err);  // +Y sigue el mismo sentido que los angulos positivos
}

bool Machine::turnAxis(Stepper28& s, const char* name, float revs, int speedPct, const char** err) {
  static char busy[24];
  if (homing()) { *err = "Homing in progress"; return false; }
  if (isnan(revs) || revs == 0.0f || fabsf(revs) > 5.0f) { *err = "revs must be 0.01 to 5 (negative = reverse)"; return false; }
  if (s.isRunning()) {
    snprintf(busy, sizeof(busy), "%s is already moving", name);
    *err = busy;
    return false;
  }

  (void)speedPct;
  _error = "";
  s.moveBy(lroundf(revs * Cfg::STEPS_PER_REV));
  return true;
}

// Paro: detiene los motores. NO apaga el laser (el laser solo se apaga con su boton).
void Machine::emergencyStop() {
  _ts = TestState::None;
  _x.stop();
  _y.stop();
  if (_yh != YHome::None) _y.setSpeed(_spsY);  // el HOME de Y iba a velocidad baja
  _yh = YHome::None;
  _x.release();
  _y.release();
  _error = "Stopped";
  _src = 'M';
  _seq = _seq + 1;
}

void Machine::setLaser(bool on) {
  _laser = on;
  digitalWrite(Pins::LASER, on ? Cfg::LASER_ACTIVE : !Cfg::LASER_ACTIVE);
}

void Machine::setCalibration(float offsetDeg) {
  _angleOffset = offsetDeg;
}

// ---- Safety: never push into a pressed limit switch --------------------------

void Machine::guardLimits() {
  if (!Cfg::LIMIT_SWITCHES_STOP_MOTION) return;  // movimiento libre: los interruptores no cortan el movimiento
  if (Cfg::LIMITS_FREE_UNTIL_CALIBRATED && !_xHomed) return;  // X sin calibrar: libre (los interruptores aun no protegen)
  if (_limMin.active() && _x.target() < _x.position()) {
    _x.stop();
    _error = "X- limit switch";
  }
  if (_limMax.active() && _x.target() > _x.position()) {
    _x.stop();
    _error = "X+ limit switch";
  }
}

// ---- Homing ------------------------------------------------------------------
// HOME = la posicion marcada en CALIBRAR > HOME (X = 0 y Y = 0 ahi).
// X: va a X = 0 (el HOME marcado; sin marcar, el final de carrera X-). Necesita X referenciado; si no, avisa y no se
// mueve. REFERENCIAR X (startRefX): POR LADO, igual que la calibracion de limits (una corrida; rapido si el largo del
// riel esta guardado): toca los finales de carrera y deja X- como referencia; termina en el final de carrera X-.
// Y: va al CERO DEL AS5600 (= el HOME marcado) por el LADO MAS CORTO: el AS5600 da el angulo real en -180..+180, asi
// que el giro nunca pasa de media vuelta (no da la vuelta completa: riesgo de choque). Al llegar vuelve a leer el
// sensor y corrige (hasta kYHomeTries intentos); al final el contador de Y queda igual al sensor. Sin AS5600 listo
// (conectado, con iman y con cero) no se mueve y avisa.

// HOME de los dos a la vez: si falta X referenciado o el AS5600, no se mueve ninguno.
bool Machine::startHome(const char** err) {
  if (homing()) { *err = "Homing already running"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }
  if (!encoderReady()) { *err = "AS5600 no listo"; return false; }
  if (!startHomeX(err)) return false;
  beginHomeY();
  return true;
}

bool Machine::startHomeX(const char** err) {
  if (homing()) { *err = "Homing already running"; return false; }
  if (!_xHomed) { *err = "Calibra X primero"; return false; }
  _error = "";
  _x.moveTo(_xOff);  // X = 0 (sin limites: es el HOME)
  _src = 'M';
  _seq = _seq + 1;
  return true;
}

bool Machine::startRefX(const char** err) { return beginHome(err); }

bool Machine::startHomeY(const char** err) {
  if (homing()) { *err = "Homing already running"; return false; }
  if (!encoderReady()) { *err = "AS5600 no listo"; return false; }
  _error = "";
  beginHomeY();
  _src = 'M';
  _seq = _seq + 1;
  return true;
}

bool Machine::encoderReady() const { return _enc && _enc->live() && _enc->hasZero(); }

// Angulo real del laser respecto al cero del AS5600 (grados mostrados, -180..+180).
float Machine::encoderDeg() const { return _enc->relativeDeg(); }

// ---- HOME de Y en LAZO CERRADO: el AS5600 es el juez ------------------------------------------------------------------
// No se confia en el signo (ENC_INVERT / ANGLE_SIGN) ni en los grados por paso de la configuracion:
//  1. PRUEBA: un movimiento corto (kYProbeSteps, ~2.6 grados) hacia el lado que se SUPONE que reduce el error. Si el error
//     del sensor CRECE, el sentido estaba al reves y se invierte (_yA); de la prueba sale tambien cuantos grados del
//     sensor mueve cada paso (_yK). Si casi no se mueve, una segunda prueba mas larga; si tampoco, avisa y no sigue.
//  2. RECORRIDO: el resto, con el sentido y la escala ya medidos, a velocidad baja (kYHomeSps), mirando el sensor cada
//     25 ms: si cruza el cero se detiene al instante, y si el error CRECE mas de kYWorseDeg se detiene y avisa.
//  3. AJUSTE: al parar espera la lectura y corrige hasta quedar a menos de kYHomeTolDeg (hasta kYHomeTries veces).
// El error del sensor esta siempre en -180..+180, asi que el recorrido nunca pasa de media vuelta. Cada paso sale en el
// monitor serie con el prefijo [homeY].
void Machine::beginHomeY() {
  _y.stop();
  _y.setSpeed(min(_spsY, kYHomeSps));
  _yTries = 0;
  _yProbeTry = 0;
  _yA = +1;
  _yK = 0;
  const float rel = encoderDeg();
  Serial.printf("[homeY] inicio: AS5600 = %+.2f grd del cero (contador %+.2f grd)\n", rel, angleDeg());
  if (fabsf(rel) > kYMaxStartDeg) { failHomeY("Y cerca de 180 grd"); return; }
  if (fabsf(rel) <= kYHomeTolDeg) { finishHomeY(rel); return; }
  startProbe(rel, kYProbeSteps);
}

void Machine::startProbe(float rel, int steps) {
  _yProbeRel = rel;
  _yProbeCmd = steps;
  _y.moveBy((long)_yA * (rel > 0 ? 1 : -1) * steps);
  _yh = YHome::Probe;
  Serial.printf("[homeY] prueba %d: %+d pasos (sentido supuesto %+d)\n", _yProbeTry + 1, (int)(_yA * (rel > 0 ? 1 : -1) * steps),
                (int)_yA);
}

void Machine::evalProbe() {
  const float rel = encoderDeg();
  const float moved = fabsf(rel - _yProbeRel);            // cuanto se movio el sensor (grados)
  const float better = fabsf(_yProbeRel) - fabsf(rel);    // >0: se acerco al cero
  Serial.printf("[homeY] prueba: AS5600 %+.2f -> %+.2f (se movio %.2f grd, %s)\n", _yProbeRel, rel, moved,
                better > 0 ? "se acerco" : "se alejo");
  if (moved < 0.3f) {  // casi no se movio: holgura o motor sin fuerza; una prueba mas larga
    if (++_yProbeTry >= 2) { failHomeY("Y no se mueve"); return; }
    startProbe(rel, kYProbeSteps * 4);
    return;
  }
  if (better < 0) {  // se alejo: el sentido supuesto estaba al reves
    _yA = -_yA;
    Serial.println("[homeY] DIAGNOSTICO: motor y sensor giran en sentidos CONTRARIOS (sentido invertido para esta corrida). "
                   "Falta poner ENC_INVERT = true en dmp_config.h.");
  }
  // grados del sensor por paso; los pasos de la holgura (Y_BACKLASH_STEPS) no mueven nada, asi que no cuentan
  const int eff = max(1, _yProbeCmd - Cfg::Y_BACKLASH_STEPS);
  _yK = moved / eff;
  if (_yK < 0.005f || _yK > 0.2f) {
    Serial.printf("[homeY] escala medida fuera de rango (%.5f grd/paso): uso la de la configuracion (%.5f)\n", _yK,
                  Cfg::Y_DEG_PER_STEP);
    _yK = Cfg::Y_DEG_PER_STEP;
  }
  Serial.printf("[homeY] escala medida: %.5f grd del sensor por paso (configuracion: %.5f)\n", _yK, Cfg::Y_DEG_PER_STEP);
  planRun(rel);
}

// Manda lo que falta segun el sentido y la escala medidos.
void Machine::planRun(float rel) {
  const float a = fabsf(rel);
  _yRunSign = rel > 0 ? +1 : -1;
  _yBest = a;
  const long steps = lroundf(a / _yK);
  Serial.printf("[homeY] recorrido: error %+.2f grd -> %ld pasos\n", rel, (long)_yA * _yRunSign * steps);
  _y.moveBy((long)_yA * _yRunSign * steps);
  _yT = millis();
  _yh = YHome::Run;
}

void Machine::finishHomeY(float rel) {
  _y.stop();
  _y.setSpeed(_spsY);
  _y.setPosition(lroundf(Cfg::ANGLE_SIGN * rel / Cfg::Y_DEG_PER_STEP));  // el contador de Y = el sensor
  _yZeroed = true;
  _calibrated = _xHomed && _yZeroed;
  _yh = YHome::None;
  if (fabsf(rel) > kYHomeTolDeg) _error = "HOME Y: no llego a 0";
  Serial.printf("[homeY] fin: %+.2f grd del cero%s\n", rel, fabsf(rel) > kYHomeTolDeg ? " (NO llego)" : "");
  _src = 'M';
  _seq = _seq + 1;
}

void Machine::failHomeY(const char* msg) {
  _y.stop();
  _y.setSpeed(_spsY);
  _yh = YHome::None;
  _error = msg;
  Serial.printf("[homeY] DETENIDO: %s (AS5600 = %+.2f grd)\n", msg, encoderReady() ? encoderDeg() : 0.0f);
  _src = 'M';
  _seq = _seq + 1;
}

void Machine::updateHomeY() {
  const uint32_t now = millis();
  switch (_yh) {
    case YHome::Probe:
      if (!_y.isRunning()) { _yT = now; _yh = YHome::ProbeWait; }
      break;

    case YHome::ProbeWait:
      if (now - _yT < kYSettleMs) break;  // espera a que el AS5600 tome una lectura nueva (lee cada 100 ms)
      if (!encoderReady()) { failHomeY("AS5600 perdido"); break; }
      evalProbe();
      break;

    case YHome::Run: {
      if (now - _yT >= 25) {  // vigila el sensor
        _yT = now;
        if (!encoderReady()) { failHomeY("AS5600 perdido"); break; }
        const float rel = encoderDeg();
        const float a = fabsf(rel);
        const bool crossed = (rel * _yRunSign < 0) && a < 90.0f;  // paso por el cero (con <90 no es un salto de +-180)
        if (crossed || a <= kYHomeTolDeg) {
          _y.stop();
          Serial.printf("[homeY] llego al cero (AS5600 = %+.2f grd), freno\n", rel);
          _yT = now;
          _yh = YHome::Settle;
          break;
        }
        if (a > _yBest + kYWorseDeg) {  // se aleja del cero: no se sigue
          failHomeY("HOME Y: se aleja");
          break;
        }
        if (a < _yBest) _yBest = a;
      }
      if (_yh == YHome::Run && !_y.isRunning()) { _yT = now; _yh = YHome::Settle; }
      break;
    }

    case YHome::Settle: {
      if (now - _yT < kYSettleMs) break;
      if (!encoderReady()) { failHomeY("AS5600 perdido"); break; }
      const float rel = encoderDeg();
      Serial.printf("[homeY] ajuste %d: AS5600 = %+.2f grd\n", _yTries, rel);
      if (fabsf(rel) <= kYHomeTolDeg || _yTries >= kYHomeTries) { finishHomeY(rel); break; }
      _yTries++;
      planRun(rel);
      break;
    }

    default:
      break;
  }
}

// Cancela un HOME o una calibracion en curso: los motores se paran donde esten. Si ya iba al destino final (X = 0 o el
// medio), X ya tenia referencia y se conserva; antes de eso el 0 de X aun no esta definido.
void Machine::cancelHome() {
  if (!homing()) return;
  const bool xActive = (_ts != TestState::None);
  const bool xRef = xActive ? (_ts == TestState::ToMiddle) : _xHomed;  // si solo corria el HOME de Y, X no cambia
  const bool wasHome = _endZero || !xActive;
  _x.stop();
  _y.stop();
  if (_yh != YHome::None) _y.setSpeed(_spsY);  // el HOME de Y iba a velocidad baja
  _ts = TestState::None;
  _yh = YHome::None;
  _xHomed = xRef;
  _calibrated = _xHomed && _yZeroed;
  _error = wasHome ? "Home cancelled" : "Calibration cancelled";
  _src = 'M';
  _seq = _seq + 1;
}

// REFERENCIAR X con los finales de carrera (por lado; rapido si el largo del riel esta guardado).
bool Machine::beginHome(const char** err) {
  if (homing()) { *err = "Homing already running"; return false; }

  // MODO PRUEBA (sin finales de carrera): el HOME de X se da por hecho al instante, donde este.
  if (Cfg::TEST_NO_LIMIT_SWITCHES) {
    _x.stop();
    _x.setPosition(0);
    _xHomed = true;
    _railSteps = 0;  // sin medir: el maximo de X queda en X_MAX_CM
    _calibrated = _xHomed && _yZeroed;
    _error = "";
    _src = 'M';
    _seq = _seq + 1;
    return true;
  }

  if (!startCal(1, true, err)) return false;
  _calibrated = false;
  return true;
}

static long homeSpanSteps() {
  return 1000000000L;  // SIN limite de distancia: avanza hasta que el interruptor toque (se cancela con STOP / clic / 'C')
}

const char* Machine::homePhase() const {
  static char buf[24];
  switch (_ts) {
    case TestState::Seek:  return _quick ? "buscando X- (rapido)" : "buscando un limite";
    case TestState::Leave: return _from == 0 ? "saliendo de X-" : "saliendo de X+";
    case TestState::Cross:
      if (_endZero) return _from == 0 ? "midiendo hacia X+" : "midiendo hacia X-";
      snprintf(buf, sizeof(buf), _from == 0 ? "hacia X+  %d/%d" : "hacia X-  %d/%d", _legCount + 1, _tSamples);
      return buf;
    case TestState::ToMiddle: return _endZero ? "volviendo a X = 0" : "yendo al medio";
    default:
      switch (_yh) {
        case YHome::Probe:
        case YHome::ProbeWait: return "Y probando sentido";
        case YHome::Run:       return "Y hacia el cero";
        case YHome::Settle:    return "Y ajustando";
        default:               return "";
      }
  }
}

// ---- Calibracion de los finales de carrera -------------------------------------
// Por LADO, no por etiqueta: en cuanto un interruptor toca, el carro se detiene y sale en sentido contrario al que lo
// trajo; el conteo de cada tramo empieza cuando se DESPEGA el interruptor del que sale y termina cuando toca el otro.
// Asi funciona aunque el sentido de los pasos este al reves: si al salir de un interruptor presionado este no se
// despega en LIMIT_DIR_DETECT_STEPS, se invierte el sentido (y se guarda). Sin limite de distancia.
static long dirDetectSteps() { return Cfg::LIMIT_DIR_DETECT_STEPS; }

bool Machine::startLimitCal(int samples, const char** err) { return startCal(samples, false, err); }

// Arranque comun de la calibracion de limits (endZero = false) y del HOME (endZero = true, una corrida, termina en X = 0
// y no toca el promedio ni las muestras de la calibracion).
bool Machine::startCal(int samples, bool endZero, const char** err) {
  if (homing()) { *err = "Homing in progress"; return false; }
  if (_limMin.active() && _limMax.active()) { *err = "Both limit switches active"; return false; }
  if (samples < 1) samples = 1;
  if (samples > kMaxLegs) samples = kMaxLegs;
  _error = "";
  _endZero = endZero;
  if (!endZero) {
    _legCount = 0;
    _avgSteps = 0;
  }
  _tSamples = samples;
  _flipTried = false;
  _dirVerified = false;
  _xFlipped = false;
  _signMinToMax = 0;
  _xHomed = false;
  _x.stop();
  _x.setPosition(0);  // referencia provisional: los tramos se miden por diferencias de posicion
  // HOME RAPIDO: con el largo del riel ya guardado basta tocar UN interruptor (X- = 0, X+ = largo guardado).
  _quick = endZero && _savedRail > 0;
  if (_quick) {
    if (_limMin.active()) referenceAt(false, _savedRail, homeDest(_savedRail));      // ya esta en X-: va al HOME de X
    else if (_limMax.active()) referenceAt(true, _savedRail, homeDest(_savedRail));  // ya esta en X+: va al HOME de X
    else {  // a medio riel: hacia X- (si toca X+, el sentido estaba al reves y se corrige)
      _dir = -1;
      _ts = TestState::Seek;
      _x.moveBy(_dir * homeSpanSteps());
    }
    _src = 'M';
    _seq = _seq + 1;
    return true;
  }
  if (_limMin.active()) {  // sobre X-: sale hacia el otro lado (se supone +; si no se despega se corrige)
    _from = 0;
    _dir = +1;
    startLeave();
  } else if (_limMax.active()) {  // sobre X+: sale hacia el otro lado
    _from = 1;
    _dir = -1;
    startLeave();
  } else {  // a medio riel: avanza hasta que CUALQUIER interruptor toque
    _dir = -1;
    _ts = TestState::Seek;
    _x.moveBy(_dir * homeSpanSteps());
  }
  _src = 'M';
  _seq = _seq + 1;
  return true;
}

void Machine::startLeave() {
  _ts = TestState::Leave;
  _leaveStart = _x.position();
  _x.moveBy(_dir * homeSpanSteps());
}

void Machine::addLeg(int dir, long steps) {
  if (_legCount < kMaxLegs) {
    _legs[_legCount] = steps;
    _legDir[_legCount] = (int8_t)dir;
    _legCount = _legCount + 1;
  }
}

void Machine::failLimitCal(const char* msg) {
  _x.stop();
  _ts = TestState::None;
  _error = msg;
}

// Promedio de los tramos; el carro esta sobre el interruptor donde acabo el ultimo tramo (atMax = X+). Si hubo que
// corregir el sentido, se invierte el motor X (y se guarda) para que los pasos positivos vayan hacia X+. Se
// referencia ahi (X- = 0, X+ = promedio) y se va a la mitad.
void Machine::finishLimitCal(bool atMax) {
  long sum = 0;
  for (int i = 0; i < _legCount; i++) sum += _legs[i];
  const long n = _legCount > 0 ? _legCount : 1;
  _avgSteps = (sum + n / 2) / n;
  saveRail(_avgSteps);  // el HOME rapido usa este largo
  setXHome(_avgSteps / 2);  // el CENTRO del riel pasa a ser X = 0 (X- queda en negativo y X+ en positivo)
  referenceAt(atMax, _avgSteps, _avgSteps / 2);
}

// HOME (sin largo guardado): el tramo medido es el largo del riel, se guarda; termina en X = 0 (sobre X-).
void Machine::finishHome(bool atMax, long rail) {
  saveRail(rail);
  referenceAt(atMax, rail, homeDest(rail));  // termina en el HOME de X (el centro si se calibro el riel)
}

// El carro esta sobre el interruptor donde acabo el ultimo tramo (atMax = X+). Si hubo que corregir el sentido, se
// invierte el motor X (y se guarda) para que los pasos positivos vayan hacia X+. Se referencia ahi (X- = 0, X+ = rail)
// y se va al destino.
void Machine::referenceAt(bool atMax, long rail, long dest) {
  _railSteps = rail;
  _xHomed = true;
  if (_signMinToMax < 0) {  // los pasos positivos iban hacia X-: el sentido estaba al reves
    const bool inv = !_x.invert();
    _x.setInvert(inv);
    xInvertSave(inv);
    _xFlipped = true;
  }
  _x.setPosition(atMax ? rail : 0);
  _x.moveTo(dest);
  _ts = TestState::ToMiddle;
}

void Machine::updateLimitCal() {
  switch (_ts) {
    case TestState::Seek:  // hasta que cualquier interruptor toque
      if (_quick && (_limMin.active() || _limMax.active())) {  // HOME rapido: el interruptor dice donde esta
        _x.stop();
        const bool atMax = !_limMin.active();
        if (atMax) _signMinToMax = -1;  // iba hacia X- (pasos negativos) y toco X+: sentido al reves, se corrige
        referenceAt(atMax, _savedRail, homeDest(_savedRail));
      } else if (_limMin.active() || _limMax.active()) {
        _x.stop();
        _from = _limMin.active() ? 0 : 1;
        Serial.printf("[ref] toco %s en el paso %ld\n", _from == 0 ? "X-" : "X+", _x.position());
        _dir = -_dir;  // sale en sentido contrario al que lo trajo
        startLeave();
      }
      break;

    case TestState::Leave: {  // saliendo del interruptor del que parte: el conteo empieza cuando se despega
      const bool fromActive = (_from == 0) ? _limMin.active() : _limMax.active();
      if (!fromActive) {
        _dirVerified = true;
        _tStart = _x.position();
        if (_signMinToMax == 0) _signMinToMax = (_from == 0) ? _dir : -_dir;
        Serial.printf("[ref] se despego de %s tras %ld pasos (sentido %+d)\n", _from == 0 ? "X-" : "X+",
                      labs(_x.position() - _leaveStart), _dir);
        _ts = TestState::Cross;
      } else if (labs(_x.position() - _leaveStart) > dirDetectSteps()) {
        if (!_dirVerified && !_flipTried) {  // no se despega: el sentido de los pasos estaba al reves
          Serial.printf("[ref] %s no se despego en %ld pasos (sentido %+d): invierto el sentido\n", _from == 0 ? "X-" : "X+",
                        labs(_x.position() - _leaveStart), _dir);
          _flipTried = true;
          _dir = -_dir;
          _x.stop();
          startLeave();
        } else {
          Serial.printf("[ref] ERROR: %s sigue activo tras %ld pasos (sentido %+d). X-:%d X+:%d, invertido=%d, vel=%d p/s\n",
                        _from == 0 ? "X-" : "X+", labs(_x.position() - _leaveStart), _dir, _limMin.active(), _limMax.active(),
                        _x.invert(), _spsX);
          Serial.println("[ref] Revisar: fuente de 5 V del ULN2003 encendida, carro libre, y que el interruptor suelte al alejarse.");
          failLimitCal("Final no se despega");  // 20 caracteres maximo: asi cabe en la pantalla
        }
      }
      break;
    }

    case TestState::Cross: {  // hacia el otro interruptor
      const bool otherActive = (_from == 0) ? _limMax.active() : _limMin.active();
      if (otherActive) {
        _x.stop();
        const long steps = labs(_x.position() - _tStart);
        const bool atMax = (_from == 0);
        if (_endZero) {  // HOME: una corrida, sin tocar las muestras de la calibracion
          finishHome(atMax, steps);
          break;
        }
        addLeg(_from == 0 ? +1 : -1, steps);
        if (_legCount >= _tSamples) {
          finishLimitCal(atMax);
        } else {
          _from = atMax ? 1 : 0;
          _dir = -_dir;
          startLeave();
        }
      }
      break;
    }

    case TestState::ToMiddle:  // destino final: el medio (calibracion) o X = 0 (HOME)
      if (!_x.isRunning()) {
        _calibrated = _xHomed && _yZeroed;
        _ts = TestState::None;
        _src = 'M';       // el objetivo cambio (ahora en el medio): la pantalla lo toma
        _seq = _seq + 1;
      }
      break;

    default:
      break;
  }
}
