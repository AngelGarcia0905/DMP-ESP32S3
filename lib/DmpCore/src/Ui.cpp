#include "Ui.h"

#include <cstdarg>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "As5600.h"
#include "Imu.h"
#include "Machine.h"
#include "SheetMap.h"
#include "Net.h"
#include "dmp_config.h"
#include "dmp_pins.h"

// ---- Ajustes de la interfaz -------------------------------------------------------------
namespace {
constexpr bool     KNOB_INVERT   = false;  // true si al girar a la derecha el valor baja
constexpr int      X_STEP_MM     = 1;      // un "clic" de la perilla = 1 mm
// Y avanza Y_CLICK_DEG grados por clic (unos 0.7 mm sobre la hoja cerca del centro; un paso del motor son solo 0.022
// grados y no se nota). El objetivo va siempre a multiplos de Y_CLICK_DEG, redondeado al paso mas cercano, y el valor
// mostrado es el que el motor alcanza de verdad (por eso sale 0.24, 0.51...). Para lo fino: MOVER en modo PASOS.
constexpr float    Y_CLICK_DEG   = 0.25f;
constexpr int      X_FAST_MULT   = 5;      // girando rapido: 5 mm por clic
constexpr int      Y_FAST_MULT   = 5;      // girando rapido: 5 x Y_CLICK_DEG por clic
constexpr uint32_t FAST_MS       = 45;     // dos clics mas juntos que esto = giro rapido
constexpr int      LIVE_SPEED    = 100;    // % de velocidad al mover con la perilla (100 % = SPS_MAX)

constexpr uint16_t WHITE = 0xFFFF, BLACK = 0x0000, GREY = 0x8410;

// Orden de color de la pantalla (verificado en el equipo: los colores salen bien). Con SHOW_COLOR_CHECK = true
// se muestran 2.5 s al arrancar tres barras (ROJO, VERDE, AZUL) para revisarlo de nuevo.
constexpr bool TFT_SWAP_RB = false;
constexpr bool TFT_INVERT = false;
constexpr bool SHOW_COLOR_CHECK = false;

constexpr uint16_t pack565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return TFT_SWAP_RB ? pack565(b, g, r) : pack565(r, g, b);
}
// Un color por modulo, los mismos tonos que las secciones del panel web (mas claros para fondo negro).
constexpr uint16_t C_BLUE   = rgb(70, 150, 255);   // RED = Conexion
constexpr uint16_t C_GREEN  = rgb(50, 205, 100);   // MOVER = Posicionamiento
constexpr uint16_t C_RED    = rgb(240, 75, 80);    // LASER = Control
constexpr uint16_t C_ORANGE = rgb(255, 160, 40);   // CALIBRAR = Calibracion
constexpr uint16_t C_PINK   = rgb(235, 70, 150);   // SENSORES = Telemetria
constexpr uint16_t C_TEAL   = rgb(40, 205, 205);   // HOME
constexpr uint16_t C_YELLOW = rgb(240, 215, 60);   // AJUSTES = Velocidad, medir avance y escala
constexpr uint16_t C_VIOLET = rgb(180, 120, 255);  // PUNTOS = puntos guardados y mapa

// Orden de la interfaz (el laser se enciende y apaga dentro de MOVER y al marcar los puntos de la hoja):
//   MOVER     = mover X, Y, escribir H/V e IR, HOME X / Y / AMBOS (ir al HOME) y laser
//   GUARDADOS = hasta 8 puntos guardados y su mapa
//   CALIBRAR  = lo que se calibra y se marca: IMUs, HOME (marcar), LIMITS (riel y escala de X), HOJA y TOPES MIN/MAX
//   AJUSTES   = VELOCIDAD, MEDIR Y ESCALA (contador y escala de Y) y ORIGEN de la hoja
//   SENSORES  = toda la informacion      RED = punto de acceso
constexpr int MENU_N = 6;
const char* const kMenu[MENU_N] = {"MOVER", "GUARDADOS", "CALIBRAR", "AJUSTES", "SENSORES", "RED"};
const uint16_t kMenuColor[MENU_N] = {C_GREEN, C_VIOLET, C_ORANGE, C_YELLOW, C_PINK, C_BLUE};
constexpr int CAL_N = 6;
const char* const kCal[CAL_N] = {"CALIBRAR LAS 2 IMUs", "HOME (MARCAR)", "CALIBRAR LIMITS", "HOJA (4/6/9 PUNTOS)",
                                 "TOPES MIN / MAX", "AFINAR (PRUEBAS)"};
constexpr int SENS_ROWS = 12;  // renglones visibles en SENSORES (la lista se recorre con la perilla)
constexpr float LEVEL_TOL_DEG = 0.3f;  // NIVEL: dentro de esta inclinacion (grados) en los dos ejes se considera nivelado
constexpr int H_STEP_MM = 1, H_FAST_MULT = 5;  // H / V de la hoja: 1 mm por clic, 5 mm girando rapido

inline float yDegOf(int steps) { return steps * Cfg::Y_DEG_PER_STEP; }
// H/V de la hoja: por dentro siempre centro = 0; en pantalla se suma esto si el origen es la esquina inferior izquierda.
inline float hShift(const Machine& m) { return m.sheetCorner() ? Cfg::SHEET_W_MM / 2 : 0.0f; }
inline float vShift(const Machine& m) { return m.sheetCorner() ? Cfg::SHEET_H_MM / 2 : 0.0f; }
// Rango de MOVER en las unidades de la perilla (mm enteros de X, pasos de Y), siempre hacia adentro.
inline int xLoMm(const Machine& m) { return (int)ceilf(m.xMinCm() * 10.0f - 0.01f); }
inline int xHiMm(const Machine& m) { return (int)floorf(m.xMaxCm() * 10.0f + 0.01f); }
inline int yLoSteps(const Machine& m) { return (int)ceilf(m.yMinDeg() / Cfg::Y_DEG_PER_STEP - 0.01f); }
inline int yHiSteps(const Machine& m) { return (int)floorf(m.yMaxDeg() / Cfg::Y_DEG_PER_STEP + 0.01f); }
constexpr int FREE_X_MM = (int)(Cfg::X_FREE_CM * 10.0f);  // tope tecnico de X (no es un limite real)
inline long freeXSteps() { return (long)(Cfg::X_FREE_CM * 10.0f / Cfg::X_MM_PER_STEP); }  // tope tecnico de X en pasos (la escala puede cambiar)
constexpr int STEPS_FAST_MULT = 50;  // modo PASOS, girando rapido: 50 veces el tamano de paso elegido (x1, x10, x100)

// ---- Perilla KY-040 por interrupcion ----------------------------------------------------
// Dibujar un cuadro tarda ~40 ms; leerla solo en loop() perderia pulsos. La interrupcion
// decodifica la cuadratura (4 transiciones = un clic) y deja los clics pendientes aqui.
const int8_t kT[16] DRAM_ATTR = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile uint8_t g_prev = 0;
volatile int8_t g_acc = 0;
volatile int g_pend = 0;
volatile bool g_fast = false;
volatile uint32_t g_lastMs = 0;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR knobIsr() {
  const uint8_t cur = (gpio_get_level((gpio_num_t)Pins::KNOB_CLK) << 1) | gpio_get_level((gpio_num_t)Pins::KNOB_DT);
  const uint8_t prev = g_prev;
  if (cur == prev) return;
  int acc = g_acc + kT[(prev << 2) | cur];
  g_prev = cur;
  if (acc >= 4 || acc <= -4) {
    const int dir = acc > 0 ? 1 : -1;
    acc = 0;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - g_lastMs < FAST_MS) g_fast = true;
    g_lastMs = now;
    g_pend = g_pend + (KNOB_INVERT ? -dir : dir);
  }
  g_acc = (int8_t)acc;
}
}  // namespace

// ---- Arranque ---------------------------------------------------------------------------
void Ui::begin() {
  pinMode(Pins::KNOB_CLK, INPUT_PULLUP);
  pinMode(Pins::KNOB_DT, INPUT_PULLUP);
  pinMode(Pins::KNOB_SW, INPUT_PULLUP);
  g_prev = (digitalRead(Pins::KNOB_CLK) << 1) | digitalRead(Pins::KNOB_DT);
  attachInterrupt(digitalPinToInterrupt(Pins::KNOB_CLK), knobIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pins::KNOB_DT), knobIsr, CHANGE);

  SPI.begin(Pins::TFT_SCK, -1, Pins::TFT_MOSI, -1);  // sin MISO: GPIO 13 es DC
  _tft = new Adafruit_ST7735(Pins::TFT_CS, Pins::TFT_DC, Pins::TFT_RST);
  _tft->initR(INITR_BLACKTAB);
  _tft->setRotation(0);  // vertical 128x160
  if (TFT_INVERT) _tft->invertDisplay(true);
  _tft->fillScreen(BLACK);
  _cv = new GFXcanvas16(128, 160);  // se dibuja aqui y se manda completo: sin parpadeo
  if (SHOW_COLOR_CHECK) {
    colorCheck();
    delay(2500);
  }

  _limRuns = Cfg::LIMIT_CAL_SAMPLES;
  adoptTarget();
  if (!_m.xHomed() && !Cfg::TEST_NO_LIMIT_SWITCHES) _screen = ARRANQUE;  // ofrece referenciar X (no se mueve nada hasta el clic)
  _seen = _m.seq();
  render();
}

// Prueba de color al arrancar: cada barra debe verse del color que dice su rotulo.
void Ui::colorCheck() {
  _cv->fillScreen(BLACK);
  const uint16_t cols[3] = {rgb(255, 0, 0), rgb(0, 255, 0), rgb(0, 0, 255)};
  const char* names[3] = {"ROJO", "VERDE", "AZUL"};
  for (int i = 0; i < 3; i++) {
    _cv->fillRect(0, i * 38, 128, 36, cols[i]);
    textCentered(64, i * 38 + 14, names[i], 1, i == 1 ? BLACK : WHITE);
  }
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  _cv->setCursor(3, 116);  _cv->print("Cada barra debe ser");
  _cv->setCursor(3, 125);  _cv->print("del color que dice.");
  _cv->setCursor(3, 134);  _cv->print("Si no, en Ui.cpp:");
  _cv->setCursor(3, 143);  _cv->print("TFT_SWAP_RB");
  _cv->setCursor(3, 152);  _cv->print("o TFT_INVERT");
  _tft->drawRGBBitmap(0, 0, _cv->getBuffer(), 128, 160);
  _frameHash = 0;
}

// ---- Bucle ------------------------------------------------------------------------------
void Ui::update() {
  if (!_tft || !_cv) return;

  int pend;
  bool fast;
  portENTER_CRITICAL(&g_mux);
  pend = g_pend;
  fast = g_fast;
  g_pend = 0;
  g_fast = false;
  portEXIT_CRITICAL(&g_mux);
  if (pend) onTurn(pend, fast);

  pollButton();
  syncFromMachine();
  if (millis() - _tiltFeed >= 100) {  // la inclinacion de las IMUs va a la Machine (correccion de IR)
    _tiltFeed = millis();
    float px, py, dx, dy;
    const int n = levelInfo(&px, &py, &dx, &dy);
    _m.setTilt(px, py, n);
  }
  if (_afPend) {  // AFINAR: termino el primer movimiento (el que pasa de largo): ahora el que llega al punto
    if (_screen != AFINAR) _afPend = false;
    else if (!_m.moving()) {
      _afPend = false;
      const char* e = "";
      if (_m.moveToSheet(_afH2, _afV2, &e, 'S')) say("Llegando: mide"); else say(e);
    }
  }

  const uint32_t now = millis();
  if (_msgUntil && (int32_t)(now - _msgUntil) >= 0) { _msgUntil = 0; _dirty = true; }
  if (_dirty || now - _lastDraw >= 150) {
    _dirty = false;
    _lastDraw = now;
    render();
  }
}

// ---- Sincronia con el panel web ---------------------------------------------------------
// Se toma el objetivo tal cual (sin recortar). H / V dejan de valer.
void Ui::adoptTarget() {
  _tgtXmm = (int)lroundf(_m.targetXCm() * 10.0f);
  _tgtXs = lroundf(_m.targetXCm() * 10.0f / Cfg::X_MM_PER_STEP);
  _tgtYsteps = (int)lroundf(_m.targetAngleDeg() / Cfg::Y_DEG_PER_STEP);
}

// MOVER en modo PASOS: X se maneja en pasos (_tgtXs) y _tgtXmm solo acompana para lo que se muestra en otras partes.
bool Ui::stepsMode() const { return _screen == MOVER && _stepsMode; }

void Ui::clampToLimits() {
  if (stepsMode()) {
    const long lo = (long)ceilf(_m.xMinCm() * 10.0f / Cfg::X_MM_PER_STEP - 0.01f);
    const long hi = (long)floorf(_m.xMaxCm() * 10.0f / Cfg::X_MM_PER_STEP + 0.01f);
    _tgtXs = constrain(_tgtXs, lo, hi);
    _tgtXmm = (int)lroundf(_tgtXs * Cfg::X_MM_PER_STEP);
  } else {
    _tgtXmm = constrain(_tgtXmm, xLoMm(_m), xHiMm(_m));
  }
  _tgtYsteps = constrain(_tgtYsteps, yLoSteps(_m), yHiSteps(_m));
}

// Si el panel web (o un HOME/STOP) cambio el objetivo, la pantalla lo toma; asi las dos muestran lo mismo.
void Ui::syncFromMachine() {
  if (_m.homing()) { _wasHoming = true; return; }
  if (_wasHoming) {  // termino HOME: X en 0, Y en 0 (si algo fallo, se muestra el motivo)
    _wasHoming = false;
    if (*_m.lastError()) say(_m.lastError());
    adoptTarget();
    _seen = _m.seq();
    _dirty = true;
    return;
  }
  if (_m.seq() != _seen) {
    _seen = _m.seq();
    if (_m.lastSource() != 'S') { adoptTarget(); _dirty = true; }
  }
  // Un final de carrera puede recortar el objetivo: si se aleja mas de un paso, se toma y se muestra el motivo
  // (asi el numero no "regresa" sin explicacion).
  if (!_m.moving()) {
    if (fabsf(_m.targetXCm() * 10.0f - _tgtXmm) > 1.0f ||
        fabsf(_m.targetAngleDeg() - yDegOf(_tgtYsteps)) > Cfg::Y_DEG_PER_STEP * 0.5f) {
      adoptTarget();
      if (*_m.lastError()) say(_m.lastError());
      _dirty = true;
    }
  }
}

// MOVER: dentro de los limites (los puntos de la hoja, si esta calibrada). MARCAR: sin limites de rango.
void Ui::moveLive() {
  const char* err = "";
  bool ok;
  if (_screen == MARCAR || _screen == ESCALA || _screen == MINMAX || _screen == HMARK) {  // en CALIBRAR todo es libre
    ok = _m.jogTo(_tgtXmm / 10.0f, yDegOf(_tgtYsteps), &err, 'S');
  } else {
    clampToLimits();
    // modo PASOS: X exacta en pasos; si no, en mm
    const float xCm = stepsMode() ? _tgtXs * Cfg::X_MM_PER_STEP / 10.0f : _tgtXmm / 10.0f;
    ok = _m.moveTo(xCm, yDegOf(_tgtYsteps), LIVE_SPEED, &err, 'S');
  }
  if (!ok) say(err);
  _seen = _m.seq();
}

void Ui::say(const char* msg) {
  strncpy(_msg, msg, 20);  // en pantalla solo caben 20 caracteres por linea
  _msg[20] = 0;
  _msgUntil = millis() + 2500;
  _dirty = true;
}

// ---- Entrada ----------------------------------------------------------------------------
void Ui::onTurn(int d, bool fast) {
  if (_screen == MENU) {
    _menuSel = ((_menuSel + d) % MENU_N + MENU_N) % MENU_N;
  } else if (_screen == CALIBRAR) {
    _calSel = ((_calSel + d) % (CAL_N + 1) + CAL_N + 1) % (CAL_N + 1);  // IMUs, HOME, LIMITS, HOJA, TOPES, REGRESAR
  } else if (_screen == AFINAR) {
    if (!_afIn) _afTest = ((_afTest + d) % 10 + 10) % 10;  // NIVEL, LIMITS, HOME, ESCALA X, HOLGURA Y/X, INCL H/V, ESQUINAS, REGRESAR
    else if (_editing && _afSel == 2) {
      if (_afTest == 3) _afReal = constrain(_afReal + d * (fast ? 1.0f : 0.1f), 150.0f, 250.0f);
      else _afReal = constrain(_afReal + d * (fast ? 1.0f : 0.1f), -30.0f, 30.0f);
    } else _afSel = ((_afSel + d) % 5 + 5) % 5;  // IR 1, IR 2, valor medido, APLICAR, ATRAS
  } else if (_screen == AJUSTES) {
    _ajSel = ((_ajSel + d) % 4 + 4) % 4;    // VELOCIDAD, MEDIR Y ESCALA, ORIGEN HOJA, REGRESAR
  } else if (_screen == GUARDADOS) {
    if (_editing && _gSel == 0) _gSlot = ((_gSlot + d) % Machine::kMaxSaved + Machine::kMaxSaved) % Machine::kMaxSaved;
    else _gSel = ((_gSel + d) % 5 + 5) % 5;  // PUNTO n, GUARDAR AQUI, IR AL PUNTO, BORRAR, REGRESAR
  } else if (_screen == ESCALA) {
    if (_editing && _escSel <= 1) editAxis(_escSel, d, fast);  // mover X / Y (sin limites)
    else if (_editing && _escSel == 3) _realY = max(0.0f, _realY + d * (fast ? 0.5f : 0.05f));
    else _escSel = ((_escSel + d) % 7 + 7) % 7;  // X, Y, MARCAR INICIO, REAL Y, APLICAR Y, FABRICA, REGRESAR
  } else if (_screen == VELOCIDAD) {
    if (!_editing) {
      _spdSel = ((_spdSel + d) % 3 + 3) % 3;  // X, Y, REGRESAR
    } else {
      int& v = (_spdSel == 0) ? _spdX : _spdY;
      v = constrain(v + d * Cfg::SPS_UI_STEP, Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX);
    }
  } else if (_screen == SENSORES) {  // girar recorre la lista
    _sensTop = constrain(_sensTop + d, 0, max(0, _sensLines - SENS_ROWS));
  } else if (_screen == HOME) {
    _homeSel = ((_homeSel + d) % 4 + 4) % 4;  // HOME X, HOME Y, AMBOS, REGRESAR
  } else if (_screen == ARRANQUE) {
    _bootSel = 1 - _bootSel;  // REFERENCIAR AHORA / DESPUES
  } else if (_screen == LIMITS) {
    if (_editing && _limSel == 0) _limRuns = constrain(_limRuns + d, 1, Cfg::LIMIT_CAL_MAX_RUNS);
    else if (_editing && _limSel == 2) { _limReal = max(0.0f, limRealShown() + d * (fast ? 1.0f : 0.1f)); _limRealSet = true; }
    else if (!_m.testing()) _limSel = ((_limSel + d) % 5 + 5) % 5;  // Corr., INICIAR, REAL, APLICAR ESCALA, REGRESAR
  } else if (_screen == PUNTOS) {
    if (_editing) {  // 4 -> 6 -> 9 -> 4 ...
      static const int kN[3] = {4, 6, 9};
      int k = _ptsN == 4 ? 0 : (_ptsN == 6 ? 1 : 2);
      _ptsN = kN[((k + d) % 3 + 3) % 3];
    } else {
      _ptsSel = ((_ptsSel + d) % 4 + 4) % 4;  // Pts, INICIAR, AGREGAR, REGRESAR
    }
  } else if (_screen == MARCAR) {
    if (!_editing) _markSel = ((_markSel + d) % 5 + 5) % 5;  // X, Y, OK, LASER, REGRESAR
    else editAxis(_markSel, d, fast);
  } else if (_screen == HMARK) {
    if (!_editing) _hmSel = ((_hmSel + d) % 7 + 7) % 7;  // X, Y, MARCAR AMBOS/X/Y, REFERENCIAR X, REGRESAR
    else editAxis(_hmSel, d, fast);
  } else if (_screen == MINMAX) {
    if (!_editing) _mmSel = ((_mmSel + d) % 8 + 8) % 8;  // X, Y, 4 topes, BORRAR, REGRESAR
    else editAxis(_mmSel, d, fast);
  } else if (_screen == MOVER) {
    if (!_editing) {
      _axisSel = ((_axisSel + d) % 11 + 11) % 11;  // X, Y, H, V, IR, HOME X, HOME Y, AMBOS, UNIDAD, LASER, REGRESAR
    } else if (_m.homing()) {
      say("HOME en curso");
    } else if (_axisSel <= 1) {
      editAxis(_axisSel, d, fast);  // X / Y: en vivo, cada clic mueve
    } else {  // H / V de la hoja (mm): solo se ESCRIBE el valor; el boton IR mueve
      const float step = d * H_STEP_MM * (fast ? H_FAST_MULT : 1);
      // Dentro del area util (centro = 0); lo que pase de los topes o de la hoja marcada lo rechaza IR.
      if (_axisSel == 2) _hMm = constrain(_hMm + step, -Cfg::SHEET_W_MM / 2, Cfg::SHEET_W_MM / 2);
      else _vMm = constrain(_vMm + step, -Cfg::SHEET_H_MM / 2, Cfg::SHEET_H_MM / 2);
    }
  }
  else if (_screen == RED) _redSel = 1 - _redSel;  // boton del punto de acceso o REGRESAR
  _dirty = true;
}

// HOME: 0 = X, 1 = Y, 2 = los dos (al HOME marcado en CALIBRAR > HOME).
void Ui::runHome(int kind) {
  const char* err = "";
  bool ok;
  if (kind == 0) ok = _m.startHomeX(&err);
  else if (kind == 1) ok = _m.startHomeY(&err);
  else ok = _m.startHome(&err);
  say(ok ? (kind == 0 ? "HOME X..." : (kind == 1 ? "HOME Y..." : "HOME X y Y...")) : err);
}

// X (mm) o Y (pasos) a mano. MOVER se queda en la orilla de los limites (moveLive recorta); MARCAR no tiene limite.
void Ui::editAxis(int axis, int d, bool fast) {
  if (_m.homing()) { say("HOME en curso"); return; }
  if (stepsMode()) {  // pasos exactos de cada motor; girando rapido, STEPS_FAST_MULT pasos por clic
    const int k = d * _stepSize * (fast ? STEPS_FAST_MULT : 1);
    if (axis == 0) _tgtXs = constrain(_tgtXs + k, -freeXSteps(), freeXSteps());
    else _tgtYsteps += k;
    _tgtXmm = (int)lroundf(_tgtXs * Cfg::X_MM_PER_STEP);
  } else if (axis == 0) {  // X en mm: 1 mm por clic (x10 con el recuadro de unidad en "x10")
    const int mult = (_screen == MOVER) ? _mmMult : 1;
    _tgtXmm = constrain(_tgtXmm + d * X_STEP_MM * mult * (fast ? X_FAST_MULT : 1), -FREE_X_MM, FREE_X_MM);
  } else {  // Y en grados: al multiplo de Y_CLICK_DEG mas cercano y de ahi d clics (x10 igual que X)
    const int mult = (_screen == MOVER) ? _mmMult : 1;
    const float base = roundf(yDegOf(_tgtYsteps) / Y_CLICK_DEG) * Y_CLICK_DEG;
    const float deg = base + d * Y_CLICK_DEG * mult * (fast ? Y_FAST_MULT : 1);
    _tgtYsteps = (int)lroundf(deg / Cfg::Y_DEG_PER_STEP);
  }
  moveLive();  // en vivo: cada clic mueve el laser
}

void Ui::onClick() {
  const char* err = "";
  // Con un HOME en curso, cualquier clic lo cancela (desde la pantalla que sea).
  if (_m.homing()) {
    _m.cancelHome();
    say("HOME cancelado");
    return;
  }
  if (_screen == MENU) {
    switch (_menuSel) {
      case 0: _screen = MOVER; _axisSel = 0; _editing = false; break;
      case 1: _screen = GUARDADOS; _gSel = 0; _editing = false; break;
      case 2: _screen = CALIBRAR; break;
      case 3: _screen = AJUSTES; _ajSel = 0; _editing = false; break;
      case 4: _screen = SENSORES; _sensTop = 0; break;
      case 5: _screen = RED; _redSel = 1; break;  // entra con REGRESAR elegido: no se apaga el Wi-Fi por accidente
    }
  } else if (_screen == MOVER) {
    if (_axisSel == 10) { _screen = MENU; _editing = false; }  // pestana REGRESAR
    else if (_axisSel == 9) _m.setLaser(!_m.laser());        // LASER: un clic lo enciende o lo apaga
    else if (_axisSel == 8) {  // UNIDAD: mm/grados x1 -> mm/grados x10 -> pasos x1 -> x10 -> x100 -> mm/grados x1
      if (!_stepsMode && _mmMult == 1) _mmMult = 10;
      else if (!_stepsMode) { _stepsMode = true; _stepSize = 1; }
      else if (_stepSize == 1) _stepSize = 10;
      else if (_stepSize == 10) _stepSize = 100;
      else { _stepsMode = false; _mmMult = 1; }
      adoptTarget();  // los valores se vuelven a leer de la maquina en la unidad nueva
      char b[24];
      if (_stepsMode) snprintf(b, sizeof(b), "Pasos x%d por clic", _stepSize);
      else snprintf(b, sizeof(b), "mm y grados x%d", _mmMult);
      say(b);
    }
    else if (_axisSel >= 5) runHome(_axisSel - 5);           // recuadros HOME X / HOME Y / AMBOS: un clic
    else if (_axisSel >= 2 && !_m.gridPoints()) say("Hoja sin calibrar");  // H / V / IR necesitan la hoja calibrada
    else if (_axisSel == 4) {  // IR: va al H / V escrito
      if (!_sheetValid) say("Escribe H y V");
      else if (_m.moveToSheet(_hMm, _vMm, &err, 'S')) {
        _tgtXmm = (int)lroundf(_m.targetXCm() * 10.0f);
        _tgtXs = lroundf(_m.targetXCm() * 10.0f / Cfg::X_MM_PER_STEP);
        _tgtYsteps = (int)lroundf(_m.targetAngleDeg() / Cfg::Y_DEG_PER_STEP);
        _seen = _m.seq();
        say("Yendo a H, V");
      } else {
        say(err);
      }
    } else {
      if (_axisSel >= 2 && !_sheetValid) { _hMm = 0; _vMm = 0; _sheetValid = true; }  // parte del centro del area (H 0, V 0)
      _editing = !_editing;  // un clic entra a editar; otro clic sale y se puede elegir otra fila
    }
  } else if (_screen == ARRANQUE) {
    if (_bootSel == 0) {  // REFERENCIAR AHORA: busca un final de carrera y lleva X al HOME
      if (_m.startRefX(&err)) { _screen = HOME; say("Referenciando X..."); }
      else { say(err); _screen = MENU; }
    } else {
      _screen = MENU;
    }
  } else if (_screen == LIMITS) {
    if (_limSel == 0 || _limSel == 2) _editing = !_editing;  // corridas / riel real: clic para cambiar, otro clic para dejarlo
    else if (_limSel == 1) {
      _limRealSet = false;  // el riel real se vuelve a proponer con lo que mida esta calibracion
      if (_m.startLimitCal(_limRuns, &err)) say("Calibrando limits..."); else say(err);
    } else if (_limSel == 3) {  // APLICAR ESCALA: mm por paso de X = riel real medido / pasos del riel
      const long steps = _m.savedRailSteps();
      if (steps < 1000) say("Mide el riel primero");
      else {
        const float real = limRealShown(), n = real / steps;
        if (real <= 0 || n < Cfg::X_MM_PER_STEP * 0.2f || n > Cfg::X_MM_PER_STEP * 5.0f) say("Valor fuera de rango");
        else if (_m.setScales(n, 0)) { _limRealSet = false; say("Escala X guardada"); }
      }
    } else { _screen = CALIBRAR; _editing = false; }
  } else if (_screen == PUNTOS) {
    if (_ptsSel == 0) _editing = !_editing;
    else if (_ptsSel == 1 || _ptsSel == 2) {  // INICIAR (marca todos) o AGREGAR (conserva los vigentes y marca solo los nuevos)
      if (_m.beginGrid(_ptsN, &err, _ptsSel == 2)) {
        _screen = MARCAR;
        _ptIdx = _m.nextPoint(0);
        _markSel = 0;
        _editing = false;
        adoptTarget();
      } else {
        say(err);
      }
    } else { _screen = CALIBRAR; _editing = false; }  // HOJA esta dentro de CALIBRAR
  } else if (_screen == MINMAX) {
    if (_mmSel <= 1) {
      _editing = !_editing;  // X o Y: un clic entra a editar (sin limites), otro sale
    } else if (_mmSel <= 5) {
      static const char* const kT[4] = {"X min fijado", "X max fijado", "Y min fijado", "Y max fijado"};
      if (_m.setTope(_mmSel - 2, &err)) say(kT[_mmSel - 2]); else say(err);
    } else if (_mmSel == 6) {
      _m.clearTopes();
      say("Topes borrados");
    } else {
      _screen = CALIBRAR;
      _editing = false;
    }
  } else if (_screen == MARCAR) {
    if (_markSel <= 1) {
      _editing = !_editing;  // X o Y: un clic entra a editar, otro sale
    } else if (_markSel == 2) {  // OK: guarda el punto y pasa al siguiente
      char b[24];
      if (!_m.markPoint(_ptIdx, &err)) {
        say(err);
      } else if (_m.nextPoint(_ptIdx + 1) >= _ptsN) {  // ya estan todos (los conservados no se vuelven a marcar)
        _m.commitGrid();
        snprintf(b, sizeof(b), "Hoja lista: %d pts", _ptsN);
        say(b);
        _screen = PUNTOS;
        _ptsSel = 0;
      } else {
        snprintf(b, sizeof(b), "Punto %d guardado", _ptIdx + 1);
        say(b);
        _ptIdx = _m.nextPoint(_ptIdx + 1);
        _markSel = 0;
      }
    } else if (_markSel == 3) {
      _m.setLaser(!_m.laser());  // LASER: para ver el punto al marcar
    } else {  // REGRESAR: cancela; la calibracion anterior queda igual
      say("Marcado cancelado");
      _screen = PUNTOS;
      _editing = false;
    }
  } else if (_screen == CALIBRAR) {
    switch (_calSel) {
      case 0: {  // las dos IMUs a la vez (las que esten conectadas)
        int started = 0;
        const char* missing = nullptr;
        if (_imuW.canStart(&err)) { _imuW.startCalibration(&err); started++; }
        else if (!_imuW.ok()) missing = "IMU blanco";
        if (_imuB.canStart(&err)) { _imuB.startCalibration(&err); started++; }
        else if (!_imuB.ok()) missing = "IMU negro";
        if (started == 2) say("Calibrando 2 IMUs");
        else if (started == 1 && missing) { char b[26]; snprintf(b, sizeof(b), "Falta %s", missing); say(b); }
        else if (started == 0 && missing) say("IMUs no conectadas");
        else say("IMUs ocupadas");
        break;
      }
      case 1:  // marcar el HOME (X = 0 y Y = 0 ahi), moviendo libre como en MOVER
        _screen = HMARK;
        _hmSel = 0;
        _editing = false;
        adoptTarget();
        break;
      case 2: _screen = LIMITS; _limSel = 0; _editing = false; break;  // finales de carrera: corridas y escala de X
      case 3: _screen = PUNTOS; _ptsSel = 0; _editing = false; break;  // HOJA: marcar 4, 6 o 9 puntos
      case 4: _screen = MINMAX; _mmSel = 0; _editing = false; adoptTarget(); break;  // topes de MOVER
      case 5: _screen = AFINAR; _afTest = 0; _afIn = false; _afPend = false; _editing = false; break;  // pruebas guiadas
      default: _screen = MENU; break;  // REGRESAR
    }
  } else if (_screen == AJUSTES) {
    if (_ajSel == 0) {  // velocidad de cada eje
      _screen = VELOCIDAD;
      _spdSel = 0;
      _editing = false;
      _spdX = _m.speedX();
      _spdY = _m.speedY();
    } else if (_ajSel == 1) {  // MEDIR Y ESCALA: contador libre desde un inicio y correccion de la escala de Y
      _screen = ESCALA;
      _escSel = 0;
      _editing = false;
      adoptTarget();
      markMeasureStart();
      _realY = 0;
    } else if (_ajSel == 2) {  // ORIGEN de la hoja: centro <-> esquina inferior izquierda (solo cambia lo que se muestra)
      _m.setSheetCorner(!_m.sheetCorner());
      say(_m.sheetCorner() ? "Origen: esquina" : "Origen: centro");
    } else {
      _screen = MENU;
    }
  } else if (_screen == GUARDADOS) {
    char b[24];
    if (_gSel == 0) {
      _editing = !_editing;  // un clic para elegir el punto (girar), otro para dejarlo
    } else if (_gSel == 1) {
      if (_m.savePoint(_gSlot, &err)) { snprintf(b, sizeof(b), "Punto %d guardado", _gSlot + 1); say(b); } else say(err);
    } else if (_gSel == 2) {
      if (_m.gotoPoint(_gSlot, &err)) {
        adoptTarget();  // MOVER toma el objetivo nuevo
        _seen = _m.seq();
        snprintf(b, sizeof(b), "Yendo al punto %d", _gSlot + 1);
        say(b);
      } else {
        say(err);
      }
    } else if (_gSel == 3) {
      _m.clearPoint(_gSlot);
      snprintf(b, sizeof(b), "Punto %d borrado", _gSlot + 1);
      say(b);
    } else {
      _screen = MENU;
      _editing = false;
    }
  } else if (_screen == ESCALA) {
    if (_escSel <= 1) {
      _editing = !_editing;  // X / Y: un clic entra a mover (sin limites), otro sale
    } else if (_escSel == 2) {
      markMeasureStart();
      _realY = 0;
      say("Inicio marcado");
    } else if (_escSel == 3) {  // REAL Y: al entrar propone lo que midio el AS5600 (o lo que cree el equipo)
      if (!_editing) {
        if (_measEncOk && _enc.live() && _enc.hasZero()) {
          float e = _enc.relativeDeg() - _measEnc0;
          while (e > 180.0f) e -= 360.0f;
          while (e < -180.0f) e += 360.0f;
          _realY = fabsf(e);
        } else {
          _realY = labs(_m.ySteps() - _measY0) * Cfg::Y_DEG_PER_STEP;
        }
      }
      _editing = !_editing;
    } else if (_escSel == 4) {  // APLICAR Y: grados por paso = lo que se midio / los pasos que dio
      const long dy = labs(_m.ySteps() - _measY0);
      if (!_measMarked || dy < 50) say("Mueve Y primero");
      else {
        const float n = _realY / dy;
        if (_realY <= 0 || n < Cfg::Y_DEG_PER_STEP * 0.2f || n > Cfg::Y_DEG_PER_STEP * 5.0f) say("Valor fuera de rango");
        else if (_m.setScales(0, n)) {
          if (_enc.live() && _enc.hasZero()) _m.setYFromDeg(_enc.relativeDeg());  // el contador de Y = el sensor
          say("Escala Y guardada");
          markMeasureStart();
          _realY = 0;
        }
      }
    } else if (_escSel == 5) {
      _m.resetScales();
      say("Escalas de fabrica");
    } else {
      _screen = AJUSTES;
      _editing = false;
    }
  } else if (_screen == VELOCIDAD) {
    if (_spdSel <= 1) {
      if (_editing) { _m.setSpeeds(_spdX, _spdY); say("Velocidad guardada"); }  // al salir de editar se aplica y guarda
      _editing = !_editing;
    } else {
      _screen = AJUSTES;
      _editing = false;
    }
  } else if (_screen == HMARK) {
    if (_hmSel <= 1) {
      _editing = !_editing;  // X o Y: un clic entra a editar (sin limites), otro sale
    } else if (_hmSel <= 4) {  // MARCAR AMBOS / X / Y
      const bool x = (_hmSel != 4), y = (_hmSel != 3);
      if (_m.markHome(x, y, &err)) {
        say(x && y ? "HOME marcado (X y Y)" : (x ? "HOME X marcado" : "HOME Y marcado"));
        adoptTarget();  // ahora este lugar es X = 0 / Y = 0
      } else {
        say(err);
      }
    } else if (_hmSel == 5) {
      if (_m.startRefX(&err)) say("Referenciando X..."); else say(err);
    } else {
      _screen = CALIBRAR;
      _editing = false;
    }
  } else if (_screen == NIVEL) {
    _screen = AFINAR;  // un clic regresa a la lista
    _afIn = false;
  } else if (_screen == AFINAR) {
    afinarClick();
  } else if (_screen == HOME) {
    if (_homeSel <= 2) runHome(_homeSel);  // HOME X / HOME Y / AMBOS
    else _screen = MENU;                   // REGRESAR
  } else {
    if (_screen == RED && _redSel == 0) {
      Net::setEnabled(!Net::enabled());
      say(Net::enabled() ? "Punto acceso ON" : "Punto acceso OFF");
    } else {
      _screen = MENU;  // SENSORES / REGRESAR de RED: un clic vuelve
    }
  }
  _dirty = true;
}

// Solo clic corto (ya no hay clic largo): se actua al soltar el boton.
void Ui::pollButton() {
  static bool down = false;
  static uint32_t tLast = 0;
  const bool pressed = (digitalRead(Pins::KNOB_SW) == LOW);
  const uint32_t now = millis();
  if (now - tLast < 25) return;  // antirrebote
  if (pressed && !down) { down = true; tLast = now; }
  else if (!pressed && down) { down = false; tLast = now; onClick(); }
}

// ---- Dibujo -----------------------------------------------------------------------------
void Ui::textCentered(int cx, int y, const char* s, int size, uint16_t color) {
  const int w = strlen(s) * 6 * size;
  _cv->setTextSize(size);
  _cv->setTextColor(color);
  _cv->setCursor(cx - w / 2, y);
  _cv->print(s);
}

// Color del modulo de la pantalla actual (el menu principal queda en blanco).
uint16_t Ui::accent() const {
  switch (_screen) {
    case MOVER:    return C_GREEN;
    case HOME:     return C_TEAL;
    case SENSORES: return C_PINK;
    case RED:      return C_BLUE;
    case CALIBRAR:
    case MINMAX:
    case HMARK:
    case AFINAR:
    case NIVEL:
    case LIMITS:
    case PUNTOS:
    case MARCAR:
    return C_ORANGE;
    case AJUSTES:
    case ESCALA:
    case VELOCIDAD: return C_YELLOW;
    case GUARDADOS: return C_VIOLET;
    case ARRANQUE: return C_ORANGE;
    default:       return WHITE;
  }
}

void Ui::header(const char* title) {
  const uint16_t ac = accent();
  _cv->setTextSize(1);
  _cv->setTextColor(ac);
  _cv->setCursor(3, 3);
  _cv->print(title);
  _cv->setTextColor(WHITE);
  _cv->setCursor(58, 3);
  if (Net::enabled()) _cv->printf("W%d", Net::clients()); else _cv->print("W-");
  _cv->setCursor(78, 3);
  _cv->print(_m.homing() ? (_m.testing() ? "LIMITS" : "HOME") : (_m.moving() ? "MOV" : (_m.testMode() ? "TEST" : "")));
  // "X?" en rojo: X todavia no tiene referencia (se pierde al apagar; HOME X no funciona hasta REFERENCIAR X)
  if (!_m.xHomed() && !_m.homing() && !_m.moving()) {
    _cv->setTextColor(C_RED);
    _cv->setCursor(98, 3);
    _cv->print("X?");
  }
  if (_m.laser()) _cv->fillCircle(120, 6, 4, C_RED); else _cv->drawCircle(120, 6, 4, C_RED);
  _cv->drawFastHLine(0, 13, 128, ac);
  _cv->drawFastHLine(0, 14, 128, ac);
}

void Ui::drawMenu() {
  header("DMP");
  for (int i = 0; i < MENU_N; i++) {  // cuadricula de 2 x 3
    const int c = i % 2, r = i / 2;
    const int x = 1 + c * 64, y = 17 + r * 37, w = 62, h = 33;
    const bool sel = (i == _menuSel);
    const uint16_t ac = kMenuColor[i];
    // seleccionado: relleno del color del modulo; si no: marco de 2 px y texto en ese color
    if (sel) {
      _cv->fillRect(x, y, w, h, ac);
    } else {
      _cv->drawRect(x, y, w, h, ac);
      _cv->drawRect(x + 1, y + 1, w - 2, h - 2, ac);
    }
    textCentered(x + w / 2, y + 13, kMenu[i], 1, sel ? BLACK : ac);
  }
  _cv->drawFastHLine(0, 133, 128, GREY);
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  char b[24];
  snprintf(b, sizeof(b), "X %+.1f mm", _m.xCm() * 10.0f);
  _cv->setCursor(3, 138);
  _cv->print(b);
  snprintf(b, sizeof(b), "Y %+.1f", _m.angleDeg());
  _cv->setCursor(3, 148);
  _cv->print(b);
  _cv->drawCircle(3 + strlen(b) * 6 + 3, 151, 2, WHITE);  // simbolo de grado pegado al numero
}

// Fila de valor (alto h, 21 por defecto; minimo 18), con tres aspectos:
//   editando    -> relleno del color del modulo, texto negro (el giro cambia el valor)
//   elegida     -> marco doble del color del modulo (el giro elige otra fila, el clic entra a editar)
//   normal      -> marco fino
// Etiqueta chica (hasta 5 caracteres) a la izquierda; valor grande (hasta 7 caracteres) desde x = 38.
void Ui::valueRow(int y, const char* label, const char* value, bool sel, bool edit, int h) {
  const uint16_t ac = accent();
  if (edit) _cv->fillRect(1, y, 126, h, ac);
  _cv->drawRect(1, y, 126, h, ac);
  if (sel && !edit) _cv->drawRect(2, y + 1, 124, h - 2, ac);
  _cv->setTextSize(1);
  _cv->setTextColor(edit ? BLACK : (sel ? ac : GREY));
  _cv->setCursor(5, y + (h - 7) / 2);
  _cv->print(label);
  _cv->setTextSize(2);
  _cv->setTextColor(edit ? BLACK : WHITE);
  _cv->setCursor(38, y + (h - 14) / 2);
  _cv->print(value);
}

void Ui::button(int y, int h, const char* text, bool sel) {
  if (sel) _cv->fillRect(1, y, 126, h, accent()); else _cv->drawRect(1, y, 126, h, accent());
  textCentered(64, y + (h - 7) / 2, text, 1, sel ? BLACK : WHITE);
}

// Pestana REGRESAR al pie de la pantalla.
void Ui::drawBackTab(bool selected) {
  const uint16_t ac = accent();
  if (selected) _cv->fillRect(1, 138, 126, 20, ac);
  _cv->drawRect(1, 138, 126, 20, ac);
  textCentered(64, 144, "< REGRESAR", 1, selected ? BLACK : WHITE);
}

// Una fila "etiqueta ........ estado": la etiqueta (hasta 10 caracteres) a la izquierda, en blanco, y el
// estado (hasta 9 caracteres) desde x = 69, en el color que se indique.
void Ui::statusRow(int y, const char* label, const char* status, uint16_t color) {
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  _cv->setCursor(3, y);
  _cv->print(label);
  _cv->setTextColor(color);
  _cv->setCursor(69, y);
  _cv->print(status);
}

// Recuadro pequeno con texto centrado (elegido: relleno).
void Ui::box(int x, int y, int w, int h, const char* text, bool sel) {
  if (sel) _cv->fillRect(x, y, w, h, accent()); else _cv->drawRect(x, y, w, h, accent());
  _cv->setTextSize(1);
  _cv->setTextColor(sel ? BLACK : WHITE);
  _cv->setCursor(x + (w - (int)strlen(text) * 6) / 2, y + (h - 7) / 2);
  _cv->print(text);
}

// Tres recuadros pequenos: HOME X, HOME Y y AMBOS (cada eje a su HOME marcado). sel0 = indice del primero elegido.
void Ui::homeBoxes(int y, int h, int sel) {
  static const char* const kHB[3] = {"HOME X", "HOME Y", "AMBOS"};
  for (int i = 0; i < 3; i++) box(1 + i * 43, y, 40, h, kHB[i], sel == i);
}

// Filas: X (mm del riel o pasos) y Y (grados o pasos) se mueven EN VIVO; H y V (mm de la hoja, con el origen de AJUSTES;
// solo con la hoja calibrada) solo se escriben y el boton IR A H, V mueve. Recuadros HOME X / HOME Y / AMBOS (un clic),
// UNIDAD (mm/grados <-> pasos) y LASER.
void Ui::drawMover() {
  header("MOVER");
  char v[24];
  if (_stepsMode) {  // modo PASOS: los pasos exactos de cada motor (X desde el HOME marcado)
    snprintf(v, sizeof(v), "%+ld", _tgtXs);
    valueRow(16, "X pas", v, _axisSel == 0, _editing && _axisSel == 0, 17);
    snprintf(v, sizeof(v), "%+d", _tgtYsteps);
    valueRow(34, "Y pas", v, _axisSel == 1, _editing && _axisSel == 1, 17);
  } else {
    snprintf(v, sizeof(v), "%+d", _tgtXmm);
    valueRow(16, "X mm", v, _axisSel == 0, _editing && _axisSel == 0, 17);
    snprintf(v, sizeof(v), "%+.2f", yDegOf(_tgtYsteps));
    valueRow(34, "Y grd", v, _axisSel == 1, _editing && _axisSel == 1, 17);
  }
  const bool sheet = _m.gridPoints() && _sheetValid;
  const char* fmt = _m.sheetCorner() ? "%.0f" : "%+.0f";
  if (sheet) snprintf(v, sizeof(v), fmt, _hMm + hShift(_m)); else snprintf(v, sizeof(v), "--");
  valueRow(52, "H mm", v, _axisSel == 2, _editing && _axisSel == 2, 17);
  if (sheet) snprintf(v, sizeof(v), fmt, _vMm + vShift(_m)); else snprintf(v, sizeof(v), "--");
  valueRow(70, "V mm", v, _axisSel == 3, _editing && _axisSel == 3, 17);
  button(88, 11, "IR A H, V", _axisSel == 4);
  homeBoxes(101, 11, _axisSel - 5);
  char u[12];
  if (_stepsMode) snprintf(u, sizeof(u), "PASOS x%d", _stepSize);
  else if (_mmMult > 1) snprintf(u, sizeof(u), "MM/GRD x%d", _mmMult);
  else snprintf(u, sizeof(u), "MM / GRD");
  box(1, 114, 62, 11, u, _axisSel == 8);  // un clic cambia la unidad y el tamano de paso
  box(65, 114, 62, 11, _m.laser() ? "LASER: ON" : "LASER: OFF", _axisSel == 9);  // un clic lo enciende o lo apaga

  // Posicion real. X siempre sale de contar pasos; Y sale del AS5600 si esta listo y con cero (ver SENSORES).
  // En modo PASOS se muestran los contadores de pasos de cada motor.
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  if (_stepsMode) {
    snprintf(v, sizeof(v), "Real X%+ld Y%+ld", _m.xStepsRel(), lroundf(Cfg::ANGLE_SIGN * _m.ySteps()));
  } else {
    const float yReal = (_enc.live() && _enc.hasZero()) ? _enc.relativeDeg() : _m.angleDeg();
    snprintf(v, sizeof(v), "Real X%+.1f Y%+.1f", _m.xCm() * 10.0f, yReal);
  }
  _cv->setCursor(3, 128);
  _cv->print(v);
  drawBackTab(_axisSel == 10);
}

// HOME: los mismos tres recuadros que MOVER (HOME X, HOME Y, AMBOS) al HOME marcado en CALIBRAR > HOME. Con un HOME o
// una referencia en curso se muestra un aviso y cualquier clic lo cancela.
void Ui::drawHome() {
  header("HOME");
  if (_m.homing()) {
    _cv->fillRect(1, 18, 126, 80, C_TEAL);
    textCentered(64, 32, _m.testing() ? "CALIBRANDO LIMITS" : "HOME EN CURSO", 1, BLACK);
    textCentered(64, 46, _m.homePhase(), 1, BLACK);
    _cv->drawRect(14, 64, 100, 22, BLACK);
    textCentered(64, 71, "CLIC = CANCELAR", 1, BLACK);
  } else {
    homeBoxes(20, 30, _homeSel);
    _cv->setTextSize(1);
    _cv->setTextColor(GREY);
    _cv->setCursor(3, 58);
    _cv->print("Van al HOME marcado");
    _cv->setCursor(3, 67);
    _cv->print("(CALIBRAR > HOME).");
    _cv->setCursor(3, 79);
    _cv->print("Y: lado mas corto.");
  }
  const bool encOk = _enc.live() && _enc.hasZero();
  statusRow(100, "Eje X", _m.xHomed() ? "referido" : "sin ref.", _m.xHomed() ? C_GREEN : C_RED);
  statusRow(109, "HOME X", _m.xHomeSet() ? "marcado" : "en X-", _m.xHomeSet() ? C_GREEN : WHITE);
  statusRow(118, "Eje Y", encOk ? "AS5600 ok" : "sin AS5600", encOk ? C_GREEN : C_RED);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 128);
  _cv->print(_m.homing() ? "" : "girar:elegir clic:ok");
  drawBackTab(_homeSel == 3);
}

// Estado de calibracion de una IMU en texto corto y con color (verde = bien, rojo = problema).
static const char* imuCalText(const Imu& imu, char* buf, size_t n, uint16_t& color) {
  color = WHITE;
  if (!imu.ok()) { color = C_RED; return "NO CONECT"; }
  switch (imu.state()) {
    case Imu::State::Running: snprintf(buf, n, "cal. %d%%", imu.progress()); color = C_ORANGE; return buf;
    case Imu::State::Failed:  color = C_RED; return "fallo";
    default: break;
  }
  if (imu.cal().valid) { color = C_GREEN; return "calibrada"; }
  return "sin calib";
}

// Toda la informacion de los componentes en una lista que se recorre con la perilla (12 renglones visibles); un clic
// regresa. Cada renglon: etiqueta (hasta 10 caracteres) y valor (hasta 9) con color.
void Ui::drawSensores() {
  header("SENSORES");
  _cv->setTextSize(1);
  int n = 0;  // renglon de la lista (no de la pantalla)
  char v[16], buf[20];
  auto row = [&](const char* label, const char* value, uint16_t color) {
    const int r = n++ - _sensTop;
    if (r >= 0 && r < SENS_ROWS) statusRow(18 + r * 10, label, value, color);
  };
  auto title = [&](const char* t) {
    const int r = n++ - _sensTop;
    if (r < 0 || r >= SENS_ROWS) return;
    _cv->setTextColor(C_PINK);
    _cv->setCursor(3, 18 + r * 10);
    _cv->print(t);
  };

  title("MOTOR X (riel)");
  snprintf(v, sizeof(v), "%d p/s", _m.speedX());                       row("Velocidad", v, WHITE);
  snprintf(v, sizeof(v), "%.2f", _m.speedX() * Cfg::X_MM_PER_STEP);    row("Vel. mm/s", v, WHITE);
  snprintf(v, sizeof(v), "%.1f:1", Cfg::xReductionEff());               row("Reduccion", v, WHITE);
  snprintf(v, sizeof(v), "%.5f", Cfg::X_MM_PER_STEP);                  row("mm/paso", v, WHITE);
  snprintf(v, sizeof(v), "%.0f", Cfg::STEPS_PER_REV);                  row("Pasos/vta", v, WHITE);  // del motor
  snprintf(v, sizeof(v), "%.0f", Cfg::STEPS_PER_REV * Cfg::xReductionEff()); row("Vta salida", v, WHITE);
  snprintf(v, sizeof(v), "%ld", _m.xSteps());                          row("Pasos pos.", v, WHITE);
  snprintf(v, sizeof(v), "%+.1f", _m.xCm() * 10.0f);                   row("Pos. mm", v, WHITE);
  if (_m.savedRailSteps() > 0) snprintf(v, sizeof(v), "%ld", _m.savedRailSteps()); else snprintf(v, sizeof(v), "sin medir");
  row("Riel pasos", v, _m.savedRailSteps() > 0 ? C_GREEN : WHITE);
  if (_m.savedRailSteps() > 0) snprintf(v, sizeof(v), "%.1f", _m.savedRailSteps() * Cfg::X_MM_PER_STEP); else snprintf(v, sizeof(v), "--");
  row("Riel mm", v, WHITE);
  row("Referencia", _m.xHomed() ? "calibrado" : "sin ref.", _m.xHomed() ? C_GREEN : C_RED);
  row("Sentido", _m.xInvertEffective() ? "invertido" : "normal", WHITE);
  snprintf(v, sizeof(v), "%lu", (unsigned long)_m.stepsX());          row("Pasos tot.", v, GREY);

  title("MOTOR Y (angulo)");
  snprintf(v, sizeof(v), "%d p/s", _m.speedY());                       row("Velocidad", v, WHITE);
  snprintf(v, sizeof(v), "%.1f", _m.speedY() * Cfg::Y_DEG_PER_STEP);   row("Vel. grd/s", v, WHITE);
  snprintf(v, sizeof(v), "%.1f:1", Cfg::yReductionEff());               row("Reduccion", v, WHITE);
  snprintf(v, sizeof(v), "%.4f", Cfg::Y_DEG_PER_STEP);                 row("grd/paso", v, WHITE);
  snprintf(v, sizeof(v), "%.0f", Cfg::STEPS_PER_REV);                  row("Pasos/vta", v, WHITE);
  snprintf(v, sizeof(v), "%.0f", Cfg::STEPS_PER_REV * Cfg::yReductionEff()); row("Vta salida", v, WHITE);
  snprintf(v, sizeof(v), "%ld", _m.ySteps());                          row("Pasos pos.", v, WHITE);
  snprintf(v, sizeof(v), "%+.2f", _m.angleDeg());                      row("Angulo", v, WHITE);
  row("Referencia", _m.yZeroed() ? "si" : "no", _m.yZeroed() ? C_GREEN : C_RED);
  snprintf(v, sizeof(v), "%lu", (unsigned long)_m.stepsY());          row("Pasos tot.", v, GREY);

  title("FINALES DE CARRERA");
  row("Fin X-", _m.limitXMin() ? "ACTIVADO" : "libre", _m.limitXMin() ? C_RED : C_GREEN);
  row("Fin X+", _m.limitXMax() ? "ACTIVADO" : "libre", _m.limitXMax() ? C_RED : C_GREEN);

  title("PASOS PERDIDOS");  // X: al tocar un final de carrera; Y: correccion con el AS5600 tras cada movimiento
  snprintf(v, sizeof(v), "%+ld", _m.xLostLast());                      row("X ult.", v, _m.xRefCount() ? WHITE : GREY);
  snprintf(v, sizeof(v), "%ld", _m.xLostTotal());                      row("X total", v, _m.xRefCount() ? WHITE : GREY);
  snprintf(v, sizeof(v), "%d", _m.xRefCount());                        row("X referenc", v, GREY);
  snprintf(v, sizeof(v), "%+ld", _m.yLostLast());                      row("Y ult.", v, _m.yTrimCount() ? WHITE : GREY);
  snprintf(v, sizeof(v), "%ld", _m.yLostTotal());                      row("Y total", v, _m.yTrimCount() ? WHITE : GREY);
  snprintf(v, sizeof(v), "%d", _m.yTrimCount());                       row("Y correg.", v, GREY);
  row("Y sensor", _m.yTrimBlocked() ? "AL REVES" : (_enc.live() && _enc.hasZero() ? "corrige" : "sin sensor"),
      _m.yTrimBlocked() ? C_RED : (_enc.live() && _enc.hasZero() ? C_GREEN : GREY));

  title("AFINAR");  // lo que dejaron las pruebas de CALIBRAR > AFINAR (tambien sale en el monitor serie al arrancar)
  snprintf(v, sizeof(v), "%.7f", Cfg::X_MM_PER_STEP);                  row("Escala X", v, WHITE);
  snprintf(v, sizeof(v), "%d", Cfg::X_BACKLASH_STEPS);                 row("Holg. X", v, WHITE);
  snprintf(v, sizeof(v), "%d", Cfg::Y_BACKLASH_STEPS);                 row("Holg. Y", v, WHITE);
  snprintf(v, sizeof(v), "%.1f mm", _m.afinarValue(0));                row("Esc.X ult", v, (_m.afinarDone() & 1) ? C_GREEN : GREY);
  snprintf(v, sizeof(v), "%+.1f mm", _m.afinarValue(1));               row("DifY ult", v, (_m.afinarDone() & 2) ? C_GREEN : GREY);
  snprintf(v, sizeof(v), "%+.1f mm", _m.afinarValue(2));               row("DifX ult", v, (_m.afinarDone() & 4) ? C_GREEN : GREY);

  {  // inclinacion: referencia (al marcar la hoja), ganancias aprendidas y lo que se esta corrigiendo ahora
    float cH, cV;
    const bool act = _m.tiltCorr(&cH, &cV);
    if (_m.tiltRefOk()) snprintf(v, sizeof(v), "%+.1f %+.1f", _m.tiltRefX(), _m.tiltRefY()); else snprintf(v, sizeof(v), "sin ref");
    row("Incl ref", v, _m.tiltRefOk() ? WHITE : GREY);
    if (_m.tiltGainOk(0)) snprintf(v, sizeof(v), "%+.2f", _m.tiltGain(0)); else snprintf(v, sizeof(v), "--");
    row("Gan H", v, _m.tiltGainOk(0) ? C_GREEN : GREY);
    if (_m.tiltGainOk(1)) snprintf(v, sizeof(v), "%+.2f", _m.tiltGain(1)); else snprintf(v, sizeof(v), "--");
    row("Gan V", v, _m.tiltGainOk(1) ? C_GREEN : GREY);
    snprintf(v, sizeof(v), "%+.1f mm", cH);
    row("Comp H", v, act ? WHITE : GREY);
    snprintf(v, sizeof(v), "%+.1f mm", cV);
    row("Comp V", v, act ? WHITE : GREY);
  }

  title("LASER");
  row("Estado", _m.laser() ? "ENCENDIDO" : "apagado", _m.laser() ? C_RED : GREY);

  title("AS5600 (angulo)");
  row("Conexion", _enc.present() ? "conectado" : "NO CONECT", _enc.present() ? C_GREEN : C_RED);
  row("Iman", _enc.present() ? (_enc.live() ? _enc.zone() : "sin iman") : "--", _enc.live() ? C_GREEN : C_RED);
  row("Cero", _enc.hasZero() ? "fijado" : "sin fijar", _enc.hasZero() ? C_GREEN : WHITE);
  if (_enc.live() && _enc.hasZero()) snprintf(v, sizeof(v), "%+.2f", _enc.relativeDeg()); else snprintf(v, sizeof(v), "--");
  row("Angulo", v, WHITE);

  // IMUs: la negra va en el extremo X- del riel y la blanca en el X+, horizontales con el chip hacia arriba.
  const Imu* imus[2] = {&_imuB, &_imuW};
  const char* titles[2] = {"IMU NEGRO (X-)", "IMU BLANCO (X+)"};
  float tl[2][2] = {{0, 0}, {0, 0}};
  bool tOk[2] = {false, false};
  for (int i = 0; i < 2; i++) {
    title(titles[i]);
    row("Conexion", imus[i]->ok() ? "conectada" : "NO CONECT", imus[i]->ok() ? C_GREEN : C_RED);
    uint16_t c;
    const char* t = imuCalText(*imus[i], buf, sizeof(buf), c);
    row("Calibrac.", imus[i]->ok() ? t : "--", imus[i]->ok() ? c : GREY);
    tOk[i] = imus[i]->tilt(&tl[i][0], &tl[i][1]);  // inclinacion a lo largo de cada eje del chip (grados)
    if (tOk[i]) snprintf(v, sizeof(v), "%+.2f", tl[i][0]); else snprintf(v, sizeof(v), "--");
    row("Incl chipX", v, WHITE);
    if (tOk[i]) snprintf(v, sizeof(v), "%+.2f", tl[i][1]); else snprintf(v, sizeof(v), "--");
    row("Incl chipY", v, WHITE);
  }
  title("INCLINACION EXTREMOS");  // promedio = inclinacion general; diferencia = cuanto cambia de un extremo al otro
  const bool both = tOk[0] && tOk[1];
  if (both) snprintf(v, sizeof(v), "%+.2f", (tl[0][0] + tl[1][0]) / 2); else snprintf(v, sizeof(v), "--");
  row("Prom chipX", v, WHITE);
  if (both) snprintf(v, sizeof(v), "%+.2f", (tl[0][1] + tl[1][1]) / 2); else snprintf(v, sizeof(v), "--");
  row("Prom chipY", v, WHITE);
  if (both) snprintf(v, sizeof(v), "%+.2f", tl[0][0] - tl[1][0]); else snprintf(v, sizeof(v), "--");
  row("Dif chipX", v, WHITE);  // negro (X-) menos blanco (X+)
  if (both) snprintf(v, sizeof(v), "%+.2f", tl[0][1] - tl[1][1]); else snprintf(v, sizeof(v), "--");
  row("Dif chipY", v, WHITE);

  title("HOJA");
  if (_m.gridPoints()) snprintf(v, sizeof(v), "%d", _m.gridPoints()); else snprintf(v, sizeof(v), "sin calib");
  row("Puntos", v, _m.gridPoints() ? C_GREEN : WHITE);
  snprintf(v, sizeof(v), "%.0fx%.0f", Cfg::SHEET_W_MM, Cfg::SHEET_H_MM);
  row("Tamano mm", v, WHITE);

  title("RED");
  row("Punto acc.", Net::enabled() ? "ON" : "OFF", Net::enabled() ? C_GREEN : GREY);
  snprintf(v, sizeof(v), "%d de 4", Net::clients());
  row("Equipos", v, WHITE);

  _sensLines = n;
  if (n > SENS_ROWS) {  // barra de desplazamiento a la derecha
    const int track = SENS_ROWS * 10, barH = max(6, track * SENS_ROWS / n);
    const int barY = 18 + (track - barH) * _sensTop / (n - SENS_ROWS);
    _cv->drawFastVLine(126, 18, track, GREY);
    _cv->fillRect(125, barY, 3, barH, C_PINK);
  }
  drawBackTab(true);
}

// Punto de acceso: boton para encenderlo / apagarlo (se recuerda al reiniciar) y, encendido, sus datos.
void Ui::drawRed() {
  header("RED");
  button(17, 17, Net::enabled() ? "PUNTO ACCESO: ON" : "PUNTO ACCESO: OFF", _redSel == 0);
  _cv->setTextSize(1);
  if (Net::enabled()) {
    _cv->setTextColor(WHITE);
    _cv->setCursor(3, 40);
    _cv->print("Red:");
    _cv->setTextSize(2);
    _cv->setCursor(3, 50);
    _cv->print(Net::ssid());
    _cv->setTextSize(1);
    _cv->setCursor(3, 72);
    _cv->printf("Clave: %s", Net::password());
    _cv->setCursor(3, 84);
    _cv->printf("IP: %s", Net::ip().c_str());
    _cv->setCursor(3, 96);
    _cv->printf("Equipos: %d de 4", Net::clients());
    _cv->setTextColor(Net::clients() ? C_GREEN : C_RED);
    _cv->setCursor(3, 108);
    _cv->print(Net::clients() ? "Equipo conectado" : "Esperando conexion");
  } else {
    _cv->setTextColor(WHITE);
    _cv->setCursor(3, 44);
    _cv->print("Wi-Fi apagado.");
    _cv->setTextColor(GREY);
    _cv->setCursor(3, 56);
    _cv->print("El LED ya no se pone");
    _cv->setCursor(3, 65);
    _cv->print("rojo por falta de");
    _cv->setCursor(3, 74);
    _cv->print("equipos conectados.");
  }
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 127);
  _cv->print("girar:fila clic:ok");
  drawBackTab(_redSel == 1);
}

void Ui::drawCalibrar() {
  header("CALIBRAR");
  for (int i = 0; i < CAL_N; i++) button(16 + i * 16, 14, kCal[i], i == _calSel);

  // Abajo, dos renglones (el detalle completo esta en SENSORES).
  _cv->setTextSize(1);
  if (_m.testing()) {  // calibrando los finales de carrera (por ejemplo, desde el monitor serie): un clic cancela
    _cv->setTextColor(WHITE);
    _cv->setCursor(3, 116);
    _cv->print("Calibrando limits");
    _cv->setCursor(3, 126);
    _cv->print(_m.homePhase());
  } else {
    const char* as = !_enc.present() ? "NO CONECT" : (!_enc.live() ? "sin iman" : (_enc.hasZero() ? "con cero" : "sin cero"));
    statusRow(116, "AS5600", as, !_enc.present() ? C_RED : (_enc.hasZero() ? C_GREEN : WHITE));
    char rail[12];
    if (_m.savedRailSteps() > 0) snprintf(rail, sizeof(rail), "%ld", _m.savedRailSteps()); else snprintf(rail, sizeof(rail), "sin medir");
    statusRow(126, "Pasos riel", rail, _m.savedRailSteps() > 0 ? C_GREEN : WHITE);  // largo guardado en la flash
  }
  drawBackTab(_calSel == CAL_N);
}

// AJUSTES: VELOCIDAD de cada eje, MEDIR Y ESCALA (contador libre y correccion de la escala de Y; la de X va en LIMITS) y
// ORIGEN de la hoja (un clic cambia entre centro y esquina inferior izquierda; solo cambia los numeros que se muestran).
void Ui::drawAjustes() {
  header("AJUSTES");
  button(17, 21, "VELOCIDAD", _ajSel == 0);
  button(41, 21, "MEDIR Y ESCALA", _ajSel == 1);
  button(65, 21, _m.sheetCorner() ? "ORIGEN: ESQUINA" : "ORIGEN: CENTRO", _ajSel == 2);
  char b[24];
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  snprintf(b, sizeof(b), "Vel. X %d p/s", _m.speedX());
  _cv->setCursor(3, 90);
  _cv->print(b);
  snprintf(b, sizeof(b), "Vel. Y %d p/s", _m.speedY());
  _cv->setCursor(3, 99);
  _cv->print(b);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 110);
  _cv->print(_m.sheetCorner() ? "(0,0) = esq. inf izq" : "(0,0) = centro");
  _cv->setCursor(3, 119);
  snprintf(b, sizeof(b), "Area H%.0f x V%.0f mm", Cfg::SHEET_W_MM, Cfg::SHEET_H_MM);
  _cv->print(b);
  _cv->setCursor(3, 128);
  _cv->print("Escala X: en LIMITS");
  drawBackTab(_ajSel == 3);
}

// MEDIR Y ESCALA: se marca un inicio, se mueven X y Y con la perilla (sin limites) y se ve en vivo cuantos pasos dio cada
// motor y cuanto CREE el equipo que avanzo (Y tambien con el AS5600). Para Y se escribe lo real (propone el AS5600) y
// APLICAR Y corrige grados por paso. La escala de X se corrige en CALIBRAR > LIMITS con el riel completo (mas exacto).
void Ui::drawEscala() {
  header("ESCALA");  // el titulo no cabe mas largo (a la derecha van W, MOV y X?)
  char v[24];
  snprintf(v, sizeof(v), "%+d", _tgtXmm);
  valueRow(16, "X mm", v, _escSel == 0, _editing && _escSel == 0, 17);
  snprintf(v, sizeof(v), "%+.2f", yDegOf(_tgtYsteps));
  valueRow(34, "Y grd", v, _escSel == 1, _editing && _escSel == 1, 17);
  button(52, 11, "MARCAR INICIO", _escSel == 2);
  const long dx = _m.xSteps() - _measX0;
  const long dy = lroundf(Cfg::ANGLE_SIGN * (_m.ySteps() - _measY0));  // en el sentido del angulo que se muestra
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  snprintf(v, sizeof(v), "X%+ld p %+.1fmm", dx, dx * Cfg::X_MM_PER_STEP);
  _cv->setCursor(3, 65);
  _cv->print(v);
  snprintf(v, sizeof(v), "Y%+ld p %+.2f", dy, dy * Cfg::Y_DEG_PER_STEP);
  _cv->setCursor(3, 74);
  _cv->print(v);
  if (_measEncOk && _enc.live() && _enc.hasZero()) {  // AS5600: solo vale para giros de menos de media vuelta
    float e = _enc.relativeDeg() - _measEnc0;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    snprintf(v, sizeof(v), "%+.2f", e);
  } else {
    snprintf(v, sizeof(v), "--");
  }
  statusRow(83, "Y AS5600", v, C_GREEN);
  snprintf(v, sizeof(v), "%.2f", _realY);
  valueRow(93, "Y real", v, _escSel == 3, _editing && _escSel == 3, 17);
  box(1, 112, 62, 11, "APLICAR Y", _escSel == 4);
  box(65, 112, 62, 11, "FABRICA", _escSel == 5);
  _cv->setTextColor(GREY);
  snprintf(v, sizeof(v), "Y %.5f grd/p", Cfg::Y_DEG_PER_STEP);
  _cv->setCursor(3, 127);
  _cv->print(v);
  drawBackTab(_escSel == 6);
}

// PUNTOS: dibujo del area (X horizontal, Y vertical, arriba = angulo negativo) con los puntos guardados (cuadritos), el
// elegido (marco blanco) y donde esta ahora el laser (cruz, o circulo rojo si esta encendido). Se guardan hasta 8.
void Ui::drawGuardados() {
  header("GUARDADOS");
  const int mx = 2, my = 17, mw = 124, mh = 52;
  _cv->drawRect(mx, my, mw, mh, C_VIOLET);
  long x0, x1, y0, y1;
  _m.mapRange(&x0, &x1, &y0, &y1);
  const float padx = (x1 - x0) * 0.06f + 1, pady = (y1 - y0) * 0.06f + 1;  // margen para que los bordes se vean
  const float fx0 = x0 - padx, fx1 = x1 + padx, fy0 = y0 - pady, fy1 = y1 + pady;
  auto px = [&](long x) { return mx + 3 + (int)(constrain((x - fx0) / (fx1 - fx0), 0.0f, 1.0f) * (mw - 7)); };
  auto py = [&](long y) { return my + 3 + (int)(constrain((y - fy0) / (fy1 - fy0), 0.0f, 1.0f) * (mh - 7)); };
  for (int i = 0; i < Machine::kMaxSaved; i++) {
    if (!_m.pointSaved(i)) continue;
    const int cx = px(_m.pointXSteps(i)), cy = py(_m.pointYSteps(i));
    _cv->fillRect(cx - 1, cy - 1, 3, 3, C_VIOLET);
    if (i == _gSlot) _cv->drawRect(cx - 3, cy - 3, 7, 7, WHITE);  // el elegido
  }
  if (_m.xHomed()) {  // donde esta el laser ahora (sin referencia de X la posicion no vale)
    const int cx = px(_m.xSteps()), cy = py(_m.yStepsShown());
    if (_m.laser()) _cv->fillCircle(cx, cy, 2, C_RED);
    else { _cv->drawFastHLine(cx - 2, cy, 5, WHITE); _cv->drawFastVLine(cx, cy - 2, 5, WHITE); }
  } else {
    _cv->setTextSize(1);
    _cv->setTextColor(C_RED);
    _cv->setCursor(mx + 18, my + 22);
    _cv->print("X sin referencia");
  }
  char b[28];
  const bool saved = _m.pointSaved(_gSlot);
  if (_editing && _gSel == 0) snprintf(b, sizeof(b), "<PUNTO %d: %s>", _gSlot + 1, saved ? "guardado" : "libre");
  else snprintf(b, sizeof(b), "PUNTO %d: %s", _gSlot + 1, saved ? "guardado" : "libre");
  button(72, 13, b, _gSel == 0);
  button(86, 12, "GUARDAR AQUI", _gSel == 1);
  button(99, 12, "IR AL PUNTO", _gSel == 2);
  button(112, 12, "BORRAR PUNTO", _gSel == 3);
  _cv->setTextSize(1);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 126);
  _cv->print(_editing ? "girar:punto clic:ok" : "girar:fila clic:ok");
  drawBackTab(_gSel == 4);
}

// X/Y MIN Y MAX: se acomoda el laser con X y Y (SIN limites) y cada boton guarda ahi ese tope.
// CALIBRAR > HOME (MARCAR): como MOVER pero sin limites; MARCAR deja ahi el HOME (X = 0 y/o Y = 0; para Y sobrescribe
// el cero del AS5600). REFERENCIAR X: el HOME por los finales de carrera (da la referencia de X despues de reiniciar).
void Ui::drawHMark() {
  header("HOME (MARCAR)");
  char v[24];
  snprintf(v, sizeof(v), "%+d", _tgtXmm);
  valueRow(17, "X mm", v, _hmSel == 0, _editing && _hmSel == 0, 19);
  snprintf(v, sizeof(v), "%+.2f", yDegOf(_tgtYsteps));
  valueRow(37, "Y grd", v, _hmSel == 1, _editing && _hmSel == 1, 19);
  button(58, 12, "MARCAR AMBOS AQUI", _hmSel == 2);
  box(1, 71, 62, 12, "MARCAR X", _hmSel == 3);
  box(65, 71, 62, 12, "MARCAR Y", _hmSel == 4);
  button(85, 12, "REFERENCIAR X", _hmSel == 5);
  const bool encOk = _enc.live();
  statusRow(102, "Ref. X", _m.xHomed() ? "si" : "falta", _m.xHomed() ? C_GREEN : C_RED);
  statusRow(111, "AS5600", encOk ? "ok" : "sin iman", encOk ? C_GREEN : C_RED);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 125);
  _cv->print(_editing ? "girar:valor clic:ok" : "marcar = X/Y en 0");
  drawBackTab(_hmSel == 6);
}

void Ui::drawMinMax() {
  header("MIN Y MAX");
  char v[24];
  snprintf(v, sizeof(v), "%+d", _tgtXmm);
  valueRow(17, "X mm", v, _mmSel == 0, _editing && _mmSel == 0, 19);
  snprintf(v, sizeof(v), "%+.2f", yDegOf(_tgtYsteps));
  valueRow(37, "Y grd", v, _mmSel == 1, _editing && _mmSel == 1, 19);
  static const char* const kName[4] = {"X MIN", "X MAX", "Y MIN", "Y MAX"};
  for (int i = 0; i < 4; i++) {
    if (!_m.topeSet(i)) snprintf(v, sizeof(v), "%s: --", kName[i]);
    else if (i < 2) snprintf(v, sizeof(v), "%s: %+.1f mm", kName[i], _m.topeValue(i) * 10.0f);
    else snprintf(v, sizeof(v), "%s: %+.1f grd", kName[i], _m.topeValue(i));
    button(58 + i * 13, 12, v, _mmSel == 2 + i);
  }
  button(110, 12, "BORRAR TOPES", _mmSel == 6);
  _cv->setTextSize(1);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 126);
  _cv->print(_editing ? "girar:valor clic:ok" : "clic tope = aqui");
  drawBackTab(_mmSel == 7);
}

void Ui::markMeasureStart() {
  _measMarked = true;
  _measX0 = _m.xSteps();
  _measY0 = _m.ySteps();
  _measEncOk = _enc.live() && _enc.hasZero();
  _measEnc0 = _measEncOk ? _enc.relativeDeg() : 0.0f;
}

// Velocidad de cada eje (pasos/s): clic para editar, girar cambia de SPS_UI_STEP en SPS_UI_STEP, otro clic la aplica
// y la guarda en la flash.
void Ui::drawVelocidad() {
  header("VELOCIDAD");
  char v[8], b[24];
  snprintf(v, sizeof(v), "%d", _spdX);
  valueRow(17, "X p/s", v, _spdSel == 0, _editing && _spdSel == 0);
  snprintf(v, sizeof(v), "%d", _spdY);
  valueRow(39, "Y p/s", v, _spdSel == 1, _editing && _spdSel == 1);
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  snprintf(b, sizeof(b), "X = %.2f mm/s", _spdX * Cfg::X_MM_PER_STEP);
  _cv->setCursor(3, 66);
  _cv->print(b);
  snprintf(b, sizeof(b), "Y = %.1f grd/s", _spdY * Cfg::Y_DEG_PER_STEP);
  _cv->setCursor(3, 76);
  _cv->print(b);
  _cv->setTextColor(GREY);
  snprintf(b, sizeof(b), "De %d a %d p/s,", Cfg::SPS_UI_MIN, Cfg::SPS_UI_MAX);
  _cv->setCursor(3, 90);
  _cv->print(b);
  snprintf(b, sizeof(b), "de %d en %d. Se", Cfg::SPS_UI_STEP, Cfg::SPS_UI_STEP);
  _cv->setCursor(3, 99);
  _cv->print(b);
  _cv->setCursor(3, 110);
  _cv->print("guarda al dar clic.");
  _cv->setCursor(3, 127);
  _cv->print(_editing ? "girar:valor clic:ok" : "girar:fila clic:edit");
  drawBackTab(_spdSel == 2);
}

// Fase de la calibracion de limits, ultima muestra y como cancelar (4 lineas desde y).
void Ui::drawLimitProgress(int y) {
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  _cv->setCursor(3, y);
  _cv->print("Calibrando limits");
  _cv->setCursor(3, y + 9);
  _cv->print(_m.homePhase());
  if (_m.legCount() > 0) {  // pasos de la ultima muestra terminada
    char lb[24];
    snprintf(lb, sizeof(lb), "muestra %d: %ld", _m.legCount(), _m.legSteps(_m.legCount() - 1));
    _cv->setCursor(3, y + 18);
    _cv->print(lb);
  }
  _cv->setTextColor(GREY);
  _cv->setCursor(3, y + 27);
  _cv->print("clic = cancelar");
}

// CALIBRAR LIMITS: se eligen las corridas (1 a LIMIT_CAL_MAX_RUNS) y se inicia; abajo, el avance o las muestras.
// Riel real que se propone/escribe en LIMITS: lo que escribio el usuario, o si no lo que cree el equipo (pasos x escala).
float Ui::limRealShown() const {
  return _limRealSet ? _limReal : _m.savedRailSteps() * Cfg::X_MM_PER_STEP;
}

// CALIBRAR LIMITS: corridas, INICIAR, y al terminar el resultado: pasos del riel, DISPERSION entre corridas (si pierde pasos,
// las corridas no se parecen), el riel REAL medido con regla (se escribe y APLICAR ESCALA corrige mm por paso) y las muestras.
// Al terminar, el carro queda en el CENTRO del riel y ahi X = 0.
void Ui::drawLimits() {
  header("LIMITS");
  const bool run = _m.testing();
  char v[16];
  snprintf(v, sizeof(v), "%d", _limRuns);
  valueRow(17, "Corr.", v, !run && _limSel == 0, _editing && _limSel == 0, 19);
  button(37, 13, "INICIAR", !run && _limSel == 1);
  if (run) {
    drawLimitProgress(54);
    drawBackTab(false);
    return;
  }
  const long rail = _m.savedRailSteps();
  snprintf(v, sizeof(v), "%ld", rail);
  statusRow(54, "Pasos riel", rail > 0 ? v : "sin medir", rail > 0 ? C_GREEN : WHITE);
  // dispersion: diferencia entre la corrida mas larga y la mas corta (solo con 2 o mas corridas)
  if (_m.legCount() >= 2 && rail > 0) {
    long lo = _m.legSteps(0), hi = lo;
    for (int i = 1; i < _m.legCount(); i++) { lo = min(lo, _m.legSteps(i)); hi = max(hi, _m.legSteps(i)); }
    const float pct = 100.0f * (hi - lo) / rail;
    snprintf(v, sizeof(v), "%ld (%.1f%%)", hi - lo, pct);
    statusRow(63, "Dif corr.", v, pct > 0.5f ? C_RED : (pct > 0.2f ? C_ORANGE : C_GREEN));  // >0.5 % = pierde pasos
  } else {
    statusRow(63, "Dif corr.", "--", GREY);
  }
  snprintf(v, sizeof(v), "%.1f", limRealShown());
  valueRow(72, "Real", v, _limSel == 2, _editing && _limSel == 2, 19);  // mm reales de un final de carrera al otro
  button(92, 12, "APLICAR ESCALA", _limSel == 3);
  _cv->setTextSize(1);
  _cv->setTextColor(C_ORANGE);
  snprintf(v, sizeof(v), "Cree %.1f mm", rail * Cfg::X_MM_PER_STEP);
  _cv->setCursor(3, 106);
  _cv->print(v);
  _cv->setTextColor(WHITE);  // las ultimas muestras, de dos en dos
  const int n = _m.legCount(), first = max(0, n - 4);
  for (int i = first; i < n; i += 2) {
    char lb[28];
    if (i + 1 < n) snprintf(lb, sizeof(lb), "%d:%ld %d:%ld", i + 1, _m.legSteps(i), i + 2, _m.legSteps(i + 1));
    else snprintf(lb, sizeof(lb), "%d:%ld", i + 1, _m.legSteps(i));
    _cv->setCursor(3, 115 + ((i - first) / 2) * 9);
    _cv->print(lb);
  }
  drawBackTab(_limSel == 4);
}

// ARRANQUE: al encender, X no sabe donde esta el carro. Se ofrece referenciarlo (nada se mueve hasta dar clic).
void Ui::drawArranque() {
  header("ARRANQUE");
  _cv->setTextSize(1);
  _cv->setTextColor(WHITE);
  _cv->setCursor(3, 20);  _cv->print("Al apagar se pierde");
  _cv->setCursor(3, 29);  _cv->print("donde esta el carro.");
  _cv->setCursor(3, 41);  _cv->print("REFERENCIAR busca un");
  _cv->setCursor(3, 50);  _cv->print("final de carrera y");
  _cv->setCursor(3, 59);  _cv->print("lo lleva al HOME.");
  _cv->setTextColor(C_ORANGE);
  _cv->setCursor(3, 71);  _cv->print("Revisa que no haya");
  _cv->setCursor(3, 80);  _cv->print("nada en el riel.");
  button(96, 22, "REFERENCIAR AHORA", _bootSel == 0);
  button(122, 22, "DESPUES", _bootSel == 1);
}

// PUNTOS DE LA HOJA: se eligen 4, 6 o 9 puntos y se inicia el marcado (pantalla MARCAR): INICIAR marca todos, AGREGAR conserva
// los que ya estan (4 -> 6 o 9, 6 -> 9) y solo pide los nuevos.
void Ui::drawPuntos() {
  header("HOJA");
  char v[8];
  snprintf(v, sizeof(v), "%d", _ptsN);
  valueRow(17, "Pts", v, _ptsSel == 0, _editing && _ptsSel == 0);
  button(40, 15, "INICIAR (TODOS)", _ptsSel == 1);
  button(57, 15, "AGREGAR PUNTOS", _ptsSel == 2);  // conserva los puntos vigentes: solo se marcan los nuevos
  char b[24];
  if (_m.gridPoints()) snprintf(b, sizeof(b), "%d puntos", _m.gridPoints()); else snprintf(b, sizeof(b), "sin calib");
  statusRow(75, "Hoja", b, _m.gridPoints() ? C_GREEN : WHITE);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 85);
  _cv->print(_ptsN == 4 ? "4 esquinas" : (_ptsN == 6 ? "esq+centro arr/abajo" : "esq+lados+centro"));
  _cv->setTextColor(WHITE);
  snprintf(b, sizeof(b), "Area H%.0f x V%.0f mm", Cfg::SHEET_W_MM, Cfg::SHEET_H_MM);
  _cv->setCursor(3, 95);
  _cv->print(b);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 104);
  _cv->print(_m.sheetCorner() ? "(0,0) = esq. inf izq" : "(0,0) = centro");
  // Requisitos para marcar: X calibrado (para que los puntos sigan valiendo al reiniciar) y Y con referencia.
  const char* req = !_m.xHomed() ? "Falta calibrar X" : (!_m.yZeroed() ? "Y sin referencia" : "Listo para marcar");
  _cv->setTextColor(_m.xHomed() && _m.yZeroed() ? C_GREEN : C_RED);
  _cv->setCursor(3, 116);
  _cv->print(req);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 127);
  _cv->print(_editing ? "girar:4/6/9 clic:ok" : "girar:fila clic:ok");
  drawBackTab(_ptsSel == 3);
}

// Marcado guiado: dice que punto toca; se acomoda el laser con X y Y (SIN limite de angulo) y OK lo guarda y pasa al
// siguiente. REGRESAR cancela (la calibracion anterior queda igual).
void Ui::drawMarcar() {
  char b[24];
  snprintf(b, sizeof(b), "PUNTO %d/%d", _ptIdx + 1, _ptsN);
  header(b);
  textCentered(64, 19, Machine::pointName(_ptsN, _ptIdx), 2, WHITE);
  float h, v;
  Machine::pointSheetMm(_ptsN, _ptIdx, &h, &v);  // donde cae esa esquina en el area util (centro = 0)
  snprintf(b, sizeof(b), _m.sheetCorner() ? "hoja H%.1f V%.1f" : "hoja H%+.1f V%+.1f", h + hShift(_m), v + vShift(_m));
  _cv->setTextSize(1);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 37);
  _cv->print(b);
  char val[12];
  snprintf(val, sizeof(val), "%+d", _tgtXmm);
  valueRow(47, "X mm", val, _markSel == 0, _editing && _markSel == 0);
  snprintf(val, sizeof(val), "%+.2f", yDegOf(_tgtYsteps));
  valueRow(69, "Y grd", val, _markSel == 1, _editing && _markSel == 1);
  button(92, 15, "OK - GUARDAR PUNTO", _markSel == 2);
  button(109, 14, _m.laser() ? "LASER: ON" : "LASER: OFF", _markSel == 3);  // para ver el punto al marcar
  _cv->setTextSize(1);
  _cv->setTextColor(GREY);
  _cv->setCursor(3, 127);
  _cv->print(_editing ? "girar:valor clic:ok" : "girar:fila clic:ok");
  drawBackTab(_markSel == 4);
}

// ---- AFINAR: pruebas guiadas que corrigen escala de X y holgura de X y Y --------------------------------------------
// El equipo lleva el laser a puntos conocidos, TU mides en la hoja con regla y escribes lo medido con la perilla; el equipo
// calcula el ajuste y lo guarda en la flash.
//   ESCALA X : va a V -100 y V +100 (200 mm pedidos, sin la correccion de las esquinas). Se escribe la distancia REAL entre las
//              dos marcas; mm por paso nuevo = mm por paso actual x real / 200.
//   HOLGURA  : llega al mismo punto (H 50 o V 50) desde un lado y desde el otro. DIF = marca llegando desde + menos marca llegando
//              desde - (en mm, con signo, en el sentido en que crece H o V). Si el juego de los engranes no esta bien compensado,
//              cada llegada queda corrida la mitad de la DIF hacia donde venia: holgura nueva = actual + DIF/2 en pasos.
void Ui::afinarGo(float h, float v, float h2, float v2) {
  const char* err = "";
  if (!_m.moveToSheet(h, v, &err, 'S')) { say(err); return; }
  _afH2 = h2;
  _afV2 = v2;
  _afPend = true;
  say("Yendo al lado...");
}

void Ui::afinarClick() {
  const char* err = "";
  const int k = _afTest - 3;  // prueba guiada 0 = ESCALA X, 1 = HOLGURA Y, 2 = HOLGURA X (el resto de la lista abre otra pantalla)
  if (!_afIn) {
    switch (_afTest) {
      case 0: _screen = NIVEL; return;                                                        // nivel con las IMUs
      case 1: _screen = LIMITS; _limSel = 0; _editing = false; return;                       // riel y centro
      case 2: _screen = HMARK; _hmSel = 0; _editing = false; adoptTarget(); return;           // HOME de X y Y
      case 8: _screen = PUNTOS; _ptsSel = 0; _editing = false; return;                        // esquinas
      case 9: _screen = CALIBRAR; return;
      default: break;
    }
    _afIn = true;
    _afSel = 0;
    _editing = false;
    _afPend = false;
    _afReal = (k == 0) ? 200.0f : 0.0f;
    _afT0ok = _afT1ok = false;
    return;
  }
  if (_afSel == 4) { _afIn = false; _editing = false; _afPend = false; return; }
  if (_afSel == 2) { _editing = !_editing; return; }
  _editing = false;
  if (_afSel == 3) {  // APLICAR
    if (k >= 3) {  // INCLINACION: ganancia = salto del punto (mm) / cambio de inclinacion del eje del chip que mas cambio
      if (!_afT0ok || !_afT1ok) { say("Marca inicio y fin"); return; }
      const float d0 = _afT1x - _afT0x, d1 = _afT1y - _afT0y;
      const int ax = fabsf(d0) >= fabsf(d1) ? 0 : 1;
      const float dd = ax == 0 ? d0 : d1;
      if (fabsf(dd) < 0.5f) { say("Inclina mas (0.5 grd)"); return; }
      if (fabsf(_afReal) < 0.2f) { say("Escribe el salto"); return; }
      const int other = (k == 3) ? 1 : 0;  // la otra direccion debe usar el otro eje del chip
      if (_m.tiltGainOk(other) && _m.tiltAxis(other) == ax) { say("Inclina en otra direccion"); return; }
      if (_m.setTiltGain(k - 3, ax, _afReal / dd)) {
        _m.afinarSave(k, _afReal);
        say("Ganancia guardada");
        _afReal = 0;
        _afT0ok = _afT1ok = false;
      } else {
        say("Valor fuera de rango");
      }
    } else if (k == 0) {
      if (_afReal < 150.0f || _afReal > 250.0f) { say("Valor fuera de rango"); return; }
      if (_m.setScales(Cfg::X_MM_PER_STEP * _afReal / 200.0f, 0)) {
        _m.afinarSave(0, _afReal);
        say("Escala X guardada");
        _afReal = 200.0f;
      } else {
        say("Valor fuera de rango");
      }
    } else {
      long add;
      if (k == 1) {  // mm de H a grados de Y (pendiente de la tabla cerca de H 50) y a pasos
        const float degPerMm = (SheetMap::hToYdeg(55.0f) - SheetMap::hToYdeg(45.0f)) / 10.0f;
        add = lroundf(_afReal / 2.0f * degPerMm / Cfg::Y_DEG_PER_STEP);
      } else {
        add = lroundf(_afReal / 2.0f / Cfg::X_MM_PER_STEP);
      }
      const int nx = Cfg::X_BACKLASH_STEPS + (k == 2 ? (int)add : 0);
      const int ny = Cfg::Y_BACKLASH_STEPS + (k == 1 ? (int)add : 0);
      char b[26];
      if (_m.setBacklash(nx, ny)) {
        _m.afinarSave(k, _afReal);
        snprintf(b, sizeof(b), "Holgura %c = %d pasos", k == 1 ? 'Y' : 'X', k == 1 ? ny : nx);
        say(b);
        _afReal = 0;
      } else {
        say("Valor fuera de rango");
      }
    }
    return;
  }
  if (k >= 3) {  // INCLINACION: MARCAR INICIO / MARCAR FIN guardan la inclinacion de las IMUs de ese momento
    float px, py, dx, dy;
    if (levelInfo(&px, &py, &dx, &dy) == 0) { say("Sin IMU"); return; }
    if (_afSel == 0) { _afT0x = px; _afT0y = py; _afT0ok = true; _afT1ok = false; say("Inicio marcado"); }
    else if (!_afT0ok) say("Marca el inicio primero");
    else { _afT1x = px; _afT1y = py; _afT1ok = true; say("Fin marcado"); }
    return;
  }
  // IR 1 / IR 2
  if (k == 0) {
    const float v = (_afSel == 0) ? -100.0f : 100.0f;
    if (_m.jogTo((_m.originMm() + v) / 10.0f, _m.targetAngleDeg(), &err, 'S')) say(_afSel == 0 ? "En V -100: marca" : "En V +100: marca");
    else say(err);
  } else if (k == 1) {
    if (_afSel == 0) afinarGo(85, 0, 50, 0);  // llega a H 50 desde H mayor (desde +)
    else afinarGo(15, 0, 50, 0);              // desde H menor (desde -)
  } else {
    if (_afSel == 0) afinarGo(0, 85, 0, 50);
    else afinarGo(0, 15, 0, 50);
  }
}

// Lista de pasos en el orden recomendado, con una palomita en los que ya estan hechos. Los 3 de en medio son pruebas guiadas
// (se pueden repetir cuando sea, cada una parte de lo vigente); los otros abren la pantalla que ya existia.
void Ui::drawAfinar() {
  header("AFINAR");
  _cv->setTextSize(1);
  if (!_afIn) {
    static const char* const names[9] = {"1 NIVEL (IMUs)", "2 LIMITS (RIEL)", "3 HOME X Y", "4 ESCALA DE X", "5 HOLGURA DE Y",
                                         "6 HOLGURA DE X", "7 INCLINAC. H", "8 INCLINAC. V", "9 ESQUINAS"};
    static const char* const desc[9][3] = {{"Revisa con las IMUs", "que la base este", "nivelada (burbuja)."},
                                           {"Mide el riel entre", "los finales y deja el", "centro como origen."},
                                           {"Marca el HOME de X y", "de Y: ahi V 0 y H 0.", "Despues del paso 2."},
                                           {"Mide cuanto avanza X", "de verdad (200 mm) y", "corrige mm por paso."},
                                           {"Llega al mismo punto", "desde los 2 lados y", "ajusta holgura de Y."},
                                           {"Llega al mismo punto", "desde los 2 lados y", "ajusta holgura de X."},
                                           {"Inclina la base con", "un calzo y mide el", "salto del punto en H."},
                                           {"Inclina la base con", "un calzo y mide el", "salto del punto en V."},
                                           {"Marca 4, 6 o 9 puntos", "de la hoja: corrige el", "error que quede."}};
    const uint8_t ad = _m.afinarDone();
    float lpx, lpy, ldx, ldy;
    const int ln = levelInfo(&lpx, &lpy, &ldx, &ldy);
    const bool done[9] = {ln > 0 && fabsf(lpx) <= LEVEL_TOL_DEG && fabsf(lpy) <= LEVEL_TOL_DEG, _m.savedRailSteps() > 0,
                          _m.xHomeSet() && _m.yZeroed(), (ad & 1) != 0, (ad & 2) != 0, (ad & 4) != 0, _m.tiltGainOk(0), _m.tiltGainOk(1),
                          _m.gridPoints() > 0 && !_m.gridStale()};
    const int top = constrain(_afTest - 5, 0, 3);  // lista de 6 renglones que se recorre
    char lab[24];
    for (int i = top; i < top + 6; i++) {
      snprintf(lab, sizeof(lab), "%s%s", done[i] ? "\xFB " : "  ", names[i]);  // \xFB = palomita en la fuente de la pantalla
      button(15 + (i - top) * 12, 11, lab, i == _afTest);
    }
    char b[26] = "";
    if (_afTest < 9) {
      _cv->setTextColor(WHITE);
      for (int i = 0; i < 3; i++) { _cv->setCursor(3, 101 + i * 9); _cv->print(desc[_afTest][i]); }
      switch (_afTest) {
        case 0: if (ln > 0) snprintf(b, sizeof(b), "X %+.2f  Y %+.2f grd", lpx, lpy); else snprintf(b, sizeof(b), "sin IMU"); break;
        case 1: if (_m.savedRailSteps() > 0) snprintf(b, sizeof(b), "Riel %ld pasos", _m.savedRailSteps()); else snprintf(b, sizeof(b), "sin medir"); break;
        case 2: snprintf(b, sizeof(b), "HOME X %s  Y %s", _m.xHomeSet() ? "si" : "no", _m.yZeroed() ? "si" : "no"); break;
        case 3: snprintf(b, sizeof(b), "%.7f mm/p", Cfg::X_MM_PER_STEP);
                if (ad & 1) snprintf(b + strlen(b), sizeof(b) - strlen(b), " %.0f", _m.afinarValue(0));
                break;
        case 4: snprintf(b, sizeof(b), "Holg Y %d", Cfg::Y_BACKLASH_STEPS);
                if (ad & 2) snprintf(b + strlen(b), sizeof(b) - strlen(b), " ult %+.1f", _m.afinarValue(1));
                break;
        case 5: snprintf(b, sizeof(b), "Holg X %d", Cfg::X_BACKLASH_STEPS);
                if (ad & 4) snprintf(b + strlen(b), sizeof(b) - strlen(b), " ult %+.1f", _m.afinarValue(2));
                break;
        case 6:
        case 7: {
          const int w = _afTest - 6;
          if (_m.tiltGainOk(w)) snprintf(b, sizeof(b), "%+.2f mm/grd eje %c", _m.tiltGain(w), _m.tiltAxis(w) ? 'Y' : 'X');
          else snprintf(b, sizeof(b), "sin hacer");
          break;
        }
        default: if (_m.gridPoints() == 0) snprintf(b, sizeof(b), "sin marcar");
                 else snprintf(b, sizeof(b), "%d puntos%s", _m.gridPoints(), _m.gridStale() ? " VIEJA" : "");
                 break;
      }
      _cv->setTextColor(GREY);
      _cv->setCursor(3, 128);
      _cv->print(b);
    }
    drawBackTab(_afTest == 9);
    return;
  }
  const int k = _afTest - 3;
  static const char* const txt0[4] = {"Marca el punto del", "laser en IR -100 y en", "IR +100; mide entre", "las 2 marcas (mm)."};
  static const char* const txt1[4] = {"Marca el punto del", "laser en cada llegada", "DIF = marca desde +", "menos marca desde -"};
  static const char* const txt2[4] = {"Marca punto, INICIO,", "inclina con calzo,", "FIN y SALTO: cuanto", "se movio (mm, signo)"};
  const char* const* t = k == 0 ? txt0 : (k >= 3 ? txt2 : txt1);
  _cv->setTextColor(WHITE);
  for (int i = 0; i < 4; i++) { _cv->setCursor(3, 17 + i * 9); _cv->print(t[i]); }
  char l0[22], l1[22], l2[22], l3[22];
  if (k == 0) {
    snprintf(l0, sizeof(l0), "IR A V -100");
    snprintf(l1, sizeof(l1), "IR A V +100");
    snprintf(l2, sizeof(l2), _editing ? "REAL: <%.1f> mm" : "REAL: %.1f mm", _afReal);
    snprintf(l3, sizeof(l3), "APLICAR ESCALA");
  } else if (k >= 3) {
    float px, py, dx, dy;
    const int n = levelInfo(&px, &py, &dx, &dy);
    snprintf(l0, sizeof(l0), _afT0ok ? "\xFB MARCAR INICIO" : "MARCAR INICIO");
    if (_afT0ok && n > 0) snprintf(l1, sizeof(l1), "FIN dX%+.1f dY%+.1f", px - _afT0x, py - _afT0y);
    else snprintf(l1, sizeof(l1), "MARCAR FIN");
    snprintf(l2, sizeof(l2), _editing ? "SALTO: <%+.1f> mm" : "SALTO: %+.1f mm", _afReal);
    snprintf(l3, sizeof(l3), "APLICAR (%c)", k == 3 ? 'H' : 'V');
  } else {
    snprintf(l0, sizeof(l0), "LLEGAR DESDE +");
    snprintf(l1, sizeof(l1), "LLEGAR DESDE -");
    snprintf(l2, sizeof(l2), _editing ? "DIF: <%+.1f> mm" : "DIF: %+.1f mm", _afReal);
    snprintf(l3, sizeof(l3), "APLICAR (ahora %d)", k == 1 ? Cfg::Y_BACKLASH_STEPS : Cfg::X_BACKLASH_STEPS);
  }
  button(56, 15, l0, _afSel == 0);
  button(72, 15, l1, _afSel == 1);
  button(88, 15, l2, _afSel == 2);
  button(104, 15, l3, _afSel == 3);
  button(120, 15, "ATRAS", _afSel == 4);
}

// Inclinacion de las IMUs (grados respecto a donde se calibraron: hay que calibrarlas sobre una superficie nivelada).
// px / py = promedio de las IMUs con lectura (eje X / Y del chip); dx / dy = negro (X-) menos blanco (X+). Devuelve cuantas hay.
int Ui::levelInfo(float* px, float* py, float* dx, float* dy) const {
  float t[2][2] = {{0, 0}, {0, 0}};
  const bool ok[2] = {_imuB.tilt(&t[0][0], &t[0][1]), _imuW.tilt(&t[1][0], &t[1][1])};
  int n = 0;
  *px = *py = *dx = *dy = 0;
  for (int i = 0; i < 2; i++)
    if (ok[i]) { *px += t[i][0]; *py += t[i][1]; n++; }
  if (n) { *px /= n; *py /= n; }
  if (n == 2) { *dx = t[0][0] - t[1][0]; *dy = t[0][1] - t[1][1]; }
  return n;
}

// NIVEL: burbuja con las IMUs (ejes del chip). Centrada y verde = base nivelada (dentro de LEVEL_TOL_DEG). 1 grado = 14 puntos.
void Ui::drawNivel() {
  header("NIVEL");
  float px, py, dx, dy;
  const int n = levelInfo(&px, &py, &dx, &dy);
  const int cx = 64, cy = 62, R = 40;
  const float ppd = 14.0f;
  _cv->drawCircle(cx, cy, R, WHITE);
  _cv->drawCircle(cx, cy, (int)(LEVEL_TOL_DEG * ppd) + 4, GREY);
  _cv->drawFastHLine(cx - R, cy, 2 * R, GREY);
  _cv->drawFastVLine(cx, cy - R, 2 * R, GREY);
  const bool level = n > 0 && fabsf(px) <= LEVEL_TOL_DEG && fabsf(py) <= LEVEL_TOL_DEG;
  if (n > 0) {
    float bx = px * ppd, by = -py * ppd;
    const float len = sqrtf(bx * bx + by * by), maxLen = R - 6.0f;
    if (len > maxLen) { bx *= maxLen / len; by *= maxLen / len; }
    const float mag = max(fabsf(px), fabsf(py));
    _cv->fillCircle(cx + (int)bx, cy + (int)by, 5, level ? C_GREEN : (mag < 1.0f ? C_ORANGE : C_RED));
  }
  _cv->setTextSize(1);
  char b[26];
  if (n == 0) {
    textCentered(64, 106, "SIN IMU", 1, C_RED);
  } else {
    textCentered(64, 106, level ? "NIVELADO" : "INCLINADO", 1, level ? C_GREEN : C_ORANGE);
    _cv->setTextColor(WHITE);
    snprintf(b, sizeof(b), "Prom X%+.2f Y%+.2f", px, py);
    _cv->setCursor(3, 116);
    _cv->print(b);
    if (n == 2) snprintf(b, sizeof(b), "Dif  X%+.2f Y%+.2f", dx, dy); else snprintf(b, sizeof(b), "Dif  (falta 1 IMU)");
    _cv->setCursor(3, 125);
    _cv->print(b);
  }
  drawBackTab(true);
}

void Ui::render() {
  _cv->fillScreen(BLACK);
  switch (_screen) {
    case MENU:     drawMenu(); break;
    case MOVER:    drawMover(); break;
    case HOME:     drawHome(); break;
    case SENSORES: drawSensores(); break;
    case RED:      drawRed(); break;
    case CALIBRAR: drawCalibrar(); break;
    case LIMITS:   drawLimits(); break;
    case PUNTOS:   drawPuntos(); break;
    case MARCAR:   drawMarcar(); break;
    case AJUSTES:  drawAjustes(); break;
    case ESCALA:   drawEscala(); break;
    case ARRANQUE: drawArranque(); break;
    case GUARDADOS: drawGuardados(); break;
    case VELOCIDAD: drawVelocidad(); break;
    case MINMAX:   drawMinMax(); break;
    case HMARK:    drawHMark(); break;
    case AFINAR:   drawAfinar(); break;
    case NIVEL:    drawNivel(); break;
  }
  if (_msgUntil) {  // aviso temporal en la franja de abajo
    _cv->fillRect(0, 147, 128, 13, WHITE);
    _cv->setTextSize(1);
    _cv->setTextColor(BLACK);
    _cv->setCursor(3, 151);
    _cv->print(_msg);
  }
  // Solo se manda a la pantalla si el cuadro cambio (el SPI es lo mas lento).
  uint32_t h = 2166136261u;
  const uint8_t* p = (const uint8_t*)_cv->getBuffer();
  for (int i = 0; i < 128 * 160 * 2; i++) h = (h ^ p[i]) * 16777619u;
  if (h != _frameHash) {
    _frameHash = h;
    _tft->drawRGBBitmap(0, 0, _cv->getBuffer(), 128, 160);
  }
}
