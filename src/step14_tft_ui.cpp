// Paso 14: interfaz de la pantalla TFT estilo impresora 3D (SOLO PRUEBA: no mueve motores).
//   Fondo negro, texto blanco, menu en cuadricula, todo con la perilla KY-040:
//     girar = moverse / cambiar valor      pulsar = entrar / cambiar de eje      mantener = volver
//   Ejes:  X = motor a pasos, en mm (pasos de 1 mm)      Y = angulo del laser, +/- grados (pasos de 0.1)
//   La posicion "real" se simula contando pasos a SPS_MAX (igual que lo hara el firmware completo).
//   Cableado: pantalla en dmp_pins.h (TFT_*), perilla KNOB_CLK / KNOB_DT / KNOB_SW.
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Arduino.h>
#include <SPI.h>

#include "dmp_config.h"
#include "dmp_pins.h"

// ---- Ajustes de la interfaz -------------------------------------------------------------
constexpr bool     KNOB_INVERT   = false;  // true si al girar a la derecha el valor baja
constexpr int      X_STEP_MM     = 1;      // un "clic" de la perilla = 1 mm
constexpr int      Y_STEP_TENTHS = 1;      // un "clic" = 0.1 grado (en decimas de grado)
constexpr int      X_FAST_MULT   = 5;      // girando rapido: 5 mm por clic
constexpr int      Y_FAST_MULT   = 10;     // girando rapido: 1 grado por clic
constexpr uint32_t FAST_MS       = 45;     // dos clics mas juntos que esto = giro rapido
constexpr uint32_t LONG_PRESS_MS = 600;    // mantener pulsado = volver

constexpr uint16_t WHITE = 0xFFFF, BLACK = 0x0000, GREY = 0x8410;

constexpr uint8_t TAB_TYPE = INITR_BLACKTAB;
Adafruit_ST7735 tft(Pins::TFT_CS, Pins::TFT_DC, Pins::TFT_RST);
GFXcanvas16 cv(128, 160);  // se dibuja aqui y se manda completo: sin parpadeo

// ---- Estado -----------------------------------------------------------------------------
enum Screen { MENU, MOVER, INFO_SENSORES, INFO_RED, INFO_CALIB };
static Screen screen = MENU;
static int menuSel = 0;          // 0..5
static int axisSel = 0;          // 0 = X, 1 = Y
static bool laserOn = false;
static bool dirty = true;
static const char* kMenu[6] = {"MOVER", "HOME", "LASER", "CALIBRAR", "SENSORES", "RED"};

// Objetivo (lo que pide la perilla)
static int tgtXmm = 0;           // mm
static int tgtYtenths = 0;       // decimas de grado

// Posicion "real" simulada: se cuenta en pasos (como el firmware real)
static long stepsX = 0, stepsY = 0;
static float accX = 0, accY = 0;
static uint32_t lastSim = 0;

static float xMmReal() { return stepsX * Cfg::X_MM_PER_STEP; }
static float yDegReal() { return stepsY * Cfg::Y_DEG_PER_STEP; }

static long tgtStepsX() { return lroundf(tgtXmm / Cfg::X_MM_PER_STEP); }
static long tgtStepsY() { return lroundf(tgtYtenths * 0.1f / Cfg::Y_DEG_PER_STEP); }

static bool moving() { return stepsX != tgtStepsX() || stepsY != tgtStepsY(); }

static void simulate() {
  const uint32_t now = micros();
  const float dt = (now - lastSim) * 1e-6f;
  lastSim = now;
  accX += Cfg::SPS_MAX * dt;
  accY += Cfg::SPS_MAX * dt;
  // cada eje avanza hasta SPS_MAX pasos por segundo, hacia su objetivo
  const long ax = (long)accX, ay = (long)accY;
  accX -= ax; accY -= ay;
  long ex = tgtStepsX() - stepsX, ey = tgtStepsY() - stepsY;
  if (ex) { stepsX += constrain(ex, -ax, ax); }
  if (ey) { stepsY += constrain(ey, -ay, ay); }
}

// ---- Dibujo -----------------------------------------------------------------------------
static void textCentered(int cx, int y, const char* s, int size, uint16_t color) {
  const int w = strlen(s) * 6 * size;
  cv.setTextSize(size);
  cv.setTextColor(color);
  cv.setCursor(cx - w / 2, y);
  cv.print(s);
}

static void degreeMark(int x, int y, uint16_t color) { cv.drawCircle(x, y, 2, color); }

static void header(const char* title) {
  cv.setTextSize(1);
  cv.setTextColor(WHITE);
  cv.setCursor(3, 3);
  cv.print(title);
  cv.setCursor(86, 3);
  cv.print("SIM");
  if (laserOn) cv.fillCircle(120, 6, 4, WHITE); else cv.drawCircle(120, 6, 4, WHITE);
  cv.drawFastHLine(0, 13, 128, WHITE);
}

static void drawMenu() {
  header("DMP");
  char label[12];
  for (int i = 0; i < 6; i++) {
    const int c = i % 2, r = i / 2;
    const int x = 1 + c * 64, y = 17 + r * 38, w = 62, h = 34;
    const bool sel = (i == menuSel);
    if (sel) cv.fillRect(x, y, w, h, WHITE); else cv.drawRect(x, y, w, h, WHITE);
    const char* t = kMenu[i];
    if (i == 2) { snprintf(label, sizeof(label), "LASER %s", laserOn ? "ON" : "OFF"); t = label; }
    textCentered(x + w / 2, y + 13, t, 1, sel ? BLACK : WHITE);
  }
  cv.drawFastHLine(0, 133, 128, WHITE);
  cv.setTextSize(1);
  cv.setTextColor(WHITE);
  cv.setCursor(3, 138);
  cv.printf("X %+6.1f mm", xMmReal());
  cv.setCursor(3, 148);
  cv.printf("Y %+6.1f ", yDegReal());
  degreeMark(66, 151, WHITE);
}

// Un bloque de eje: relleno blanco si esta seleccionado (texto negro), contorno si no.
static void axisBlock(int y, const char* name, const char* value, bool isY, bool sel) {
  const uint16_t bg = sel ? WHITE : BLACK, fg = sel ? BLACK : WHITE;
  cv.fillRect(1, y, 126, 44, bg);
  cv.drawRect(1, y, 126, 44, WHITE);
  cv.setTextSize(1);
  cv.setTextColor(fg);
  cv.setCursor(5, y + 4);
  cv.print(name);
  cv.setTextSize(3);
  cv.setCursor(5, y + 17);
  cv.print(value);
  if (isY) degreeMark(5 + strlen(value) * 18 + 4, y + 20, fg);
}

static void drawMover() {
  header("MOVER");
  char v[12];
  snprintf(v, sizeof(v), "%+d", tgtXmm);
  axisBlock(17, "X  mm   (motor)", v, false, axisSel == 0);
  snprintf(v, sizeof(v), "%+.1f", tgtYtenths / 10.0f);
  axisBlock(65, "Y  grados (angulo)", v, true, axisSel == 1);

  cv.setTextSize(1);
  cv.setTextColor(WHITE);
  cv.setCursor(3, 114);
  cv.print("REAL (pasos)");
  cv.setCursor(3, 124);
  cv.printf("X %+7.1f mm", xMmReal());
  cv.setCursor(3, 134);
  cv.printf("Y %+7.2f ", yDegReal());
  degreeMark(75, 137, WHITE);
  if (moving()) { cv.setCursor(94, 124); cv.print("MOVE"); }
  cv.setTextColor(GREY);
  cv.setCursor(3, 146);
  cv.print("girar:valor  clic:eje");
  cv.setCursor(3, 153);
  cv.print("mantener: volver");
}

static void drawInfo(const char* title, const char* l1, const char* l2, const char* l3) {
  header(title);
  cv.setTextSize(1);
  cv.setTextColor(WHITE);
  cv.setCursor(3, 24);  cv.print(l1);
  cv.setCursor(3, 36);  cv.print(l2);
  cv.setCursor(3, 48);  cv.print(l3);
  cv.setTextColor(GREY);
  cv.setCursor(3, 146);
  cv.print("(pantalla de prueba)");
  cv.setCursor(3, 153);
  cv.print("mantener: volver");
}

static void render() {
  cv.fillScreen(BLACK);
  switch (screen) {
    case MENU:          drawMenu(); break;
    case MOVER:         drawMover(); break;
    case INFO_SENSORES: drawInfo("SENSORES", "IMU blanco:  --", "IMU negro:   --", "AS5600:      --"); break;
    case INFO_RED:      drawInfo("RED", "Red:  DMPS3", "IP:   192.168.4.1", "Equipos: --"); break;
    case INFO_CALIB:    drawInfo("CALIBRAR", "Fijar cero AS5600", "Calibrar IMUs", "(aun sin funcion)"); break;
  }
  tft.drawRGBBitmap(0, 0, cv.getBuffer(), 128, 160);
}

// ---- Perilla KY-040 ---------------------------------------------------------------------
static uint8_t knobPrev = 0;
static int8_t knobAcc = 0;
static uint32_t lastDetent = 0;

// Devuelve -1, 0 o +1 (un "clic" completo) y deja en fast si el giro es rapido.
static int readKnob(bool& fast) {
  static const int8_t T[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  const uint8_t cur = (digitalRead(Pins::KNOB_CLK) << 1) | digitalRead(Pins::KNOB_DT);
  if (cur != knobPrev) {
    knobAcc += T[(knobPrev << 2) | cur];
    knobPrev = cur;
    if (knobAcc >= 4 || knobAcc <= -4) {
      int dir = knobAcc > 0 ? 1 : -1;
      knobAcc = 0;
      const uint32_t now = millis();
      fast = (now - lastDetent) < FAST_MS;
      lastDetent = now;
      return KNOB_INVERT ? -dir : dir;
    }
  }
  return 0;
}

static void onTurn(int dir, bool fast) {
  if (screen == MENU) {
    menuSel = (menuSel + dir + 6) % 6;
  } else if (screen == MOVER) {
    if (axisSel == 0) {
      tgtXmm = constrain(tgtXmm + dir * X_STEP_MM * (fast ? X_FAST_MULT : 1),
                         (int)Cfg::X_MIN_CM * 10, (int)Cfg::X_MAX_CM * 10);
    } else {
      tgtYtenths = constrain(tgtYtenths + dir * Y_STEP_TENTHS * (fast ? Y_FAST_MULT : 1),
                             (int)(Cfg::Y_MIN_DEG * 10), (int)(Cfg::Y_MAX_DEG * 10));
    }
  }
  dirty = true;
}

static void onClick() {
  if (screen == MENU) {
    switch (menuSel) {
      case 0: screen = MOVER; break;
      case 1: tgtXmm = 0; tgtYtenths = 0; break;  // HOME simulado: vuelve a 0,0
      case 2: laserOn = !laserOn; break;
      case 3: screen = INFO_CALIB; break;
      case 4: screen = INFO_SENSORES; break;
      case 5: screen = INFO_RED; break;
    }
  } else if (screen == MOVER) {
    axisSel ^= 1;
  }
  dirty = true;
}

static void onLongPress() {
  if (screen != MENU) { screen = MENU; dirty = true; }
}

static void pollButton() {
  static bool down = false, longDone = false;
  static uint32_t tDown = 0, tLast = 0;
  const bool pressed = (digitalRead(Pins::KNOB_SW) == LOW);
  const uint32_t now = millis();
  if (now - tLast < 25) return;  // antirrebote
  if (pressed && !down) { down = true; longDone = false; tDown = now; tLast = now; }
  else if (pressed && down && !longDone && now - tDown >= LONG_PRESS_MS) { longDone = true; onLongPress(); }
  else if (!pressed && down) { down = false; tLast = now; if (!longDone) onClick(); }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(Pins::KNOB_CLK, INPUT_PULLUP);
  pinMode(Pins::KNOB_DT, INPUT_PULLUP);
  pinMode(Pins::KNOB_SW, INPUT_PULLUP);
  knobPrev = (digitalRead(Pins::KNOB_CLK) << 1) | digitalRead(Pins::KNOB_DT);

  SPI.begin(Pins::TFT_SCK, -1, Pins::TFT_MOSI, -1);  // sin MISO: GPIO 13 es DC
  tft.initR(TAB_TYPE);
  tft.setRotation(0);  // vertical 128x160
  tft.fillScreen(BLACK);
  lastSim = micros();
  Serial.println("\n=== Paso 14: interfaz TFT (prueba, sin motores) ===");
  Serial.println("Perilla: girar / pulsar / mantener. Sin perilla conectada no pasa nada (pines con pull-up).");
  render();
}

void loop() {
  bool fast = false;
  const int d = readKnob(fast);
  if (d) onTurn(d, fast);
  pollButton();

  simulate();
  static uint32_t lastDraw = 0;
  static bool wasMoving = false;
  const uint32_t now = millis();
  const bool mv = moving();
  if (dirty || (mv && now - lastDraw > 80) || (wasMoving && !mv)) {  // ultimo cuadro al llegar
    wasMoving = mv;
    dirty = false;
    lastDraw = now;
    render();
  }
}
