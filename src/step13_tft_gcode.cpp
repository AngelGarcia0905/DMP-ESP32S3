// Paso 13: terminal de G-code en la pantalla TFT ST7735 (solo prueba, no mueve motores).
//   Escribe lineas de G-code en el monitor serie (termina con Enter); la pantalla muestra
//   la linea y como la interpreto el parser, mas la posicion y el estado del laser.
//   Cableado ESP32-S3: VCC->3V3, GND->GND, SCK->GPIO12, SDA->GPIO11, CS->GPIO10, DC->GPIO13, RES->GPIO21, BL->3V3
//   Comandos: G0/G1 X Y F, G28, G90, G91, M3, M5, M114, HELP
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Arduino.h>
#include <SPI.h>

#include "dmp_pins.h"  // pines de la pantalla en el ESP32-S3

constexpr uint8_t TFT_CS = Pins::TFT_CS;
constexpr uint8_t TFT_DC = Pins::TFT_DC;
constexpr uint8_t TFT_RST = Pins::TFT_RST;
constexpr uint8_t TAB_TYPE = INITR_BLACKTAB;

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

constexpr int COLS = 21;       // 128 px / 6 px por caracter
constexpr int LOG_LINES = 13;  // lineas visibles del registro
constexpr int LOG_Y = 30;      // donde empieza el registro

struct LogLine { char text[COLS + 1]; uint16_t color; };
static LogLine logBuf[LOG_LINES];
static int logCount = 0;

// Estado simulado de la maquina.
static float posX = 0, posY = 0, feed = 1000;
static bool absolute = true;
static bool laserOn = false;

static void drawStatus() {
  tft.fillRect(0, 0, tft.width(), LOG_Y - 2, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(2, 2);
  tft.print("G-code  (prueba)");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2, 12);
  tft.printf("X%.1f Y%.1f %s", posX, posY, absolute ? "G90" : "G91");
  tft.setCursor(2, 21);
  tft.setTextColor(laserOn ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("Laser %s  F%.0f", laserOn ? "ON " : "OFF", feed);
  tft.drawFastHLine(0, LOG_Y - 3, tft.width(), ST77XX_BLUE);
}

static void drawLog() {
  tft.fillRect(0, LOG_Y, tft.width(), tft.height() - LOG_Y, ST77XX_BLACK);
  tft.setTextSize(1);
  for (int i = 0; i < logCount; i++) {
    tft.setTextColor(logBuf[i].color);
    tft.setCursor(2, LOG_Y + i * 10);
    tft.print(logBuf[i].text);
  }
}

static void addLine(const char* s, uint16_t color) {
  if (logCount == LOG_LINES) {
    memmove(&logBuf[0], &logBuf[1], sizeof(LogLine) * (LOG_LINES - 1));
    logCount--;
  }
  strncpy(logBuf[logCount].text, s, COLS);
  logBuf[logCount].text[COLS] = 0;
  logBuf[logCount].color = color;
  logCount++;
}

static void show(const char* cmd, const char* result, bool ok) {
  addLine(cmd, ST77XX_CYAN);
  char buf[48];
  snprintf(buf, sizeof(buf), " %s %s", ok ? ">" : "!", result);
  addLine(buf, ok ? ST77XX_GREEN : ST77XX_RED);
  drawStatus();
  drawLog();
  Serial.printf("%-28s -> %s%s\n", cmd, ok ? "" : "ERROR: ", result);
}

// Devuelve el valor de una letra en la linea (ej. 'X'), o false si no esta.
static bool word(const char* line, char letter, float& value) {
  for (const char* p = line; *p; p++) {
    if (*p == letter && (p == line || p[-1] == ' ')) {
      value = atof(p + 1);
      return true;
    }
  }
  return false;
}

static void handle(const char* raw) {
  // Quita comentarios (; ...) y pasa a mayusculas.
  char line[64];
  size_t n = 0;
  for (const char* p = raw; *p && n < sizeof(line) - 1; p++) {
    if (*p == ';') break;
    line[n++] = toupper((unsigned char)*p);
  }
  line[n] = 0;
  while (n && line[n - 1] == ' ') line[--n] = 0;
  if (!n) return;

  char res[40];
  float v;

  if (!strcmp(line, "HELP")) {
    show(line, "G0 G1 G28 G90 G91", true);
    show("", "M3 M5 M114", true);
    return;
  }

  // Numero de comando: primer token tipo G0, G1, M3...
  char kind = line[0];
  int num = atoi(line + 1);
  if ((kind != 'G' && kind != 'M') || !isdigit((unsigned char)line[1])) {
    show(line, "comando no valido", false);
    return;
  }

  if (kind == 'G' && (num == 0 || num == 1)) {
    float x = posX, y = posY;
    if (word(line, 'X', v)) x = absolute ? v : posX + v;
    if (word(line, 'Y', v)) y = absolute ? v : posY + v;
    if (word(line, 'F', v)) feed = v;
    posX = x;
    posY = y;
    snprintf(res, sizeof(res), "mover X%.1f Y%.1f", x, y);
    show(line, res, true);
  } else if (kind == 'G' && num == 28) {
    posX = posY = 0;
    show(line, "home X0 Y0", true);
  } else if (kind == 'G' && num == 90) {
    absolute = true;
    show(line, "coord. absolutas", true);
  } else if (kind == 'G' && num == 91) {
    absolute = false;
    show(line, "coord. relativas", true);
  } else if (kind == 'M' && num == 3) {
    laserOn = true;
    show(line, "laser ON", true);
  } else if (kind == 'M' && num == 5) {
    laserOn = false;
    show(line, "laser OFF", true);
  } else if (kind == 'M' && num == 114) {
    snprintf(res, sizeof(res), "X%.1f Y%.1f", posX, posY);
    show(line, res, true);
  } else {
    show(line, "no soportado", false);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  // Pines propios del S3, sin MISO (-1): evita que el GPIO 13 (DC) se use como MISO.
  SPI.begin(Pins::TFT_SCK, -1, Pins::TFT_MOSI, -1);
  tft.initR(TAB_TYPE);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("\n=== Paso 13: terminal G-code en TFT ===");
  Serial.println("Escribe G-code y Enter. Ejemplos: G28 | G1 X10 Y5 F500 | M3 | M5 | M114 | HELP");
  drawStatus();
  addLine("Escribe G-code en", ST77XX_WHITE);
  addLine("el monitor serie", ST77XX_WHITE);
  addLine("Ej: G1 X10 Y5", ST77XX_WHITE);
  addLine("HELP = comandos", ST77XX_WHITE);
  drawLog();
}

void loop() {
  static char buf[64];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (len) {
        buf[len] = 0;
        handle(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
