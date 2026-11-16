// Paso 12: pantalla TFT ST7735 128x160 (SPI) - prueba de colores y texto.
//   ESP32-S3: VCC->3V3, GND->GND, SCK->GPIO12, SDA(MOSI)->GPIO11, CS->GPIO10, DC->GPIO13, RES->GPIO21, BL->3V3
// Si los colores salen cambiados o hay una franja en el borde, prueba otro TAB_TYPE abajo.
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Arduino.h>
#include <SPI.h>

#include "dmp_pins.h"  // pines de la pantalla en el ESP32-S3

constexpr uint8_t TFT_CS = Pins::TFT_CS;
constexpr uint8_t TFT_DC = Pins::TFT_DC;
constexpr uint8_t TFT_RST = Pins::TFT_RST;

// Opciones: INITR_BLACKTAB, INITR_GREENTAB, INITR_REDTAB
constexpr uint8_t TAB_TYPE = INITR_BLACKTAB;

// SPI por hardware (SPI2/FSPI del S3): SCK = 12, MOSI = 11 (se fijan en setup()).
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

struct Page { const char* name; uint16_t color; uint16_t text; };
static const Page pages[] = {
    {"ROJO", ST77XX_RED, ST77XX_WHITE},
    {"VERDE", ST77XX_GREEN, ST77XX_BLACK},
    {"AZUL", ST77XX_BLUE, ST77XX_WHITE},
    {"BLANCO", ST77XX_WHITE, ST77XX_BLACK},
    {"NEGRO", ST77XX_BLACK, ST77XX_WHITE},
};
static uint8_t idx = 0;

static void drawInfo() {
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), ST77XX_WHITE);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(8, 10);
  tft.print("DMP laser");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 36);
  tft.printf("ST7735 %dx%d", tft.width(), tft.height());
  tft.setCursor(8, 48);
  tft.print("Texto: Aa Bb 0123456789");
  // Barras de color para revisar el orden RGB.
  const uint16_t bars[] = {ST77XX_RED, ST77XX_GREEN, ST77XX_BLUE, ST77XX_CYAN, ST77XX_MAGENTA, ST77XX_YELLOW};
  int w = (tft.width() - 16) / 6;
  for (int i = 0; i < 6; i++) tft.fillRect(8 + i * w, 70, w, 30, bars[i]);
  tft.setCursor(8, 110);
  tft.print("Rojo Verde Azul Cian");
  tft.setCursor(8, 122);
  tft.print("Magenta Amarillo");
  tft.setCursor(8, 140);
  tft.print("Siguiente: colores...");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Paso 12: TFT ST7735 ===");
  // Pines propios del S3, sin MISO (-1): evita que el GPIO 13 (DC) se use como MISO.
  SPI.begin(Pins::TFT_SCK, -1, Pins::TFT_MOSI, -1);
  tft.initR(TAB_TYPE);
  tft.setRotation(0);
  Serial.printf("Pantalla iniciada: %d x %d\n", tft.width(), tft.height());
  drawInfo();
  delay(4000);
}

void loop() {
  const Page& p = pages[idx];
  tft.fillScreen(p.color);
  tft.setTextColor(p.text);
  tft.setTextSize(2);
  tft.setCursor(10, tft.height() / 2 - 8);
  tft.print(p.name);
  Serial.printf("Color: %s\n", p.name);
  idx = (idx + 1) % (sizeof(pages) / sizeof(pages[0]));
  delay(1500);
  if (idx == 0) {
    drawInfo();
    delay(4000);
  }
}
