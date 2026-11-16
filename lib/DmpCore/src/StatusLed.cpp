#include "StatusLed.h"

#include "dmp_pins.h"

namespace {
constexpr uint8_t kLevel = 200;  // brillo (0-255); estaba en 40 y casi no se veia
const uint8_t kPins[2] = {Pins::RGB_A, Pins::RGB_B};
rmt_obj_t* g_rmt[2] = {nullptr, nullptr};
}  // namespace

void StatusLed::begin() {
  for (int i = 0; i < 2; i++) {
    g_rmt[i] = rmtInit(kPins[i], RMT_TX_MODE, RMT_MEM_64);
    if (g_rmt[i]) rmtSetTick(g_rmt[i], 100);  // tick de 100 ns
  }
  set(Color::Red);
}

void StatusLed::set(Color c) {
  if (!_first && c == _color) return;
  _first = false;
  _color = c;
  switch (c) {
    case Color::Red:   write(kLevel, 0, 0); break;
    case Color::Blue:  write(0, 0, kLevel); break;
    case Color::Green: write(0, kLevel, 0); break;
    default:           write(0, 0, 0); break;
  }
}

void StatusLed::write(uint8_t r, uint8_t g, uint8_t b) {
  rmt_data_t data[24];
  const uint8_t grb[3] = {g, r, b};  // el WS2812 espera el orden verde, rojo, azul
  int n = 0;
  for (int c = 0; c < 3; c++) {
    for (int bit = 7; bit >= 0; bit--) {
      const bool one = grb[c] & (1 << bit);
      data[n].level0 = 1;
      data[n].duration0 = one ? 8 : 4;  // 0.8 us / 0.4 us en alto
      data[n].level1 = 0;
      data[n].duration1 = one ? 4 : 8;  // 0.4 us / 0.8 us en bajo
      n++;
    }
  }
  for (int i = 0; i < 2; i++) {
    if (g_rmt[i]) rmtWriteBlocking(g_rmt[i], data, 24);
  }
}
