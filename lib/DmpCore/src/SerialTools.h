#pragma once
#include <Arduino.h>
#include "Stepper28.h"

// Tiny serial console used by the single-motor test steps.
namespace SerialTools {

inline void motorHelp(const char* name) {
  Serial.printf("\n== %s motor test ==\n", name);
  Serial.println("  f / b : move 512 half-steps forward / backward");
  Serial.println("  F / B : run continuously forward / backward");
  Serial.println("  s     : stop and release the coils");
  Serial.println("  + / - : faster / slower");
  Serial.println("  0     : set the position counter to zero");
}

inline void motorPoll(Stepper28& m, float& sps) {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'f': m.moveBy(512); break;
      case 'b': m.moveBy(-512); break;
      case 'F': m.moveBy(10000000L); break;
      case 'B': m.moveBy(-10000000L); break;
      case 's': m.stop(); m.release(); break;
      case '+': sps = min(sps + 50.0f, 700.0f); m.setSpeed(sps); break;
      case '-': sps = max(sps - 50.0f, 50.0f); m.setSpeed(sps); break;
      case '0': m.setPosition(0); break;
      default: continue;
    }
    Serial.printf("pos=%ld  target=%ld  speed=%.0f steps/s\n", m.position(), m.target(), sps);
  }
}

}  // namespace SerialTools
