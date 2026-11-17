#pragma once
#include <math.h>
#include <stdint.h>

// Reglas puras (sin Arduino, se prueban en la PC) para no perder pasos:
//   Y: despues de un movimiento se compara el angulo pedido con el del AS5600 y se corrige la diferencia (yTrim).
//   X: no tiene sensor; cada vez que el carro toca un final de carrera, la posicion contada se compara con la que le
//      toca a ese interruptor (xLostAtLimit): la diferencia son los pasos perdidos y el contador se vuelve a referenciar.
namespace StepGuard {

enum class TrimAct : uint8_t { Ok, Fix, TooBig };
struct Trim { TrimAct act; long steps; };  // steps = pasos del motor que se piden como correccion (con signo)

// errDeg = angulo pedido - angulo del sensor (grados mostrados). Dentro de tolDeg no se toca nada; mas de maxDeg no se
// confia en el sensor (podria estar mal montado o desconectado) y no se corrige. angleSign = Cfg::ANGLE_SIGN.
inline Trim yTrim(float errDeg, float tolDeg, float maxDeg, float degPerStep, float angleSign) {
  const float a = fabsf(errDeg);
  if (a <= tolDeg) return {TrimAct::Ok, 0};
  if (a > maxDeg) return {TrimAct::TooBig, 0};
  return {TrimAct::Fix, lroundf(angleSign * errDeg / degPerStep)};
}

// Despues de una correccion el error NO debe crecer: si crece mas que margin, el sensor y el motor giran en sentidos
// contrarios (ENC_INVERT) y seguir corrigiendo empeoraria todo.
inline bool trimWorse(float errBeforeDeg, float errAfterDeg, float marginDeg) {
  return fabsf(errAfterDeg) > fabsf(errBeforeDeg) + marginDeg;
}

// Pasos perdidos de X al tocar un final de carrera: posicion contada - posicion que le toca al interruptor
// (X- = 0, X+ = largo del riel). Positivo = el contador iba adelantado hacia X+.
inline long xLostAtLimit(long counted, long expected) { return counted - expected; }

}  // namespace StepGuard
