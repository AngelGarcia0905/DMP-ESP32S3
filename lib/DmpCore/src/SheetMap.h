#pragma once
#include <math.h>

// Conversion H / V de la hoja (mm) -> ejes. Funciones puras (sin Arduino) para poder probarlas en la PC.
//   V -> motor X (carro): el desplazamiento de X en mm es igual a V en mm.
//   H -> motor Y (pivote del laser): H se escribe en mm y se convierte en el angulo que necesita Y.
// La tabla de H es la MEDIDA en la maquina: angulo que calcula el equipo contra desplazamiento horizontal real.
// H = 0 es Y = 0 grados (laser perpendicular). El lado negativo es el espejo del positivo.
namespace SheetMap {

constexpr int kN = 16;
constexpr float kHmm[kN] = {0, 5, 10, 20, 30, 35, 40, 45, 50, 55, 65, 70, 75, 80, 90, 100};
constexpr float kYdeg[kN] = {0, 1.76f, 3.49f, 6.75f, 10.00f, 11.76f, 13.49f, 15.01f,
                             16.50f, 18.00f, 21.01f, 22.50f, 23.99f, 25.51f, 27.99f, 30.50f};
constexpr float kMaxTableMm = 100.0f;   // ultimo punto medido
constexpr float kPivotMm = 169.0f;      // altura equivalente del pivote sobre la hoja (H = D*tan(angulo)); ajustada a la tabla
constexpr float kRad = 0.01745329252f;  // grados -> radianes

// Angulo de Y (grados, positivo hacia H positivo) para H mm. Dentro de la tabla: interpolacion lineal entre los puntos
// medidos. Mas alla de 100 mm (extrapolacion, a peticion del usuario): sigue la curva tan(D = 169 mm) empezando en el
// ultimo punto medido, para que no haya salto.
inline float hToYdeg(float hMm) {
  const float a = fabsf(hMm);
  float deg;
  if (a >= kMaxTableMm) {
    deg = kYdeg[kN - 1] + (atanf(a / kPivotMm) - atanf(kMaxTableMm / kPivotMm)) / kRad;
  } else {
    int i = 1;
    while (i < kN - 1 && a > kHmm[i]) i++;
    const float t = (a - kHmm[i - 1]) / (kHmm[i] - kHmm[i - 1]);
    deg = kYdeg[i - 1] + t * (kYdeg[i] - kYdeg[i - 1]);
  }
  return hMm < 0 ? -deg : deg;
}

// true si H cae fuera de la tabla medida (se esta extrapolando).
inline bool hExtrapolated(float hMm) { return fabsf(hMm) > kMaxTableMm; }

// X (mm, desde el HOME de X) para un V (mm): el centro de la hoja mas V en el sentido dir (+1 / -1).
inline float vToXmm(float vMm, float xCenterMm, int dir) { return xCenterMm + dir * vMm; }

}  // namespace SheetMap
