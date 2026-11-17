#pragma once
#include <math.h>

// Conversion H / V de la hoja (mm) -> ejes. Funciones puras (sin Arduino) para poder probarlas en la PC.
//   V -> motor X (carro): el desplazamiento de X en mm es igual a V en mm, medido desde el HOME de X.
//   H -> motor Y (pivote del laser): H se escribe en mm y se convierte en el angulo que necesita Y.
// La tabla de H es la MEDIDA en la maquina: angulo que calcula el equipo contra desplazamiento horizontal real.
// H = 0 es el HOME de Y marcado (Y = 0 grados) y V = 0 es el HOME de X marcado: el HOME es el origen, no el centro de
// la hoja ni del riel. El lado negativo de H es el espejo del positivo.
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

// X (mm, desde el HOME de X) para un V (mm): V se mide desde el HOME de X marcado (V = 0) en el sentido dir (+1 / -1).
inline float vToXmm(float vMm, int dir) { return dir * vMm; }

// Objetivo de los ejes para un H / V de la hoja. Ambos se miden desde los HOME marcados: HOME X = V 0 y HOME Y = H 0.
struct Target { float xMm; float yDeg; };  // X en mm desde el HOME de X, Y en grados desde el HOME de Y
inline Target hvFromHome(float hMm, float vMm, int dirX, int sgnY) { return {vToXmm(vMm, dirX), sgnY * hToYdeg(hMm)}; }

// Sentido de cada eje segun la hoja marcada (filas de arriba hacia abajo, columnas de izquierda a derecha):
// dirX = +1 si V positivo (arriba) lleva a mas pasos de X; sgnY = +1 si H positivo (derecha) lleva a angulo positivo.
// Solo importa hacia donde crece cada eje: donde esta el centro o el HOME no cuenta.
inline void gridDirs(const long* gx, const long* gy, int nc, int nr, int* dirX, int* sgnY) {
  float xTop = 0, xBot = 0, yL = 0, yR = 0;
  for (int c = 0; c < nc; c++) { xTop += gx[c]; xBot += gx[(nr - 1) * nc + c]; }
  for (int r = 0; r < nr; r++) { yL += gy[r * nc]; yR += gy[r * nc + nc - 1]; }
  *dirX = (xTop / nc >= xBot / nc) ? 1 : -1;
  *sgnY = (yR / nr >= yL / nr) ? 1 : -1;
}

// Pasos absolutos de X (desde el final de carrera X-) para X mm medidos desde el HOME de X marcado (xOff pasos).
inline long xStepsFromHome(float xMm, float mmPerStep, long xOff) { return lroundf(xMm / mmPerStep) + xOff; }

// Revisa el objetivo contra los limites de MOVER (en las mismas unidades de HOME). nullptr = dentro; si no, el mensaje.
inline const char* checkRange(const Target& t, float xMinMm, float xMaxMm, float yMinDeg, float yMaxDeg, float halfStepMm,
                              float halfStepDeg, const char* vMsg = "V fuera de limites",
                              const char* hMsg = "H fuera de limites") {
  if (t.xMm < xMinMm - halfStepMm || t.xMm > xMaxMm + halfStepMm) return vMsg;
  if (t.yDeg < yMinDeg - halfStepDeg || t.yDeg > yMaxDeg + halfStepDeg) return hMsg;
  return nullptr;
}

}  // namespace SheetMap
