#pragma once
#include <math.h>

// Conversion H / V de la hoja (mm) -> ejes. Funciones puras (sin Arduino) para poder probarlas en la PC.
//   V -> motor X (carro): el desplazamiento de X en mm es igual a V en mm, medido desde el centro del riel.
//   H -> motor Y (pivote del laser): H se escribe en mm y se convierte en el angulo que necesita Y.
// La tabla de H es la MEDIDA en la maquina: angulo que calcula el equipo contra desplazamiento horizontal real.
// ORIGEN: V = 0 es el CENTRO DEL RIEL (el que deja CALIBRAR LIMITS con el promedio de las corridas) y H = 0 es Y = 0 grados
// (el HOME de Y: laser perpendicular a la hoja). El lado negativo de H es el espejo del positivo.
// AREA UTIL: 30 cm en X (V de -150 a +150 mm) y 20 cm en Y (H de -100 a +100 mm): Cfg::SHEET_H_MM y SHEET_W_MM.
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

// X (mm, desde el CENTRO del riel) para un V (mm): V se mide desde el centro en el sentido dir (+1 / -1).
inline float vToXmm(float vMm, int dir) { return dir * vMm; }

// Objetivo de los ejes para un H / V de la hoja: X en mm desde el centro del riel, Y en grados desde el HOME de Y.
struct Target { float xMm; float yDeg; };
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

// Pasos absolutos de X (desde el final de carrera X-) para X mm medidos desde el HOME marcado (xOff pasos).
inline long xStepsFromHome(float xMm, float mmPerStep, long xOff) { return lroundf(xMm / mmPerStep) + xOff; }

// Centro del riel en pasos desde X- (V = 0).
inline long railCenterSteps(long railSteps) { return railSteps / 2; }

// Milimetros del centro del riel medidos desde el HOME de X marcado (que X muestra como X = 0).
inline float centerFromHomeMm(long railSteps, long xOff, float mmPerStep) {
  return (railCenterSteps(railSteps) - xOff) * mmPerStep;
}

// Pasos absolutos de X para un V (mm desde el centro del riel), con el sentido dir de la hoja. Es lo mismo que hace
// Machine::moveToSheet (mm desde el HOME de X = centro + V, y de ahi a pasos).
inline long xStepsForV(float vMm, int dir, long railSteps, long xOff, float mmPerStep) {
  return xStepsFromHome(centerFromHomeMm(railSteps, xOff, mmPerStep) + vToXmm(vMm, dir), mmPerStep, xOff);
}

// AREA UTIL: H y V (mm desde el centro) no pueden pasar de la mitad del ancho / alto del area. nullptr = dentro.
inline const char* checkArea(float hMm, float vMm, float halfWMm, float halfHMm) {
  if (fabsf(vMm) > halfHMm + 0.001f) return "V fuera del area";
  if (fabsf(hMm) > halfWMm + 0.001f) return "H fuera del area";
  return nullptr;
}
// Angulo maximo de Y (grados desde el HOME de Y) que cubre el area util a cada lado.
inline float areaYDeg(float halfWMm) { return hToYdeg(halfWMm); }

// ---- AGREGAR PUNTOS: de 4 a 6 o 9, o de 6 a 9, sin volver a marcar los que ya estan ----
inline int gridColsN(int n) { return n == 4 ? 2 : 3; }
inline int gridRowsN(int n) { return n == 9 ? 3 : 2; }
// Indice que ocupa, en la rejilla de n puntos, el punto i0 de la rejilla de n0 puntos (n > n0): las orillas se quedan en
// la orilla (primera / ultima columna y fila) y el resto son puntos nuevos. 4 -> 6: 0,1,2,3 pasan a 0,2,3,5.
inline int gridKeepIndex(int n0, int n, int i0) {
  const int nc0 = gridColsN(n0), nr0 = gridRowsN(n0), nc = gridColsN(n), nr = gridRowsN(n);
  const int r0 = i0 / nc0, c0 = i0 % nc0;
  const int c = (nc0 == nc) ? c0 : (c0 == 0 ? 0 : nc - 1);
  const int r = (nr0 == nr) ? r0 : (r0 == 0 ? 0 : nr - 1);
  return r * nc + c;
}

// ---- CORRECCION con las esquinas marcadas (4, 6 o 9 puntos) ----
// El modelo (tabla de H, V en mm desde el centro) da donde DEBERIA caer cada punto de la hoja; las esquinas marcadas dicen
// donde cayo de verdad. La diferencia en cada punto (residuo) se reparte por la hoja interpolando entre puntos (bilineal
// por celda): con 4 esquinas se corrige lo de las orillas y lo de en medio sale del reparto; con 6 o 9 puntos cada celda
// es mas chica y se corrige mas fino. Los puntos nominales son las esquinas / centros del AREA UTIL (pointSheetMm).
constexpr float kMaxResidMm = 25.0f;  // un residuo mayor es una hoja marcada para OTRA area (o un punto mal marcado)
constexpr float kMaxResidDeg = 5.0f;
struct Resid { float xMm; float yDeg; };

// Residuo de cada punto: lo marcado menos lo que dice el modelo. gx en pasos desde X-, gy en grados / degPerStep (angulo
// mostrado). centerSteps = centro del riel (V 0). Los puntos van por filas de arriba a abajo (fila 0 = V +halfH).
inline void gridResiduals(const long* gx, const long* gy, int nc, int nr, float halfWMm, float halfHMm, long centerSteps,
                          float mmPerStep, float degPerStep, int dirX, int sgnY, float* rx, float* ry) {
  for (int i = 0; i < nc * nr; i++) {
    const int r = i / nc, c = i % nc;
    const float h = -halfWMm + c * 2.0f * halfWMm / (nc - 1);
    const float v = halfHMm - r * 2.0f * halfHMm / (nr - 1);
    rx[i] = (gx[i] - centerSteps) * mmPerStep - vToXmm(v, dirX);
    ry[i] = gy[i] * degPerStep - sgnY * hToYdeg(h);
  }
}

// true si algun residuo es demasiado grande para ser un error del modelo: la hoja se marco para otra area y hay que remarcarla.
inline bool residTooBig(const float* rx, const float* ry, int n, float maxMm = kMaxResidMm, float maxDeg = kMaxResidDeg) {
  for (int i = 0; i < n; i++)
    if (fabsf(rx[i]) > maxMm || fabsf(ry[i]) > maxDeg) return true;
  return false;
}

// Residuo en (h, v): bilineal dentro de la celda de la rejilla que lo contiene (fuera del area se queda en la orilla).
inline Resid residAt(const float* rx, const float* ry, int nc, int nr, float hMm, float vMm, float halfWMm, float halfHMm) {
  float u = (hMm + halfWMm) / (2.0f * halfWMm) * (nc - 1);
  float w = (halfHMm - vMm) / (2.0f * halfHMm) * (nr - 1);
  u = u < 0 ? 0 : (u > nc - 1 ? (float)(nc - 1) : u);
  w = w < 0 ? 0 : (w > nr - 1 ? (float)(nr - 1) : w);
  int c = (int)u, r = (int)w;
  if (c > nc - 2) c = nc - 2;
  if (r > nr - 2) r = nr - 2;
  const float fu = u - c, fw = w - r;
  const int i00 = r * nc + c, i01 = i00 + 1, i10 = i00 + nc, i11 = i10 + 1;
  const float w00 = (1 - fu) * (1 - fw), w01 = fu * (1 - fw), w10 = (1 - fu) * fw, w11 = fu * fw;
  return {w00 * rx[i00] + w01 * rx[i01] + w10 * rx[i10] + w11 * rx[i11],
          w00 * ry[i00] + w01 * ry[i01] + w10 * ry[i10] + w11 * ry[i11]};
}

// Objetivo ya corregido: modelo + residuo interpolado.
inline Target correctedTarget(float hMm, float vMm, int dirX, int sgnY, const Resid& r) {
  Target t = hvFromHome(hMm, vMm, dirX, sgnY);
  t.xMm += r.xMm;
  t.yDeg += r.yDeg;
  return t;
}

// Revisa el objetivo contra los limites de MOVER (X en mm desde el centro del riel, Y en grados desde el HOME de Y). nullptr = dentro; si no, el mensaje.
inline const char* checkRange(const Target& t, float xMinMm, float xMaxMm, float yMinDeg, float yMaxDeg, float halfStepMm,
                              float halfStepDeg, const char* vMsg = "V fuera de limites",
                              const char* hMsg = "H fuera de limites") {
  if (t.xMm < xMinMm - halfStepMm || t.xMm > xMaxMm + halfStepMm) return vMsg;
  if (t.yDeg < yMinDeg - halfStepDeg || t.yDeg > yMaxDeg + halfStepDeg) return hMsg;
  return nullptr;
}

}  // namespace SheetMap
