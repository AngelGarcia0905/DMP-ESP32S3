#include <unity.h>
#include "../../lib/DmpCore/src/SheetMap.h"
#include "../../lib/DmpCore/src/StepGuard.h"

using namespace SheetMap;

void setUp() {}
void tearDown() {}

// H = 20 mm se escribe en mm; el firmware lo convierte solo en ~6.75 grados de Y.
void test_h20_es_6_75_grados() { TEST_ASSERT_FLOAT_WITHIN(0.005f, 6.75f, hToYdeg(20.0f)); }

// Todos los puntos medidos dan exactamente su angulo.
void test_puntos_de_la_tabla() {
  for (int i = 0; i < kN; i++) TEST_ASSERT_FLOAT_WITHIN(0.005f, kYdeg[i], hToYdeg(kHmm[i]));
}

// Valores intermedios: lineal entre los dos puntos vecinos.
void test_intermedios() {
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.625f, hToYdeg(7.5f));    // entre 5 mm (1.76) y 10 mm (3.49): 2.625
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 8.375f, hToYdeg(25.0f));   // entre 20 (6.75) y 30 (10.00): 8.375
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 19.505f, hToYdeg(60.0f));  // entre 55 (18.00) y 65 (21.01): 19.505
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 29.245f, hToYdeg(95.0f));  // entre 90 (27.99) y 100 (30.50): 29.245
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.88f, hToYdeg(2.5f));     // entre 0 (0) y 5 mm (1.76): 0.88
}

// Crece siempre (sin retrocesos) en toda la hoja.
void test_monotona() {
  float prev = hToYdeg(0.0f);
  for (float h = 0.25f; h <= 139.5f; h += 0.25f) {
    const float d = hToYdeg(h);
    TEST_ASSERT_TRUE(d > prev);
    prev = d;
  }
}

// El lado negativo es el espejo del positivo; H = 0 es Y = 0.
void test_espejo_y_cero() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, hToYdeg(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -6.75f, hToYdeg(-20.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -hToYdeg(73.0f), hToYdeg(-73.0f));
}

// Mas alla de 100 mm (extrapolacion pedida): continua en 100 mm y sigue la curva tan, creciente.
void test_extrapolacion_pasada_de_100() {
  TEST_ASSERT_FALSE(hExtrapolated(100.0f));
  TEST_ASSERT_TRUE(hExtrapolated(100.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.50f, hToYdeg(100.001f));
  const float expect139 = 30.50f + (atanf(139.5f / kPivotMm) - atanf(100.0f / kPivotMm)) / kRad;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, expect139, hToYdeg(139.5f));
  TEST_ASSERT_TRUE(hToYdeg(139.5f) > 30.5f && hToYdeg(139.5f) < 42.0f);
}

// V = 20 mm ordena a X un desplazamiento de exactamente 20 mm desde el centro del riel, en el sentido de la hoja marcada.
void test_v20_mueve_x_20mm() {
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20.0f, vToXmm(20.0f, 1));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -20.0f, vToXmm(20.0f, -1));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 60.0f, vToXmm(30.0f, 1) - vToXmm(-30.0f, 1));
}

// ---- ORIGEN en el centro del riel (V 0) y en Y = 0 (H 0); area util 30 x 20 cm ----
// Hoja de 9 puntos marcada: gx en pasos desde X-, gy en pasos del angulo mostrado. Filas de arriba a abajo;
// V positivo (arriba) = MAS pasos de X; H positivo (derecha) = angulo positivo. El centro de la hoja NO esta en Y = 0.
static const long kGx9[9] = {9000, 9000, 9000, 6000, 6000, 6000, 3000, 3000, 3000};
static const long kGy9[9] = {-300, 100, 500, -300, 100, 500, -300, 100, 500};
static const long kRail = 9000;               // largo del riel (pasos); su centro es 4500
static const float kMmPerStep = 0.0013728f;
static const float kHalfW = 100.0f, kHalfH = 150.0f;  // area util: H +/-100 mm (Y), V +/-150 mm (X)

// V = 0 es el centro del riel, este donde este el HOME de X marcado (que solo cambia lo que X muestra como 0).
void test_v0_es_el_centro_del_riel_aunque_el_home_este_fuera() {
  int dirX, sgnY;
  gridDirs(kGx9, kGy9, 3, 3, &dirX, &sgnY);
  const long homes[4] = {0, 1000, 4500, 9000};
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_INT32(4500, xStepsForV(0.0f, dirX, kRail, homes[i], kMmPerStep));
  TEST_ASSERT_EQUAL_INT32(4500, railCenterSteps(kRail));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, hvFromHome(0.0f, 0.0f, dirX, sgnY).yDeg);  // H = 0 es Y = 0
}

// V positivo y negativo se miden desde el centro, con la orientacion de la hoja marcada.
void test_v_se_mide_desde_el_centro_con_la_orientacion_calibrada() {
  int dirX, sgnY;
  gridDirs(kGx9, kGy9, 3, 3, &dirX, &sgnY);
  TEST_ASSERT_EQUAL_INT(1, dirX);  // arriba = mas pasos en esta hoja
  const long d = lroundf(20.0f / kMmPerStep);
  for (long home = 0; home <= 9000; home += 3000) {  // el HOME marcado no cambia nada
    TEST_ASSERT_INT32_WITHIN(1, 4500 + d, xStepsForV(20.0f, dirX, kRail, home, kMmPerStep));
    TEST_ASSERT_INT32_WITHIN(1, 4500 - d, xStepsForV(-20.0f, dirX, kRail, home, kMmPerStep));
  }
  // Hoja marcada al reves (arriba = menos pasos): el sentido se invierte, nada mas.
  static const long gxInv[9] = {3000, 3000, 3000, 6000, 6000, 6000, 9000, 9000, 9000};
  int d2, s2;
  gridDirs(gxInv, kGy9, 3, 3, &d2, &s2);
  TEST_ASSERT_EQUAL_INT(-1, d2);
  TEST_ASSERT_INT32_WITHIN(1, 4500 - d, xStepsForV(20.0f, d2, kRail, 1000, kMmPerStep));
}

// H positivo y negativo se miden desde Y = 0 (HOME de Y) con la orientacion calibrada.
void test_h_se_mide_desde_y_cero_con_la_orientacion_calibrada() {
  int dirX, sgnY;
  gridDirs(kGx9, kGy9, 3, 3, &dirX, &sgnY);
  TEST_ASSERT_EQUAL_INT(1, sgnY);  // derecha = angulo positivo en esta hoja
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 6.75f, hvFromHome(20.0f, 0.0f, dirX, sgnY).yDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, -6.75f, hvFromHome(-20.0f, 0.0f, dirX, sgnY).yDeg);
  static const long gyInv[9] = {500, 100, -300, 500, 100, -300, 500, 100, -300};  // derecha = angulo negativo
  int d2, s2;
  gridDirs(kGx9, gyInv, 3, 3, &d2, &s2);
  TEST_ASSERT_EQUAL_INT(-1, s2);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, -6.75f, hvFromHome(20.0f, 0.0f, d2, s2).yDeg);
}

// AREA UTIL: H +/-100 mm y V +/-150 mm. Pasarse se rechaza (no se recorta).
void test_area_util_30x20() {
  TEST_ASSERT_NULL(checkArea(0, 0, kHalfW, kHalfH));
  TEST_ASSERT_NULL(checkArea(100, 150, kHalfW, kHalfH));
  TEST_ASSERT_NULL(checkArea(-100, -150, kHalfW, kHalfH));
  TEST_ASSERT_EQUAL_STRING("V fuera del area", checkArea(0, 150.5f, kHalfW, kHalfH));
  TEST_ASSERT_EQUAL_STRING("V fuera del area", checkArea(0, -151, kHalfW, kHalfH));
  TEST_ASSERT_EQUAL_STRING("H fuera del area", checkArea(100.5f, 0, kHalfW, kHalfH));
  TEST_ASSERT_EQUAL_STRING("H fuera del area", checkArea(-101, 0, kHalfW, kHalfH));
  // Y cubre +/-100 mm de H = +/-30.5 grados, justo el rango de la tabla medida
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 30.5f, areaYDeg(kHalfW));
  TEST_ASSERT_FALSE(hExtrapolated(kHalfW));
}

// Los limites (topes u hoja marcada) siguen rechazando lo que queda fuera. X en mm desde el centro, Y en grados desde Y = 0.
void test_limites_rechazan_fuera_de_rango() {
  const float xmin = -50, xmax = 120, ymin = -10, ymax = 30, hx = 0.0007f, hy = 0.04f;
  TEST_ASSERT_NULL(checkRange({0, 0}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_NULL(checkRange({120, 30}, xmin, xmax, ymin, ymax, hx, hy));  // justo en el tope
  TEST_ASSERT_EQUAL_STRING("V fuera de limites", checkRange({120.5f, 0}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("V fuera de limites", checkRange({-50.5f, 0}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("H fuera de limites", checkRange({0, 30.5f}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("H fuera de limites", checkRange({0, -10.5f}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_NULL(checkRange(hvFromHome(20, 0, 1, 1), xmin, xmax, ymin, ymax, hx, hy));  // H 20 mm = 6.75 grados
  TEST_ASSERT_EQUAL_STRING("H fuera de limites", checkRange(hvFromHome(-50, 0, 1, 1), xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("V fuera de la hoja", checkRange({200, 0}, xmin, xmax, ymin, ymax, hx, hy, "V fuera de la hoja",
                                                             "H fuera de la hoja"));
}

// ---- PASOS PERDIDOS ----
static const float kDeg = 0.02197f;  // grados por paso de Y

// Y: dentro de la tolerancia no se toca nada; fuera se corrige con el signo de ANGLE_SIGN (-1); muy lejos no se confia.
void test_trim_de_y() {
  using namespace StepGuard;
  const float sign = -1.0f;
  Trim t = yTrim(0.10f, 0.25f, 5.0f, kDeg, sign);
  TEST_ASSERT_EQUAL_INT((int)TrimAct::Ok, (int)t.act);
  TEST_ASSERT_EQUAL_INT32(0, t.steps);
  t = yTrim(-0.25f, 0.25f, 5.0f, kDeg, sign);  // justo en la tolerancia
  TEST_ASSERT_EQUAL_INT((int)TrimAct::Ok, (int)t.act);
  t = yTrim(1.0f, 0.25f, 5.0f, kDeg, sign);  // el sensor quedo 1 grado corto: ANGLE_SIGN = -1 -> pasos negativos
  TEST_ASSERT_EQUAL_INT((int)TrimAct::Fix, (int)t.act);
  TEST_ASSERT_EQUAL_INT32(lroundf(-1.0f / kDeg), t.steps);
  t = yTrim(-1.0f, 0.25f, 5.0f, kDeg, sign);
  TEST_ASSERT_EQUAL_INT32(lroundf(1.0f / kDeg), t.steps);
  t = yTrim(5.5f, 0.25f, 5.0f, kDeg, sign);  // demasiado lejos: no se corrige
  TEST_ASSERT_EQUAL_INT((int)TrimAct::TooBig, (int)t.act);
  TEST_ASSERT_EQUAL_INT32(0, t.steps);
}

// Si una correccion empeora el error, el sensor gira al reves: se debe apagar la correccion.
void test_trim_detecta_sensor_al_reves() {
  using namespace StepGuard;
  TEST_ASSERT_FALSE(trimWorse(1.0f, 0.4f, 0.15f));   // mejoro
  TEST_ASSERT_FALSE(trimWorse(1.0f, 1.1f, 0.15f));   // dentro del ruido
  TEST_ASSERT_TRUE(trimWorse(1.0f, 1.5f, 0.15f));    // empeoro
  TEST_ASSERT_TRUE(trimWorse(-1.0f, 1.6f, 0.15f));   // cruzo al otro lado y quedo peor
}

// X: al tocar un final de carrera la diferencia con lo esperado son los pasos perdidos (X- = 0, X+ = riel).
void test_pasos_perdidos_de_x_en_el_limite() {
  using namespace StepGuard;
  TEST_ASSERT_EQUAL_INT32(0, xLostAtLimit(0, 0));
  TEST_ASSERT_EQUAL_INT32(-35, xLostAtLimit(-35, 0));          // toco X- con el contador en -35: le faltaban 35 pasos
  TEST_ASSERT_EQUAL_INT32(120, xLostAtLimit(kRail + 120, kRail));  // toco X+ 120 pasos antes de lo contado
}

// ---- CORRECCION con las esquinas (4 / 6 / 9 puntos) ----
// Marca "perfecta": cada punto cae exactamente donde el modelo dice (la hoja sin error).
static void idealGrid(int nc, int nr, long* gx, long* gy, int dirX, int sgnY, long center) {
  for (int i = 0; i < nc * nr; i++) {
    const int r = i / nc, c = i % nc;
    const float h = -kHalfW + c * 2.0f * kHalfW / (nc - 1);
    const float v = kHalfH - r * 2.0f * kHalfH / (nr - 1);
    gx[i] = center + lroundf(dirX * v / kMmPerStep);
    gy[i] = lroundf(sgnY * hToYdeg(h) / kDeg);
  }
}

// Sin error no se corrige nada.
void test_esquinas_perfectas_no_corrigen() {
  long gx[9], gy[9];
  float rx[9], ry[9];
  idealGrid(3, 3, gx, gy, 1, 1, 4500);
  gridResiduals(gx, gy, 3, 3, kHalfW, kHalfH, 4500, kMmPerStep, kDeg, 1, 1, rx, ry);
  for (int i = 0; i < 9; i++) { TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rx[i]); TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, ry[i]); }
  TEST_ASSERT_FALSE(residTooBig(rx, ry, 9));
  const Resid r = residAt(rx, ry, 3, 3, 37.0f, -61.0f, kHalfW, kHalfH);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r.xMm);
  TEST_ASSERT_FLOAT_WITHIN(0.04f, 0.0f, r.yDeg);
}

// Con 4 esquinas, cada esquina marcada queda EXACTA al pedir su H / V, y en medio se reparte la diferencia.
void test_4_esquinas_corrigen_y_reparten() {
  long gx[4], gy[4];
  float rx[4], ry[4];
  idealGrid(2, 2, gx, gy, 1, 1, 4500);
  gx[0] += lroundf(3.0f / kMmPerStep);  // la esquina SUP IZQ se marco 3 mm mas arriba (X) que el modelo
  gy[0] += lroundf(0.5f / kDeg);        // y 0.5 grados mas a un lado (Y)
  gridResiduals(gx, gy, 2, 2, kHalfW, kHalfH, 4500, kMmPerStep, kDeg, 1, 1, rx, ry);
  Resid r = residAt(rx, ry, 2, 2, -kHalfW, kHalfH, kHalfW, kHalfH);  // justo en esa esquina
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, r.xMm);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, r.yDeg);
  // modelo + residuo en la esquina = lo marcado
  Target c = correctedTarget(-kHalfW, kHalfH, 1, 1, r);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, (gx[0] - 4500) * kMmPerStep, c.xMm);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, gy[0] * kDeg, c.yDeg);
  // las otras esquinas no cambian y el centro recibe la cuarta parte de la diferencia (bilineal)
  r = residAt(rx, ry, 2, 2, kHalfW, -kHalfH, kHalfW, kHalfH);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r.xMm);
  r = residAt(rx, ry, 2, 2, 0.0f, 0.0f, kHalfW, kHalfH);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.75f, r.xMm);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.125f, r.yDeg);
}

// Con 9 puntos el punto del CENTRO corrige solo su zona: una esquina lejana no se entera.
void test_9_puntos_corrigen_mas_fino() {
  long gx[9], gy[9];
  float rx[9], ry[9];
  idealGrid(3, 3, gx, gy, 1, 1, 4500);
  gx[4] += lroundf(2.0f / kMmPerStep);  // el CENTRO se marco 2 mm mas arriba
  gridResiduals(gx, gy, 3, 3, kHalfW, kHalfH, 4500, kMmPerStep, kDeg, 1, 1, rx, ry);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, residAt(rx, ry, 3, 3, 0, 0, kHalfW, kHalfH).xMm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, residAt(rx, ry, 3, 3, kHalfW, kHalfH, kHalfW, kHalfH).xMm);  // esquina: nada
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, residAt(rx, ry, 3, 3, 0, kHalfH / 2, kHalfW, kHalfH).xMm);   // a medio camino
}

// AGREGAR PUNTOS: los puntos vigentes se conservan en su lugar de la rejilla nueva (las orillas siguen siendo orillas).
void test_agregar_puntos_conserva_los_vigentes() {
  // 4 -> 6: SUP IZQ, SUP DER, INF IZQ, INF DER pasan a 0, 2, 3, 5 (los centros 1 y 4 son nuevos)
  TEST_ASSERT_EQUAL_INT(0, gridKeepIndex(4, 6, 0));
  TEST_ASSERT_EQUAL_INT(2, gridKeepIndex(4, 6, 1));
  TEST_ASSERT_EQUAL_INT(3, gridKeepIndex(4, 6, 2));
  TEST_ASSERT_EQUAL_INT(5, gridKeepIndex(4, 6, 3));
  // 4 -> 9: las cuatro esquinas pasan a 0, 2, 6, 8
  TEST_ASSERT_EQUAL_INT(0, gridKeepIndex(4, 9, 0));
  TEST_ASSERT_EQUAL_INT(2, gridKeepIndex(4, 9, 1));
  TEST_ASSERT_EQUAL_INT(6, gridKeepIndex(4, 9, 2));
  TEST_ASSERT_EQUAL_INT(8, gridKeepIndex(4, 9, 3));
  // 6 -> 9: la fila de arriba se queda (0,1,2) y la de abajo pasa de 3,4,5 a 6,7,8 (la de en medio es nueva)
  for (int i = 0; i < 3; i++) TEST_ASSERT_EQUAL_INT(i, gridKeepIndex(6, 9, i));
  for (int i = 3; i < 6; i++) TEST_ASSERT_EQUAL_INT(i + 3, gridKeepIndex(6, 9, i));
  // no se pierde ni se repite ningun punto
  for (int n0 = 4; n0 <= 6; n0 += 2)
    for (int n = (n0 == 4 ? 6 : 9); n <= 9; n += 3) {
      bool used[9] = {};
      for (int i = 0; i < n0; i++) {
        const int j = gridKeepIndex(n0, n, i);
        TEST_ASSERT_TRUE(j >= 0 && j < n);
        TEST_ASSERT_FALSE(used[j]);
        used[j] = true;
      }
    }
}

// Una hoja marcada para OTRA area (por ejemplo la carta de antes) se detecta y se pide remarcar.
void test_hoja_de_otra_area_se_detecta() {
  long gx[4], gy[4];
  float rx[4], ry[4];
  // esquinas de la carta de antes: H +/-139.5, V +/-108
  const float hs[2] = {-139.5f, 139.5f}, vs[2] = {108.0f, -108.0f};
  for (int i = 0; i < 4; i++) {
    gx[i] = 4500 + lroundf(vs[i / 2] / kMmPerStep);
    gy[i] = lroundf(hToYdeg(hs[i % 2]) / kDeg);
  }
  gridResiduals(gx, gy, 2, 2, kHalfW, kHalfH, 4500, kMmPerStep, kDeg, 1, 1, rx, ry);
  TEST_ASSERT_TRUE(residTooBig(rx, ry, 4));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_h20_es_6_75_grados);
  RUN_TEST(test_puntos_de_la_tabla);
  RUN_TEST(test_intermedios);
  RUN_TEST(test_monotona);
  RUN_TEST(test_espejo_y_cero);
  RUN_TEST(test_extrapolacion_pasada_de_100);
  RUN_TEST(test_v20_mueve_x_20mm);
  RUN_TEST(test_v0_es_el_centro_del_riel_aunque_el_home_este_fuera);
  RUN_TEST(test_v_se_mide_desde_el_centro_con_la_orientacion_calibrada);
  RUN_TEST(test_h_se_mide_desde_y_cero_con_la_orientacion_calibrada);
  RUN_TEST(test_area_util_30x20);
  RUN_TEST(test_limites_rechazan_fuera_de_rango);
  RUN_TEST(test_esquinas_perfectas_no_corrigen);
  RUN_TEST(test_4_esquinas_corrigen_y_reparten);
  RUN_TEST(test_9_puntos_corrigen_mas_fino);
  RUN_TEST(test_agregar_puntos_conserva_los_vigentes);
  RUN_TEST(test_hoja_de_otra_area_se_detecta);
  RUN_TEST(test_trim_de_y);
  RUN_TEST(test_trim_detecta_sensor_al_reves);
  RUN_TEST(test_pasos_perdidos_de_x_en_el_limite);
  return UNITY_END();
}
