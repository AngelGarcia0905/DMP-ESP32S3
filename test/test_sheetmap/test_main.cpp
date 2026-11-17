#include <unity.h>
#include "../../lib/DmpCore/src/SheetMap.h"

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

// V = 20 mm ordena a X un desplazamiento de exactamente 20 mm desde el HOME, en el sentido de la hoja marcada.
void test_v20_mueve_x_20mm() {
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20.0f, vToXmm(20.0f, 1));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -20.0f, vToXmm(20.0f, -1));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 60.0f, vToXmm(30.0f, 1) - vToXmm(-30.0f, 1));
}

// ---- HOME como origen de H y V ----
// Hoja de 9 puntos marcada con el HOME FUERA del centro: gx en pasos desde X-, gy en pasos del angulo mostrado (respecto
// al HOME de Y). Filas de arriba a abajo; V positivo (arriba) = MAS pasos de X; H positivo (derecha) = angulo positivo.
static const long kGx9[9] = {9000, 9000, 9000, 6000, 6000, 6000, 3000, 3000, 3000};
static const long kGy9[9] = {-300, 100, 500, -300, 100, 500, -300, 100, 500};  // el centro de la hoja NO esta en Y = 0
static const long kXoff = 4000;                  // HOME X marcado fuera del centro (el centro de la hoja es 6000)
static const float kMmPerStep = 0.0013728f;

void test_h0_v0_va_al_home() {
  int dirX, sgnY;
  gridDirs(kGx9, kGy9, 3, 3, &dirX, &sgnY);
  const Target t = hvFromHome(0.0f, 0.0f, dirX, sgnY);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.xMm);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yDeg);
  // X absoluto = exactamente los pasos del HOME marcado (no el centro de la hoja 6000 ni el del riel)
  TEST_ASSERT_EQUAL_INT32(kXoff, xStepsFromHome(t.xMm, kMmPerStep, kXoff));
  TEST_ASSERT_TRUE(kXoff != 6000);
}

void test_v_se_mide_desde_home_x_con_la_orientacion_calibrada() {
  int dirX, sgnY;
  gridDirs(kGx9, kGy9, 3, 3, &dirX, &sgnY);
  TEST_ASSERT_EQUAL_INT(1, dirX);  // arriba = mas pasos en esta hoja
  const Target up = hvFromHome(0.0f, 20.0f, dirX, sgnY), dn = hvFromHome(0.0f, -20.0f, dirX, sgnY);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20.0f, up.xMm);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -20.0f, dn.xMm);
  TEST_ASSERT_EQUAL_INT32(kXoff + lroundf(20.0f / kMmPerStep), xStepsFromHome(up.xMm, kMmPerStep, kXoff));
  TEST_ASSERT_EQUAL_INT32(kXoff - lroundf(20.0f / kMmPerStep), xStepsFromHome(dn.xMm, kMmPerStep, kXoff));
  // Con la hoja marcada al reves (arriba = menos pasos) el sentido se invierte, nada mas.
  static const long gxInv[9] = {3000, 3000, 3000, 6000, 6000, 6000, 9000, 9000, 9000};
  int d2, s2;
  gridDirs(gxInv, kGy9, 3, 3, &d2, &s2);
  TEST_ASSERT_EQUAL_INT(-1, d2);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -20.0f, hvFromHome(0.0f, 20.0f, d2, s2).xMm);
}

void test_h_se_mide_desde_home_y_con_la_orientacion_calibrada() {
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
  TEST_ASSERT_EQUAL_FLOAT(0.0f, hvFromHome(0.0f, 0.0f, d2, s2).yDeg);
}

// Cambiar el HOME (xOff) no mueve H/V al centro de la hoja: V = 0 sigue siendo el HOME, sea cual sea.
void test_home_fuera_del_centro_no_vuelve_al_centro() {
  int dirX, sgnY;
  gridDirs(kGx9, kGy9, 3, 3, &dirX, &sgnY);
  const long homes[3] = {0, 4000, 9000};
  for (int i = 0; i < 3; i++)
    TEST_ASSERT_EQUAL_INT32(homes[i], xStepsFromHome(hvFromHome(0, 0, dirX, sgnY).xMm, kMmPerStep, homes[i]));
  // Un HOME de Y marcado en otro lado solo recorre los angulos de la hoja: los signos no cambian.
  long gy2[9];
  for (int i = 0; i < 9; i++) gy2[i] = kGy9[i] - 250;
  int d2, s2;
  gridDirs(kGx9, gy2, 3, 3, &d2, &s2);
  TEST_ASSERT_EQUAL_INT(dirX, d2);
  TEST_ASSERT_EQUAL_INT(sgnY, s2);
}

// Los limites siguen rechazando lo que queda fuera (medidos desde el HOME): V -> "V fuera", H -> "H fuera".
void test_limites_rechazan_fuera_de_rango() {
  // X de -50 a +150 mm desde el HOME; Y de -10 a +30 grados desde el HOME
  const float xmin = -50, xmax = 150, ymin = -10, ymax = 30, hx = 0.0007f, hy = 0.04f;
  TEST_ASSERT_NULL(checkRange({0, 0}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_NULL(checkRange({150, 30}, xmin, xmax, ymin, ymax, hx, hy));  // justo en el tope
  TEST_ASSERT_EQUAL_STRING("V fuera de limites", checkRange({150.5f, 0}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("V fuera de limites", checkRange({-50.5f, 0}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("H fuera de limites", checkRange({0, 30.5f}, xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("H fuera de limites", checkRange({0, -10.5f}, xmin, xmax, ymin, ymax, hx, hy));
  // H = 20 mm (6.75 grados) cabe; H = -50 mm (-16.5 grados) no cabe porque Y min es -10
  TEST_ASSERT_NULL(checkRange(hvFromHome(20, 0, 1, 1), xmin, xmax, ymin, ymax, hx, hy));
  TEST_ASSERT_EQUAL_STRING("H fuera de limites", checkRange(hvFromHome(-50, 0, 1, 1), xmin, xmax, ymin, ymax, hx, hy));
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
  RUN_TEST(test_h0_v0_va_al_home);
  RUN_TEST(test_v_se_mide_desde_home_x_con_la_orientacion_calibrada);
  RUN_TEST(test_h_se_mide_desde_home_y_con_la_orientacion_calibrada);
  RUN_TEST(test_home_fuera_del_centro_no_vuelve_al_centro);
  RUN_TEST(test_limites_rechazan_fuera_de_rango);
  return UNITY_END();
}
