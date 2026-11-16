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

// V = 20 mm ordena a X un desplazamiento de exactamente 20 mm (en cualquier sentido y desde cualquier centro).
void test_v20_mueve_x_20mm() {
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20.0f, vToXmm(20.0f, 0.0f, 1) - vToXmm(0.0f, 0.0f, 1));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20.0f, vToXmm(20.0f, 123.4f, 1) - 123.4f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -20.0f, vToXmm(20.0f, 123.4f, -1) - 123.4f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 60.0f, vToXmm(30.0f, 0.0f, 1) - vToXmm(-30.0f, 0.0f, 1));
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
  return UNITY_END();
}
