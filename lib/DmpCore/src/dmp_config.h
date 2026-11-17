#pragma once
#include <Arduino.h>

// Constantes ajustables del ESP32-S3 (sin home optico: el cero de Y es donde queda al hacer HOME).
// Tunable constants. Values marked CALIBRATE are taken from the project README
// and must be confirmed on the real mechanism.
namespace Cfg {

// ---- Kinematics ------------------------------------------------------------
// Reduccion mecanica EFECTIVA del eje X (motor : carro). 0.0613 mm/paso es el valor de tabla del motor SIN reduccion.
// MEDIDO el 2026-11-12: la pantalla marcaba 1764 mm (con 8:1) y el carro avanzo 280 mm reales -> factor 1764 / 280 = 6.3,
// asi que la reduccion efectiva es 8 x 6.3 = 50.4 (0.0613 / 50.4 = 0.0012163 mm por paso). Es una sola medicion de ~28 cm:
// conviene repetirla (CALIBRAR LIMITS: "Cree" contra el riel medido) y ajustar este numero si cambia.
constexpr float X_REDUCTION    = 50.4f;   // valor de fabrica; el efectivo vigente es 0.0613 / X_MM_PER_STEP (xReductionEff())
constexpr float X_MM_PER_STEP_DEFAULT = 0.0613f / X_REDUCTION;  // mm de carro por paso de fabrica
constexpr float STEPS_PER_REV  = 4096.0f;  // 28BYJ-48 output shaft, half-step mode
// Eje Y (motor : laser) con reduccion 4:1: grados del laser por paso = 360 / 4096 / 4 = 0.02197.
// (Antes 0.703 / 4 = 0.176: el 0.703 de la tabla es 360 / 512, o sea por cada 8 pasos, y la pantalla marcaba ~8 veces de
// mas.) COMPROBADO el 2026-11-12 con 6 puntos sobre la hoja (1, 2, 3, 4, 5 y 10 cm desde el centro, laser perpendicular en
// el centro; 10 cm = 1390 pasos y la pantalla marcaba 244.1 grados): con 15.7 cm de la PUNTA del laser a la hoja dan
// 0.0231 a 0.0238 grados/paso; el valor teorico 0.02197 cuadra si el EJE de giro esta ~1.2 cm arriba de la punta
// (~16.9 cm a la hoja). Falta medir esa distancia para confirmarlo; si no cuadra, ajustar Y_REDUCTION.
constexpr float Y_REDUCTION    = 4.0f;
constexpr float Y_DEG_PER_STEP_DEFAULT = 360.0f / STEPS_PER_REV / Y_REDUCTION;  // grados por paso de fabrica

// ESCALAS VIGENTES: parten de los valores de fabrica de arriba y se corrigen desde la pantalla (AJUSTES > ESCALA, escribiendo
// lo que se midio de verdad); lo corregido se guarda en la flash ("dmp": xmm, ydg) y se carga al encender. Se definen en
// Machine.cpp. Ya no son constexpr.
extern float X_MM_PER_STEP;   // mm de carro por paso
extern float Y_DEG_PER_STEP;  // grados del laser por paso
// Reduccion efectiva que corresponde a la escala vigente (para mostrarla): 0.0613 mm/paso y 360/4096 grados/paso
// son los valores del motor SIN reduccion.
inline float xReductionEff() { return 0.0613f / X_MM_PER_STEP; }
inline float yReductionEff() { return (360.0f / STEPS_PER_REV) / Y_DEG_PER_STEP; }
// Holgura de los engranes de Y al cambiar de sentido: el motor da estos pasos EXTRA (no se cuentan en la posicion) antes
// de que el laser empiece a moverse hacia el otro lado. Medido el 2026-11-12: al regresar hicieron falta 4 a 10 pasos de
// mas; 10 es el valor actual (0 = sin compensar). Subirlo si al regresar todavia queda corto, bajarlo si se pasa.
constexpr int Y_BACKLASH_STEPS = 10;

// Flip these if an axis moves the wrong way (positive X = right, positive Y = clockwise).
constexpr bool X_INVERT = false;  // con los finales de carrera intercambiados (GPIO 47 = X-, GPIO 1 = X+), pasos negativos
                                  // = hacia X-. Si X va al reves (el carro no se despega del interruptor), poner true.
constexpr bool Y_INVERT = false;
// Sentido de los angulos que se MUESTRAN y se ESCRIBEN (pantalla, perilla, panel web, G-code y AS5600):
// -1 = invertido (un angulo positivo gira el laser hacia el lado contrario que con +1). NO toca el giro del motor
// (eso es Y_INVERT), solo el signo con que se cuentan los grados. El home (0) no cambia.
constexpr float ANGLE_SIGN = -1.0f;

// Distancia de la PUNTA del laser a la hoja con el laser perpendicular (medida aproximada del 2026-11-12). El eje de giro
// queda mas arriba (ver Y_DEG_PER_STEP). Hoy es solo informativa: el firmware no la usa para mover.
constexpr float LASER_HEIGHT_CM = 15.7f;

// ---- MODO PRUEBA ----------------------------------------------------------------------------
// true  = para probar SIN finales de carrera conectados: se puede mover sin hacer HOME y el HOME se da por
//         hecho al instante (X = 0 y Y = 0 donde esten, sin buscar los interruptores). La pantalla muestra
//         "TEST" y el panel web "MODO PRUEBA".
// false = funcionamiento normal: no se mueve hasta que HOME toque X- y X+ (y mida el riel).
// CAMBIAR A false cuando los finales de carrera esten conectados.
constexpr bool TEST_NO_LIMIT_SWITCHES = false;  // HOME real: los finales de carrera ya van conectados

// Exigir HOME antes de mover. false = se puede mover sin hacer HOME (el HOME todavia se esta definiendo);
// los finales de carrera siguen deteniendo el carro si lo tocan. Poner true cuando el HOME ya este definido.
constexpr bool REQUIRE_HOME_BEFORE_MOVE = false;

// Los finales de carrera DETIENEN el carro al tocarlos? false = el movimiento es libre: los interruptores ya no
// cortan nada (siguen mostrandose en SENSORES / el panel y se usan en la calibracion de limits y en el HOME).
// Con false el carro puede chocar con el extremo del riel: solo lo limita el rango de X. Poner true cuando los
// interruptores esten bien conectados.
constexpr bool LIMIT_SWITCHES_STOP_MOTION = true;
// true = los interruptores solo cortan el movimiento DESPUES de calibrar X (con X sin referencia el movimiento es libre:
// sirve para despegar el carro de un interruptor, o acomodarlo a mano en el centro, antes de calibrar).
constexpr bool LIMITS_FREE_UNTIL_CALIBRATED = true;

// ---- Working envelope ------------------------------------------------------
// Los limites de MOVER los define el usuario en CALIBRAR > TOPES (X/Y min y max, o el rectangulo de los puntos de la
// hoja); sin topes, MOVER es libre. X_MIN_CM / X_MAX_CM / X_END_MARGIN_CM / X_SOFT_LIMITS ya no los usa el firmware
// completo (solo la prueba suelta de pantalla step14_tft_ui).
constexpr float X_MIN_CM  =   0.0f;
constexpr float X_MAX_CM  =  40.0f;
// Angulo (Y): sin topes, sin limite. Este rango es solo un tope tecnico (cientos de vueltas) para que los numeros no
// desborden; no es un limite real.
constexpr float Y_MIN_DEG = -100000.0f;
constexpr float Y_MAX_DEG =  100000.0f;

// Tras HOME se mide el largo del riel (hasta tocar el final de carrera X+). El maximo permitido es
// ese largo menos este margen, para no quedarse sobre el interruptor (nunca pasa de X_MAX_CM).
constexpr float X_END_MARGIN_CM = 0.2f;
// X SIN limite de software: la UNICA restriccion de X son los finales de carrera de los dos lados. (Con true se
// vuelve al rango de X_MIN_CM..X_MAX_CM / el largo medido del riel.)
constexpr bool X_SOFT_LIMITS = false;
constexpr float X_FREE_CM = 10000.0f;  // tope tecnico (100 m) para que los numeros no desborden; no es un limite real
// SIN limite de distancia al buscar un final de carrera (HOME y calibracion de limits): el carro avanza hasta que el
// interruptor TOQUE y solo se detiene si se cancela (STOP, clic de la perilla o 'C' por el monitor serie).
constexpr float LIMIT_SEARCH_MAX_CM = 120.0f;
// Calibracion de los finales de carrera: cuantos tramos (X- <-> X+) se cuentan para sacar el promedio. En la pantalla
// (CALIBRAR > CALIBRAR LIMITS) se eligen de 1 a LIMIT_CAL_MAX_RUNS; este es el valor inicial y el de la tecla 'L'.
constexpr int LIMIT_CAL_SAMPLES = 3;
constexpr int LIMIT_CAL_MAX_RUNS = 5;

// Hoja (carta acostada): ancho a lo largo del riel (X) y alto en el sentido del angulo. El (0, 0) es el centro de la
// hoja: horizontal positivo a la derecha (X+), vertical positivo hacia arriba (angulo negativo).
constexpr float SHEET_W_MM = 279.0f;
constexpr float SHEET_H_MM = 216.0f;
// Al salir de un interruptor presionado, si no se despega tras estos PASOS el sentido de los pasos estaba al reves:
// se invierte y se sigue. NO es un limite de recorrido: solo sirve para decidir el sentido. (Antes se calculaba a partir de
// mm y con la escala vieja salian 13 050 pasos; se deja fijo en pasos para que corregir los mm/paso no lo cambie.)
constexpr long LIMIT_DIR_DETECT_STEPS = 13050;

// ---- Signal polarity -------------------------------------------------------
constexpr uint8_t LIMIT_ACTIVE = LOW;   // switch to GND with pull-up -> LOW when pressed
constexpr uint8_t LASER_ACTIVE = HIGH;  // HIGH turns the laser driver on

// ---- Encoder angular AS5600 (compensacion angular automatica) ----------------
// Signo del angulo del sensor respecto al sentido positivo del motor Y.
constexpr bool  ENC_INVERT = false;   // CALIBRATE: poner true si al subir el angulo del motor baja el del sensor
// Vueltas del iman por vuelta del laser. 1.0 = iman directo en el eje del laser.
constexpr float ENC_RATIO  = 1.0f;    // CALIBRATE: si el iman va en un engrane con relacion, ponerla aqui

// ---- Speeds (half-steps per second) ---------------------------------------
// Los pasos los da un temporizador de hardware con resolucion de microsegundos, asi que SPS_MAX puede ser CUALQUIER
// valor de 10 a 2000 pasos/s (no solo 1000, 500, 333...). Stepper28 arranca a RAMP_START_SPS y acelera de forma suave
// (+RAMP_ACCEL_SPS en cada paso) hasta esta velocidad, y frena igual al llegar. Si el motor pierde pasos o zumba sin
// avanzar, bajar SPS_MAX poco a poco (900, 800...). Con la escala medida de X (0.0012 mm/paso), 1000 pasos/s son unos 1.2 mm/s.
// El 28BYJ-48 suele perder fuerza arriba de ~1000 pasos/s: subir de ahi solo probando con cuidado.
// SPS_MAX es el valor inicial de cada eje; en la pantalla (CALIBRAR > VELOCIDAD) se elige la de X y la de Y por
// separado, de SPS_UI_MIN a SPS_UI_MAX en saltos de SPS_UI_STEP, y se guardan en la flash ("dmp": spsx, spsy).
// El HOME y la calibracion de limits usan la velocidad de X.
constexpr float SPS_MAX     = 1000.0f;
// Rampa de arranque y frenado de los motores (Stepper28). Un motor cargado pierde pasos si arranca o frena brusco, y como
// X no tiene encoder el firmware no puede notarlo. La rampa sube RAMP_ACCEL_SPS pasos/s en cada paso desde RAMP_START_SPS
// hasta la velocidad elegida (y baja igual al llegar): con 80 y 1.5, de 80 a 1000 pasos/s son ~610 pasos y ~1.7 s (en X,
// ~0.75 mm). Un movimiento corto no llega a la velocidad maxima: sube y baja sin pasarse de lo que el motor aguanta.
// Antes: arranque 125 y +10 por paso (88 pasos, 0.17 s). Si aun pierde pasos: bajar RAMP_START_SPS y RAMP_ACCEL_SPS.
constexpr float RAMP_START_SPS = 80.0f;
constexpr float RAMP_ACCEL_SPS = 1.5f;
// Las bobinas siguen energizadas (el motor "sostiene" la posicion) este tiempo despues del ultimo paso y luego se sueltan
// para que no se caliente. Antes 300 ms: soltarlas pronto deja que la carga mueva el eje entre un movimiento y el siguiente.
constexpr uint32_t MOTOR_HOLD_MS = 2000;
constexpr int   SPS_UI_MIN  = 100;
constexpr int   SPS_UI_MAX  = 1000;
constexpr int   SPS_UI_STEP = 50;

// ---- Network ---------------------------------------------------------------
// Real credentials go in include/secrets.h (git-ignored). See secrets.example.h.
constexpr const char* AP_SSID     = "DMPS3";
constexpr const char* AP_PASSWORD = "dmp12345";   // at least 8 characters
constexpr const char* HOSTNAME    = "dmp-laser-s3";  // http://dmp-laser-s3.local
constexpr uint32_t    STA_TIMEOUT_MS = 12000;

}  // namespace Cfg
