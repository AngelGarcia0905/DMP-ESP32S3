#pragma once
#include <Arduino.h>
#include "DebouncedInput.h"
#include "Stepper28.h"
#include "dmp_config.h"
#include "dmp_pins.h"

class As5600;

// The whole positioner: two steppers, laser, limit switches (no optical home sensor on the S3).
// update() must be called about every millisecond (see startMotionTask()).
class Machine {
 public:
  void begin();
  void update();

  // Runs update() forever in a FreeRTOS task on core 1 so Wi-Fi/HTTP handling
  // in loop() never makes the motors stutter.
  void startMotionTask();

  // Commands. Return false and set *err on failure.
  // src dice quien pidio el movimiento: 'W' = panel web, 'S' = pantalla/perilla. Asi cada lado se
  // entera de lo que movio el otro (ver seq() y lastSource()).
  bool moveTo(float xCm, float angleDeg, int speedPct, const char** err, char src = 'W');
  // Bench test: turns the X motor by a number of output-shaft revolutions (negative = reverse).
  // Not limited to the working envelope, so use it only to measure travel per revolution.
  bool turnX(float revs, int speedPct, const char** err);
  // Same bench test for the Y (angle) motor.
  bool turnY(float revs, int speedPct, const char** err);
  // HOME de X por lado (como la calibracion de limits, una corrida; rapido con el largo del riel guardado): termina en
  // X = 0 (X-). HOME de Y: al cero del AS5600 por el lado mas corto (nunca mas de media vuelta) y corrigiendo con el
  // sensor. HOME completo = los dos a la vez. Los que mueven Y necesitan el AS5600 listo (ver attachEncoder).
  // HOME = la posicion que se marca en CALIBRAR > HOME (ahi X = 0 y Y = 0).
  //   startHomeX: X va a su HOME (X = 0). Necesita X referenciado (si no: "Calibra X primero", no se mueve).
  //   startHomeY: Y va al cero del AS5600 (= HOME marcado) por el lado mas corto, corrigiendo con el sensor.
  //   startHome:  los dos a la vez (si X no esta referenciado, no se mueve ninguno).
  //   startRefX:  REFERENCIAR X con los finales de carrera (por lado; rapido si el largo del riel esta guardado).
  //   markHome:   la posicion actual pasa a ser el HOME (X = 0 y/o Y = 0; para Y sobrescribe el cero del AS5600). Los
  //               topes y los puntos de la hoja se conservan en el mismo lugar fisico.
  bool startHome(const char** err);
  bool startHomeX(const char** err);
  bool startHomeY(const char** err);
  bool startRefX(const char** err);
  bool markHome(bool x, bool y, const char** err);
  bool xHomeSet() const { return _xHomeSet; }  // ya se marco el HOME de X (si no, X = 0 es el final de carrera X-)
  void attachEncoder(As5600* enc) { _enc = enc; }
  void cancelHome();                  // para un HOME o una calibracion de limits en curso

  // ---- Calibracion de los finales de carrera ----
  // Cruza entre los dos finales de carrera `samples` veces, POR LADO (no por etiqueta): en cuanto un interruptor toca se
  // detiene y sale en sentido contrario. Si empieza con un interruptor presionado, SOLO sale hacia el otro lado (si no se
  // despega, corrige el sentido del motor y lo guarda). Los pasos de cada tramo se cuentan desde que se DESPEGA el interruptor del que sale hasta que se
  // activa el otro. Promedia, fija X = 0 en X- y deja el carro en la MITAD del riel (promedio / 2).
  // Se cancela con cancelHome().
  bool startLimitCal(int samples, const char** err);
  bool testing() const { return _ts != TestState::None && !_endZero; }  // calibracion de limits en curso (no HOME)
  int legCount() const { return _legCount; }
  long legSteps(int i) const { return (i >= 0 && i < _legCount) ? _legs[i] : 0; }
  int legDir(int i) const { return (i >= 0 && i < _legCount) ? _legDir[i] : 0; }  // +1 hacia X+, -1 hacia X-
  long avgSteps() const { return _avgSteps; }  // promedio de los tramos (0 = sin calibrar)
  long middleSteps() const { return _avgSteps / 2; }
  bool xFlipped() const { return _xFlipped; }               // la ultima calibracion corrigio el sentido del motor X
  bool xInvertEffective() const { return _x.invert(); }      // sentido actual del motor X (config + correccion guardada)
  // ---- Calibracion de la hoja (puntos marcados a mano: CALIBRAR > PUNTOS DE LA HOJA) ----
  // Rejilla de 4 (2x2: esquinas), 6 (3x2: esquinas + centro arriba y abajo) o 9 puntos (3x3: esquinas, centros de los
  // lados y centro). Se marcan en orden: fila de arriba de izquierda a derecha, luego la de en medio y la de abajo.
  // Izquierda = X-, derecha = X+; arriba = angulo negativo. Cada punto guarda X en pasos (X- = 0) y Y en pasos del angulo
  // mostrado. Sirve como MAPA: moveToSheet() va a un punto de la hoja en mm (SheetMap.h). Ya NO limita el movimiento por
  // si sola. H y V se miden desde los HOME marcados (HOME X = V 0, HOME Y = H 0); la hoja marcada solo da el sentido de
  // cada eje. Lo que cae fuera de los limites de MOVER se rechaza.
  static int gridCols(int n) { return n == 4 ? 2 : 3; }
  static int gridRows(int n) { return n == 9 ? 3 : 2; }
  static const char* pointName(int n, int i);                 // "SUP IZQ", "CENTRO", ...
  static void pointSheetMm(int n, int i, float* h, float* v);  // donde cae ese punto en la hoja (mm)
  // Empieza a marcar n puntos (4, 6 o 9); la calibracion vigente no cambia hasta commitGrid(). Con keep = true se AGREGAN
  // puntos: los de la calibracion vigente se conservan (4 -> 6 o 9, 6 -> 9) y solo faltan los nuevos; nextPoint() dice cual
  // toca. Con keep = false hay que marcar todos.
  bool beginGrid(int n, const char** err, bool keep = false);
  bool pointKept(int i) const { return i >= 0 && i < kMaxPts && (_pHave & (1 << i)); }  // ese punto ya esta marcado
  int nextPoint(int from) const;  // primer punto desde 'from' que falta marcar (_pN = ya estan todos)
  bool gridStale() const;         // la hoja vigente no concuerda con el area (hay que remarcarla), o no se puede revisar
  bool markPoint(int i, const char** err);  // la posicion actual pasa a ser el punto i de la rejilla que se esta marcando
  void commitGrid();                        // termina: la rejilla marcada pasa a ser la vigente y se guarda en la flash
  int gridPoints() const { return _gN; }    // puntos de la calibracion vigente (0 = hoja sin calibrar)
  bool moveToSheet(float hMm, float vMm, const char** err, char src = 'S');
  // Origen de H/V que se MUESTRA (AJUSTES > ORIGEN HOJA): false = centro del area, true = esquina inferior izquierda.
  // Solo cambia los numeros de la pantalla; moveToSheet siempre recibe centro = 0. Se guarda en la flash ("sorg").
  bool sheetCorner() const { return _sheetCorner; }
  void setSheetCorner(bool corner);
  // Igual que moveTo (ya no hay limites de rango); se usa al marcar los puntos y en MEDIR AVANCE.
  bool jogTo(float xCm, float angleDeg, const char** err, char src = 'S');
  // Pone el contador de Y en el angulo que da el AS5600 (grados, mismo signo que la pantalla). Se usa al encender.
  void setYFromDeg(float deg);
  // ---- PASOS PERDIDOS ----
  // Y: tras un movimiento de MOVER / IR / GUARDADOS, si el AS5600 esta listo se compara el angulo pedido con el real y se
  // corrige la diferencia (hasta kYTrimTries veces); la diferencia queda anotada. X no tiene sensor: cada vez que el
  // carro toca un final de carrera, el contador se vuelve a referenciar y la diferencia queda anotada.
  long xLostLast() const { return _xLostLast; }    // ultima diferencia de X al tocar un final de carrera (pasos, con signo)
  long xLostTotal() const { return _xLostTotal; }  // suma de las diferencias (sin signo)
  int xRefCount() const { return _xRefs; }         // veces que X se re-referencio en un final de carrera
  long yLostLast() const { return _yLostLast; }    // ultima correccion de Y (pasos, con signo)
  long yLostTotal() const { return _yLostTotal; }  // suma de las correcciones (sin signo)
  int yTrimCount() const { return _yTrims; }       // movimientos de Y que necesitaron correccion
  bool yTrimBlocked() const { return _ytBad; }     // true = el sensor giro al reves: la correccion se apago
  // ---- PUNTOS guardados (modulo PUNTOS): hasta 8 posiciones a las que se puede volver ----
  // Se guarda la posicion actual de X (pasos desde el final de carrera X-) y de Y (pasos con el signo que se muestra), en la
  // flash ("dmp": smask, sx0..sx7, sy0..sy7). Necesitan X con referencia y Y con referencia, igual que los topes. Al re-marcar
  // el HOME de Y, los puntos se recorren para quedar en el mismo lugar fisico. Ir a un punto respeta los TOPES.
  static constexpr int kMaxSaved = 8;
  bool savePoint(int i, const char** err);
  bool gotoPoint(int i, const char** err, char src = 'S');
  void clearPoint(int i);
  bool pointSaved(int i) const { return i >= 0 && i < kMaxSaved && (_sMask & (1 << i)); }
  long pointXSteps(int i) const { return _sx[i]; }
  long pointYSteps(int i) const { return _sy[i]; }
  long yStepsShown() const { return lroundf(Cfg::ANGLE_SIGN * _y.position()); }  // Y en pasos, con el signo que se muestra
  // Rango que abarca el dibujo del mapa (en los mismos pasos): los topes, o la hoja calibrada, o el riel y +-35 grados.
  void mapRange(long* x0, long* x1, long* y0, long* y1) const;

  // ---- TOPES de MOVER (CALIBRAR > TOPES) ----
  // X/Y MIN Y MAX: 0 = X min, 1 = X max, 2 = Y min (arriba, angulo negativo), 3 = Y max (abajo). La posicion actual pasa
  // a ser ese tope; se guardan en la flash ("dmp": tmask, tx0, tx1, ty0, ty1). Para fijarlos X debe estar calibrado y Y
  // tener referencia (igual que los puntos de la hoja).
  // Limites de MOVER (moveTo, moveToSheet): con los 4 topes -> esos; si no, con la hoja calibrada -> el rectangulo que
  // encierra los puntos; si no -> libre. X solo se limita con X calibrado, Y solo con Y con referencia. En CALIBRAR todo
  // es libre (jogTo). Los finales de carrera siempre detienen el carro (con X calibrado).
  bool setTope(int i, const char** err);
  void clearTopes();
  bool topeSet(int i) const { return i >= 0 && i < 4 && (_tMask & (1 << i)); }
  bool topesReady() const { return _tMask == 0x0F; }
  float topeValue(int i) const;  // X en cm (i = 0, 1) o Y en grados (i = 2, 3)
  int limitSource() const { return topesReady() ? 1 : (_gN ? 2 : 0); }  // 0 libre, 1 topes, 2 puntos de la hoja
  float yMinDeg() const;
  float yMaxDeg() const;

  // Velocidad de cada eje en pasos/s (CALIBRAR > VELOCIDAD), de Cfg::SPS_UI_MIN a SPS_UI_MAX. Se guarda en la flash.
  int speedX() const { return _spsX; }
  int speedY() const { return _spsY; }
  void setSpeeds(int spsX, int spsY);
  // Escalas corregidas con lo medido (AJUSTES > ESCALA): mm por paso de X y grados por paso de Y (0 = no tocar ese eje).
  bool setScales(float xMmPerStep, float yDegPerStep);
  bool setBacklash(int xSteps, int ySteps);  // holgura al cambiar de sentido, en pasos (se guarda en flash)
  // AFINAR (pruebas guiadas): bit k de afinarDone() = la prueba k (0 escala X, 1 holgura Y, 2 holgura X) ya se hizo; afinarValue(k) =
  // lo ultimo que se escribio en ella (real mm / DIF mm). Se guardan en la flash para verlos al conectar.
  uint8_t afinarDone() const { return _afDone; }
  float afinarValue(int k) const { return (k >= 0 && k < 5) ? _afVal[k] : 0.0f; }
  void afinarSave(int k, float value);
  // Correccion por inclinacion: mm que se suman a H y V para que el punto caiga donde se pide aunque la base ya no este inclinada
  // igual que al marcar la hoja. false si no hay con que corregir (sin IMU, sin referencia o sin ganancia aprendida).
  void setTilt(float px, float py, int n) { _tcX = px; _tcY = py; _tcN = n; }
  bool setTiltRef();  // la inclinacion de ahora pasa a ser la de referencia (se hace sola al terminar de marcar la hoja)
  bool setTiltGain(int which, int axis, float mmPerDeg);  // which 0 = H, 1 = V; axis 0 / 1 = eje X / Y del chip
  bool tiltRefOk() const { return _trOk; }
  float tiltRefX() const { return _trX; }
  float tiltRefY() const { return _trY; }
  bool tiltGainOk(int w) const { return _tgOk[w & 1]; }
  float tiltGain(int w) const { return _tg[w & 1]; }
  int tiltAxis(int w) const { return _ta[w & 1]; }
  bool tiltCorr(float* dHmm, float* dVmm) const;
  float originMm() const;                    // origen de V (mm desde el HOME de X marcado)
  void resetScales();  // vuelve a las de fabrica de dmp_config.h
  // Largo del riel guardado en la flash (pasos; 0 = sin guardar). Lo guardan CALIBRAR LIMITS (promedio) y el HOME que
  // mide; con el guardado, el HOME es rapido: le basta tocar un interruptor.
  long savedRailSteps() const { return _savedRail; }
  long xSteps() const { return _x.position(); }  // contadores de pasos de cada motor (X desde el final de carrera X-)
  long xStepsRel() const { return _x.position() - _xOff; }  // X en pasos desde el HOME marcado (X = 0)
  long ySteps() const { return _y.position(); }

  void emergencyStop();
  void setLaser(bool on);
  void setCalibration(float offsetDeg);  // la altura del laser es fija (Cfg::LASER_HEIGHT_CM)

  // State
  // X se cuenta desde el HOME marcado (internamente los pasos van desde el final de carrera X-; _xOff = HOME).
  float xCm() const { return (_x.position() - _xOff) * Cfg::X_MM_PER_STEP / 10.0f; }
  float angleDeg() const { return Cfg::ANGLE_SIGN * _y.position() * Cfg::Y_DEG_PER_STEP; }
  // Objetivo vigente de cada eje (donde se esta moviendo o donde quedo).
  float targetXCm() const { return (_x.target() - _xOff) * Cfg::X_MM_PER_STEP / 10.0f; }
  float targetAngleDeg() const { return Cfg::ANGLE_SIGN * _y.target() * Cfg::Y_DEG_PER_STEP; }
  // Sube cada vez que cambia el objetivo (moveTo, HOME, STOP). lastSource(): 'W' web, 'S' pantalla, 'M' maquina.
  uint32_t seq() const { return _seq; }
  char lastSource() const { return _src; }
  bool laser() const { return _laser; }
  bool calibrated() const { return _calibrated; }  // X con referencia y Y en cero
  bool xHomed() const { return _xHomed; }
  bool testMode() const { return Cfg::TEST_NO_LIMIT_SWITCHES; }
  // Se puede mover: X con referencia, o modo prueba (sin finales de carrera).
  bool canMove() const { return _xHomed || Cfg::TEST_NO_LIMIT_SWITCHES || !Cfg::REQUIRE_HOME_BEFORE_MOVE; }
  bool yZeroed() const { return _yZeroed; }
  // Largo real del riel medido en el ultimo HOME (0 si aun no se mide) y maximo de X permitido.
  float railCm() const { return _railSteps * Cfg::X_MM_PER_STEP / 10.0f; }
  // Rango de X en MOVER: el de los TOPES (ver arriba) con X calibrado; si no hay topes, libre (+/- X_FREE_CM es un tope
  // tecnico) y solo los finales de carrera detienen el carro.
  float xMinCm() const;
  float xMaxCm() const;
  const char* homePhase() const;  // en que paso va el HOME, en texto corto para la pantalla
  // true durante un HOME o durante la calibracion de los finales de carrera (ambos bloquean el movimiento
  // y se cancelan igual).
  bool homing() const { return _ts != TestState::None || _yh != YHome::None; }
  bool moving() const { return _x.isRunning() || _y.isRunning(); }
  bool limitXMin() const { return _limMin.active(); }
  bool limitXMax() const { return _limMax.active(); }
  // No hay sensor fisico de home: se informa "en cero" cuando HOME termino y Y esta en 0.
  bool homeSensor() const { return _calibrated && !moving() && _y.position() == 0; }
  uint32_t stepsX() const { return _x.totalSteps(); }
  uint32_t stepsY() const { return _y.totalSteps(); }
  float revsX() const { return stepsX() / Cfg::STEPS_PER_REV; }
  float revsY() const { return stepsY() / Cfg::STEPS_PER_REV; }
  float laserHeightCm() const { return Cfg::LASER_HEIGHT_CM; }
  float angleOffsetDeg() const { return _angleOffset; }
  const char* lastError() const { return _error; }

 private:
  // Lo usan la calibracion de limits y el HOME. Seek: a medio riel, hasta que CUALQUIER interruptor toque. Leave: saliendo
  // del interruptor del que parte (sin contar). Cross: contando pasos hacia el otro interruptor. ToMiddle: yendo al
  // destino final (la mitad en la calibracion, X = 0 en el HOME).
  enum class TestState : uint8_t { None, Seek, Leave, Cross, ToMiddle };

  bool moveCore(float xCm, float angleDeg, int speedPct, const char** err, char src, bool checkRange);
  // AREA UTIL (Cfg::SHEET_W_MM x SHEET_H_MM, centrada en el HOME de X marcado y en el HOME de Y): limita MOVER siempre que el eje
  // tenga referencia; los topes / la hoja marcada solo pueden encerrarla mas.
  long railForArea() const { return _savedRail > 0 ? _savedRail : _railSteps; }  // largo del riel conocido (0 = ninguno)
  bool areaX() const { return _xHomed && railForArea() > 0; }
  bool areaY() const { return _yZeroed; }
  float areaXLoCm() const;
  float areaXHiCm() const;
  // ORIGEN de V (V = 0) en pasos desde X-: el HOME de X marcado por el usuario; si nunca se marco, el centro del riel.
  // CALIBRAR LIMITS deja el HOME de X en el centro del riel, asi que ese es el origen hasta que se marque otro HOME.
  long originSteps(long rail) const { return _xHomeSet ? _xOff : rail / 2; }
  // Correccion de Y con el AS5600 y re-referencia de X en los finales de carrera (PASOS PERDIDOS).
  void startYTrim();
  void updateYTrim();
  void endYTrim(bool ok);
  void onLimitHit(bool atMax);
  void setXHome(long steps);
  long homeDest(long rail) const;
  void loadGrid();
  void loadPoints();
  void savePoints();
  void saveTopes();
  void saveGrid();
  long gridMin(const long* v) const { long m = v[0]; for (int i = 1; i < _gN; i++) if (v[i] < m) m = v[i]; return m; }
  long gridMax(const long* v) const { long m = v[0]; for (int i = 1; i < _gN; i++) if (v[i] > m) m = v[i]; return m; }
  bool limitX() const { return limitSource() != 0 && _xHomed; }   // MOVER limita X
  bool limitY() const { return limitSource() != 0 && _yZeroed; }  // MOVER limita Y
  void loadSettings();
  void saveRail(long steps);
  bool beginHome(const char** err);
  bool startCal(int samples, bool endZero, const char** err);
  bool encoderReady() const;
  float encoderDeg() const;
  void beginHomeY();
  void updateHomeY();
  void updateLimitCal();
  void startLeave();
  void finishLimitCal(bool atMax);
  void finishHome(bool atMax, long rail);
  void referenceAt(bool atMax, long rail, long dest);
  void failLimitCal(const char* msg);
  void addLeg(int dir, long steps);
  void guardLimits();
  bool turnAxis(Stepper28& s, const char* name, float revs, int speedPct, const char** err);

  Stepper28 _x{Pins::X_IN1, Pins::X_IN2, Pins::X_IN3, Pins::X_IN4, Cfg::X_INVERT};
  Stepper28 _y{Pins::Y_IN1, Pins::Y_IN2, Pins::Y_IN3, Pins::Y_IN4, Cfg::Y_INVERT};
  DebouncedInput _limMin, _limMax;

  volatile uint32_t _seq = 0;
  volatile char _src = 'M';
  volatile bool _laser = false;
  volatile bool _calibrated = false;
  volatile bool _xHomed = false, _yZeroed = false;

  // HOME de Y (al cero del AS5600): Moving = girando, Settle = esperando una lectura nueva del sensor para corregir
  // HOME de Y en lazo cerrado (el AS5600 manda): Probe = movimiento corto de prueba; ProbeWait = espera la lectura;
  // Run = el recorrido, vigilando el sensor cada 25 ms; Settle = espera a que pare y corrige.
  enum class YHome : uint8_t { None, Probe, ProbeWait, Run, Settle };
  static constexpr int kYHomeTries = 6;                // correcciones despues del primer recorrido
  static constexpr float kYHomeTolDeg = 0.2f;          // el AS5600 distingue 0.088 grados: mas fino no se puede pedir
  static constexpr int kYHomeSps = 400;                // velocidad maxima del HOME de Y (el sensor se lee cada 100 ms)
  static constexpr int kYProbeSteps = 120;             // prueba de sentido: unos 2.6 grados
  static constexpr float kYWorseDeg = 1.0f;            // si el error crece mas que esto durante el recorrido, se detiene
  static constexpr float kYMaxStartDeg = 170.0f;       // cerca de 180 el lado mas corto es ambiguo: no se mueve
  static constexpr uint32_t kYSettleMs = 350;          // espera tras parar (3 lecturas del AS5600)
  volatile YHome _yh = YHome::None;
  int _yTries = 0, _yProbeTry = 0;
  int8_t _yA = +1;       // signo de los pasos que REDUCE un error positivo del sensor (+1 supuesto; la prueba lo verifica)
  int8_t _yRunSign = +1; // signo del error del sensor al empezar el recorrido
  float _yK = 0;         // grados del sensor por paso, medidos en la prueba
  float _yProbeRel = 0;  // error del sensor antes de la prueba
  int _yProbeCmd = 0;    // pasos de la prueba
  float _yBest = 0;      // menor error visto en el recorrido (grados)
  uint32_t _yT = 0;
  void startProbe(float rel, int steps);
  void evalProbe();
  void planRun(float rel);
  void finishHomeY(float rel);
  void failHomeY(const char* msg);
  As5600* _enc = nullptr;
  long _xOff = 0;           // HOME de X en pasos desde X- (flash "dmp": hxo); X = 0 ahi
  uint8_t _afDone = 0;      // flash "dmp": afd
  float _afVal[5] = {0, 0, 0, 0, 0};  // flash "dmp": af0..af4 (0 escala X, 1 holgura Y, 2 holgura X, 3 inclinacion H, 4 inclinacion V)
  // INCLINACION (IMUs): la Ui pasa la inclinacion actual (promedio de las IMUs, ejes del chip); la de referencia es la del momento
  // de marcar la hoja (flash trx, try, tro); la ganancia de cada eje de la hoja (H, V) sale de CALIBRAR > AFINAR (tgh, tgv, tah, tav).
  float _tcX = 0, _tcY = 0;
  int _tcN = 0;
  float _trX = 0, _trY = 0;
  bool _trOk = false;
  float _tg[2] = {0, 0};
  uint8_t _ta[2] = {0, 0};
  bool _tgOk[2] = {false, false};
  bool _xHomeSet = false;   // flash "dmp": hxs
  bool _sheetCorner = false;  // origen mostrado de la hoja: false centro, true esquina inferior izquierda

  // PASOS PERDIDOS. Correccion de Y: Wait = esperando que Y pare y el sensor se asiente; Fix = corrigiendo.
  enum class YTrim : uint8_t { Off, Wait, Fix };
  static constexpr float kYTrimTolDeg = 0.25f;    // dentro de esto no se toca (el AS5600 distingue 0.088 grados)
  static constexpr float kYTrimMaxDeg = 5.0f;     // mas que esto no se confia en el sensor: no se corrige
  static constexpr float kYTrimWorseDeg = 0.15f;  // si la correccion EMPEORA mas que esto, el sensor esta al reves
  static constexpr int kYTrimTries = 2;           // correcciones por movimiento
  static constexpr int kYTrimSps = 400;           // velocidad de las correcciones (pasos/s)
  volatile YTrim _yt = YTrim::Off;
  bool _ytBad = false;       // el sensor giro al reves: correccion apagada hasta reiniciar
  int _ytTries = 0;
  long _ytPos = 0;           // objetivo de Y (pasos del motor) que se esta verificando
  long _ytFixTarget = 0;     // objetivo de la correccion en curso (para notar si otro movimiento la reemplazo)
  long _ytAbsorbed = 0;      // pasos de correccion de este movimiento (no se cuentan en la posicion)
  float _ytWant = 0, _ytErr0 = 0;
  uint32_t _ytT = 0;
  volatile long _xLostLast = 0, _xLostTotal = 0, _yLostLast = 0, _yLostTotal = 0;
  volatile int _xRefs = 0, _yTrims = 0;
  volatile long _railSteps = 0;  // largo del riel en pasos (0 = sin medir)
  float _angleOffset = 0.0f;
  const char* _error = "";

  // calibracion de los finales de carrera (y HOME, que usa la misma maquina de estados)
  static constexpr int kMaxLegs = 10;
  volatile TestState _ts = TestState::None;
  volatile bool _endZero = false;  // true = es un HOME (una corrida, termina en X = 0)
  bool _quick = false;             // HOME rapido (con el largo del riel guardado)
  long _savedRail = 0;             // largo del riel guardado en la flash (pasos; 0 = sin guardar)
  int _spsX = 1000, _spsY = 1000;  // velocidad de cada eje (pasos/s)
  long _legs[kMaxLegs] = {};
  int8_t _legDir[kMaxLegs] = {};
  volatile int _legCount = 0;
  long _tStart = 0;              // posicion donde se despego el interruptor de salida (inicio del conteo del tramo)
  long _leaveStart = 0;          // posicion donde empezo la salida (para decidir el sentido)
  int8_t _dir = 0;               // signo de los pasos del tramo actual (+1 / -1)
  int8_t _from = 0;              // interruptor del que se parte: 0 = X-, 1 = X+
  int8_t _signMinToMax = 0;      // signo de los pasos que fue de X- hacia X+ (si es negativo, el sentido esta al reves)
  bool _flipTried = false, _dirVerified = false, _xFlipped = false;
  volatile long _avgSteps = 0;
  int _tSamples = 0;

  // puntos de la hoja: X en pasos del motor (X- = 0), Y en pasos del angulo mostrado (Cfg::ANGLE_SIGN * pasos del motor)
  static constexpr int kMaxPts = 9;
  long _gx[kMaxPts] = {}, _gy[kMaxPts] = {};  // calibracion vigente
  uint8_t _gN = 0;                             // 0 = sin calibrar
  long _px[kMaxPts] = {}, _py[kMaxPts] = {};  // la que se esta marcando
  uint8_t _pN = 0;
  uint16_t _pHave = 0;  // bit i = punto i ya marcado (conservado o marcado en esta vuelta)

  // topes: X en pasos del motor (0 = min, 1 = max), Y en pasos del angulo mostrado (0 = min, 1 = max)
  long _tx[2] = {}, _ty[2] = {};
  uint8_t _tMask = 0;  // bit i = tope i fijado (0 X min, 1 X max, 2 Y min, 3 Y max)

  // puntos guardados: X en pasos desde X-, Y en pasos con el signo que se muestra
  long _sx[kMaxSaved] = {}, _sy[kMaxSaved] = {};
  uint8_t _sMask = 0;  // bit i = punto i guardado
};
