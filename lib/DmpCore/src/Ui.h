#pragma once
#include <Arduino.h>

class Machine;
class Imu;
class As5600;
class Adafruit_ST7735;
class GFXcanvas16;

// Interfaz de la pantalla TFT (ST7735 128x160, vertical) + perilla KY-040, estilo impresora 3D:
// fondo negro, texto blanco, menu en cuadricula. Trabaja EN CONJUNTO con el panel web: los dos
// mueven la misma Machine, asi que lo que se hace en uno se ve en el otro.
//   girar = moverse / cambiar valor     pulsar = entrar / cambiar de eje     mantener = volver
// Ejes: X = motor a pasos (1 mm por clic), Y = angulo del laser, +/- grados (0.25 grados por clic).
class Ui {
 public:
  Ui(Machine& m, Imu& white, Imu& black, As5600& enc) : _m(m), _imuW(white), _imuB(black), _enc(enc) {}

  void begin();
  void update();  // llamar en cada pasada de loop()

  // true mientras se esta editando un eje en MOVER o al marcar un punto de la hoja: el LED RGB se queda en azul.
  bool editing() const {
    return (_screen == MOVER || _screen == MARCAR || _screen == ESCALA || _screen == MINMAX || _screen == HMARK) &&
           _editing;
  }
  // true durante todo el marcado de puntos de la hoja (de INICIAR al ultimo OK): el LED RGB se queda en azul.
  bool marking() const { return _screen == MARCAR; }

 private:
  // LIMITS: corridas, riel real y escala de X. PUNTOS (titulo HOJA): elegir 4/6/9 puntos. MARCAR: marcarlos uno por uno.
  // VELOCIDAD: velocidad de cada eje. ESCALA (MEDIR Y ESCALA): contador libre y escala de Y. MINMAX: topes de MOVER.
  // HMARK: marcar el HOME. GUARDADOS: puntos guardados. ARRANQUE: ofrecer REFERENCIAR X al encender.
  enum Screen : uint8_t { MENU, MOVER, HOME, SENSORES, RED, CALIBRAR, LIMITS, PUNTOS, MARCAR, VELOCIDAD,
                          MINMAX, HMARK, AJUSTES, ESCALA, GUARDADOS, ARRANQUE };

  // dibujo
  void render();
  void colorCheck();  // pantalla de prueba de color al arrancar
  void drawMenu();
  void drawMover();
  void drawHome();
  void drawSensores();
  void drawRed();
  void drawCalibrar();
  void drawLimits();
  void drawPuntos();
  void drawMarcar();
  void drawVelocidad();
  void drawMinMax();
  void drawHMark();
  void drawAjustes();
  void drawEscala();
  void drawGuardados();
  void drawArranque();
  float limRealShown() const;  // riel real que se muestra en LIMITS (escrito o propuesto)
  void box(int x, int y, int w, int h, const char* text, bool sel);  // recuadro pequeno con texto centrado
  void homeBoxes(int y, int h, int sel);  // recuadros HOME X / HOME Y / AMBOS (sel = indice elegido, -1 ninguno)
  void runHome(int kind);                 // 0 = X, 1 = Y, 2 = los dos
  void markMeasureStart();  // MEDIR Y ESCALA: la posicion actual pasa a ser el inicio del conteo
  void drawLimitProgress(int y);  // fase de la calibracion de limits y ultima muestra (4 lineas desde y)
  uint16_t accent() const;
  void header(const char* title);
  // Fila de valor (alto 21): etiqueta chica a la izquierda y valor grande. Editando: relleno; elegida: marco doble.
  void valueRow(int y, const char* label, const char* value, bool sel, bool edit, int h = 21);
  void button(int y, int h, const char* text, bool sel);  // boton de ancho completo con texto centrado
  void drawBackTab(bool selected);  // pestana "REGRESAR" al pie de cada pantalla
  void statusRow(int y, const char* label, const char* status, uint16_t color);  // etiqueta a la izquierda, estado a la derecha
  void textCentered(int cx, int y, const char* s, int size, uint16_t color);

  // entrada
  void onTurn(int detents, bool fast);
  void onClick();
  void pollButton();

  // maquina
  void adoptTarget();
  void syncFromMachine();
  void moveLive();
  bool stepsMode() const;  // MOVER en modo PASOS
  void clampToLimits();  // recorta el objetivo de MOVER a los TOPES (CALIBRAR > TOPES MIN / MAX); sin topes, libre
  void editAxis(int axis, int d, bool fast);  // axis 0 = X, 1 = Y: cambia el objetivo y mueve en vivo
  void say(const char* msg);

  Machine& _m;
  Imu& _imuW;
  Imu& _imuB;
  As5600& _enc;
  Adafruit_ST7735* _tft = nullptr;
  GFXcanvas16* _cv = nullptr;

  Screen _screen = MENU;
  int _menuSel = 0, _calSel = 0, _homeSel = 0;
  int _redSel = 1;       // en RED: 0 = boton del punto de acceso, 1 = pestana REGRESAR
  int _axisSel = 0;      // en MOVER: 0 X, 1 Y, 2 H, 3 V, 4 HOME X, 5 HOME Y, 6 AMBOS, 7 UNIDAD, 8 LASER, 9 REGRESAR
  bool _stepsMode = false;  // MOVER: false = X en mm y Y en grados; true = X y Y en pasos del motor
  int _stepSize = 1;        // modo PASOS: pasos por clic lento (1, 10 o 100)
  int _mmMult = 1;          // modo MM/GRD: 1 = 1 mm y 0.25 grados por clic; 10 = diez veces eso
  long _tgtXs = 0;       // objetivo de X en pasos desde el HOME marcado (modo PASOS)
  int _ajSel = 0;        // en AJUSTES: 0 = VELOCIDAD, 1 = MEDIR Y ESCALA, 2 = REGRESAR
  int _gSel = 0, _gSlot = 0;  // en PUNTOS: fila (0 PUNTO n, 1 GUARDAR, 2 IR, 3 BORRAR, 4 REGRESAR) y punto elegido (0..7)
  int _escSel = 0;       // en ESCALA: 0 X, 1 Y, 2 MARCAR INICIO, 3 REAL Y, 4 APLICAR Y, 5 FABRICA, 6 REGRESAR
  float _realY = 0;      // grados de Y medidos de verdad, escritos con la perilla
  bool _measMarked = false;      // ya se marco un inicio en MEDIR Y ESCALA
  int _hmSel = 0;        // en HMARK: 0 = X, 1 = Y, 2 = MARCAR AMBOS, 3 = MARCAR X, 4 = MARCAR Y, 5 = REFERENCIAR X, 6 = REGRESAR
  long _measX0 = 0, _measY0 = 0;  // pasos de cada motor al marcar el inicio
  float _measEnc0 = 0;   // angulo del AS5600 al marcar el inicio
  bool _measEncOk = false;
  int _limSel = 0;       // en LIMITS: 0 = corridas, 1 = INICIAR, 2 = riel REAL, 3 = APLICAR ESCALA, 4 = REGRESAR
  float _limReal = 0;    // riel real (mm, de tope a tope) escrito con la perilla
  bool _limRealSet = false;  // false = se propone lo que cree el equipo
  int _bootSel = 0;      // en ARRANQUE: 0 = REFERENCIAR AHORA, 1 = DESPUES
  int _limRuns = 3;      // corridas elegidas para la calibracion de limits (1..Cfg::LIMIT_CAL_MAX_RUNS)
  int _mmSel = 0;        // en MINMAX: 0 = X, 1 = Y, 2..5 = X min, X max, Y min, Y max, 6 = BORRAR, 7 = REGRESAR
  int _ptsSel = 0;       // en PUNTOS: 0 = cantidad, 1 = INICIAR, 2 = REGRESAR
  int _ptsN = 4;         // puntos a marcar: 4, 6 o 9
  int _ptIdx = 0;        // en MARCAR: punto que toca (0.._ptsN-1)
  int _markSel = 0;      // en MARCAR: 0 = X, 1 = Y, 2 = OK, 3 = LASER, 4 = REGRESAR (cancela)
  int _spdSel = 0;       // en VELOCIDAD: 0 = X, 1 = Y, 2 = REGRESAR
  int _spdX = 1000, _spdY = 1000;  // velocidades que se estan editando (pasos/s)
  int _sensTop = 0;      // en SENSORES: primer renglon visible de la lista
  int _sensLines = 0;    // renglones que tuvo la lista en el ultimo dibujo
  bool _editing = false; // true = el giro cambia el valor elegido; false = el giro elige fila
  int _tgtXmm = 0;       // objetivo de X en mm
  int _tgtYsteps = 0;    // objetivo de Y en pasos del motor (cada paso = Cfg::Y_DEG_PER_STEP grados)
  float _hMm = 0, _vMm = 0;   // objetivo en la hoja (mm, centro = 0, arriba positivo)
  bool _sheetValid = false;   // false = el laser se movio por X/Y y H/V ya no dicen donde esta
  bool _dirty = true, _wasHoming = false;
  uint32_t _seen = 0, _lastDraw = 0, _frameHash = 0, _msgUntil = 0;
  char _msg[26] = "";
};
