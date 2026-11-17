# DMP - Posicionador laser (ESP32-S3)

**Este es el unico proyecto activo de DMP** (los proyectos del ESP32 DOIT clasico, `ESP32_Completo` y `ESP32_Pasos`, se mandaron a la Papelera de reciclaje el 2026-11-15). Aqui solo vive la placa ESP32-S3. Todos los cables tienen ya su color asignado (ver **Pines y cables**).

> **Sin interfaz web.** Todo se hace desde la pantalla TFT y la perilla: mover, HOME, calibrar (IMUs, HOME marcado, limits, topes), ajustes (velocidad y medir avance), sensores y red. El ESP32-S3 sigue creando la red `DMPS3` (clave `PIA2026.`), pero solo para que el LED y la pantalla RED sepan si hay equipos conectados: no hay servidor ni API HTTP. El codigo del panel web (el viejo y el que era espejo de la pantalla) quedo archivado en `_web_removida/`, fuera de la compilacion.

## Plataforma

| | |
|---|---|
| Placa | ESP32-S3 DevKitC-1 N16R8: 16 MB flash, 8 MB PSRAM, MAC `3c:dc:75:6f:77:20` |
| Puerto | USB-C marcado **UART/COM** (chip CH343) (en esta PC salio como `COM21`; PlatformIO detecta el puerto solo) |
| Entorno | PlatformIO, `espressif32@6.13.0`, framework Arduino, entorno `esp32s3` |
| Subir | `pio run -t upload` (cerrar antes el monitor serie, ocupa el puerto) |
| Monitor | `pio device monitor` (115200) |

Firmware completo: el core de Arduino-ESP32 (`Wire`, `WiFi`, `Preferences`) mas `Adafruit GFX` y `Adafruit ST7735` para la pantalla (PlatformIO las baja solo). Codigo propio en `lib/DmpCore`: `Stepper28`, `Machine`, `Imu`/`Mpu`, `As5600`, `Net`, `StatusLed` y `Ui` (pantalla + perilla).

## Pantalla y perilla

El firmware completo incluye la pantalla TFT (blanco sobre negro, vertical, menu en cuadricula) manejada con la perilla KY-040. El equipo guarda **un unico objetivo** (X en mm, Y en pasos del motor) que la pantalla muestra y edita. Los limites y el tamano del paso los manda el equipo.

| Perilla | Accion |
|---|---|
| Girar | Moverse entre opciones; ya **dentro de la edicion de un eje**, cambia su valor (X en pasos de 1 mm, Y en **pasos del motor**; girando rapido x5) |
| Pulsar (clic corto, no hay clic largo) | Entrar a una opcion / entrar a editar un eje / salir de la edicion |

**Como se mueve un eje (pantalla MOVER):** girar elige la fila (X, Y o la pestana REGRESAR); un clic en X o Y **entra a editar** (la fila se rellena de color y dice EDITANDO); girar mueve el eje en vivo; otro clic **sale** de la edicion y se puede girar hasta la otra fila. Para volver al menu se elige la pestana **< REGRESAR** (abajo, en todas las pantallas) y se pulsa. **LED RGB:** se queda en azul mientras se edita un eje (aunque no haya ningun equipo conectado); al salir vuelve a su color normal (verde si hay un equipo conectado, rojo si no).

Cada modulo tiene su color, **MOVER verde, HOME turquesa, CALIBRAR naranja, AJUSTES amarillo, SENSORES rosa, RED azul** (la conexion). El laser ya no esta en el menu: se enciende y apaga en MOVER y al marcar los puntos de la hoja. Los datos siguen en blanco sobre negro; el color va en el marco, el encabezado y la seleccion. Si el azul y el rojo salen cambiados, la pantalla usa el orden de color contrario: probar otro `TAB_TYPE`/`INITR_*` en `Ui.cpp`.

| Opcion | Que hace |
|---|---|
| MOVER | Mueve en vivo (velocidad fija, con rampa), **sin necesidad de HOME**. Filas **X mm**, **Y grd**, **H mm** y **V mm** (mm de la hoja, solo con la hoja calibrada; H 0 y V 0 son los HOME marcados). Muestra la posicion real; Y sale del AS5600 si esta listo y con cero, si no, de contar pasos. **MOVER se limita con los TOPES** que se definen en CALIBRAR > TOPES MIN / MAX (se queda en la orilla): los 4 de X/Y MIN Y MAX si estan; si no, el rectangulo que encierra los puntos de la hoja; **sin topes, MOVER es libre**. X solo se limita con X calibrado y Y con referencia. Los finales de carrera siempre detienen el carro (con X calibrado). **X y Y se mueven en vivo; H y V solo se escriben** (girar no mueve nada) y el boton **IR A H, V** lleva el laser a ese punto de la hoja usando los puntos de CALIBRAR > HOJA como mapa. Lo escrito se queda aunque se mueva por X/Y; la primera vez parte de H 0, V 0 (el HOME). Si un movimiento se rechaza o un final de carrera lo corta, abajo aparece el motivo. Recuadros **HOME X / HOME Y / AMBOS** (un clic; ver la fila MOVER > HOME). Recuadro de unidad (cada clic cambia: **MM/GRD -> MM/GRD x10 -> PASOS x1 -> x10 -> x100**; "x10" mueve 10 mm y 2.5 grados por clic): en **mm/grados** X avanza 1 mm por clic y Y 0.25 grados por clic; en **PASOS** X e Y muestran y se mueven en pasos exactos del motor (**1, 10 o 100 por clic lento, y 50 veces eso girando rapido**; un solo medio paso del 28BYJ-48 es 0.088 grados del eje y la holgura de sus engranes lo puede tragar, por eso hay x10 y x100; X contada desde el HOME marcado) y abajo sale el contador real de pasos de cada motor, para hacer mediciones. Recuadro **LASER: ON / OFF**: un clic lo enciende o lo apaga |
| MOVER > HOME X / HOME Y / AMBOS | Tres recuadros dentro de MOVER (el modulo HOME del menu se quito por estar repetido; su pantalla solo aparece al REFERENCIAR desde ARRANQUE): **HOME X**, **HOME Y** y **AMBOS**. Cada eje va al **HOME marcado** en CALIBRAR > HOME (ahi X = 0 y Y = 0). **HOME X** necesita X referenciado (si no, avisa "Calibra X primero" y no se mueve). **HOME Y** va al cero del AS5600 (= el HOME marcado) por el **lado mas corto** (nunca mas de media vuelta) en **lazo cerrado: el sensor es el juez y no se confia en el signo ni en los grados por paso de la configuracion**. (1) **Prueba:** un movimiento corto (120 pasos, ~2.6 grados) hacia el lado que se supone reduce el error; si el error crece, el sentido motor/sensor estaba al reves y se invierte solo para esa corrida (`[homeY] DIAGNOSTICO` indica poner `ENC_INVERT = true`), y de la prueba sale la escala real en grados del sensor por paso; si casi no se mueve, una prueba 4 veces mayor y, si tampoco, "Y no se mueve". (2) **Recorrido** a velocidad baja (maximo 400 pasos/s) vigilando el sensor cada 25 ms: se detiene al cruzar el cero, y si el error crece mas de 1 grado se detiene y avisa "HOME Y: se aleja". (3) **Ajuste:** hasta 6 correcciones hasta quedar a menos de 0.2 grados (el AS5600 distingue 0.088). Si Y esta a mas de 170 grados del cero no se mueve ("Y cerca de 180 grd": el lado mas corto es ambiguo). Cada paso sale en el monitor serie con `[homeY]`. Sin AS5600 listo no se mueve. **AMBOS** = los dos a la vez (si falta algo, no se mueve ninguno). Abajo: si X esta referenciado, si el HOME de X esta marcado y el AS5600. Un clic cancela un HOME o una referencia en curso |
| MENU | Seis modulos (cuadricula de 2 x 3): **MOVER, GUARDADOS, CALIBRAR, AJUSTES, SENSORES, RED**. **MOVER** = mover, ir a H/V y ir al HOME; **GUARDADOS** = puntos guardados y mapa; **CALIBRAR** = lo que se calibra y se marca (IMUs, HOME, LIMITS, HOJA, TOPES MIN / MAX); **AJUSTES** = VELOCIDAD y MEDIR Y ESCALA; **SENSORES** = toda la informacion (incluida la inclinacion de las IMUs); **RED** = punto de acceso. En el encabezado de cualquier pantalla, **"X?" en rojo** significa que X no tiene referencia (se pierde al apagar): se ofrece al encender (ARRANQUE) o con CALIBRAR > HOME (MARCAR) > REFERENCIAR X |
| GUARDADOS | **Hasta 8 puntos guardados** para volver a ellos (apuntar y dejar ahi). Arriba, un **dibujo** del area: X horizontal, Y vertical (arriba = angulo negativo), con los puntos guardados (cuadritos), el elegido (marco blanco) y donde esta el laser ahora (cruz, o circulo rojo si esta encendido); el area es la de los topes, o la de la hoja calibrada, o el riel y +-35 grados. Filas: **PUNTO n** (clic y girar para elegir de 1 a 8), **GUARDAR AQUI**, **IR AL PUNTO** (respeta los topes) y **BORRAR PUNTO**. Se guardan en la flash (`dmp`: `smask`, `sx0..sx7`, `sy0..sy7`) y necesitan X y Y con referencia; al re-marcar el HOME de Y se recorren solos para quedar en el mismo lugar fisico |
| CALIBRAR | **Calibrar las 2 IMUs** a la vez (las que esten conectadas; avisa cual falta), **HOME (MARCAR)**, **CALIBRAR LIMITS** (riel, dispersion y escala de X), **HOJA (4/6/9 PUNTOS)** y **TOPES MIN / MAX**. Dentro de CALIBRAR todo se mueve libre. Abajo muestra el AS5600 y el largo del riel guardado, y la pestana REGRESAR |
| CALIBRAR > HOME (MARCAR) | Como MOVER pero **sin limites**: filas X y Y. **MARCAR AMBOS AQUI**, **MARCAR X** y **MARCAR Y** dejan ahi el HOME: **ese lugar pasa a ser X = 0 y/o Y = 0**. X: se guarda en pasos desde el final de carrera X- (flash `dmp`: `hxo`, `hxs`); necesita X referenciado. Y: **sobrescribe el cero del AS5600** con el angulo actual; los topes y los puntos de la hoja se recorren para quedar en el mismo lugar fisico. **REFERENCIAR X**: el HOME de antes por los finales de carrera (por lado; rapido con el largo del riel guardado); da la referencia de X despues de cada encendido |
| AJUSTES > MEDIR Y ESCALA | **Contador libre y correccion de la escala de Y en una sola pantalla** (antes eran MEDIR AVANCE y ESCALA). Al entrar se marca el inicio (o con **MARCAR INICIO**). Filas **X** y **Y**: clic para editar, girar mueve en vivo (sin limites). Debajo, en vivo desde el inicio: pasos de X y los mm que cree, pasos de Y y los grados que cree, y **Y AS5600** (lo que midio el sensor; menos de media vuelta). Fila **Y real**: al entrar propone lo del AS5600 y se corrige con la perilla; **APLICAR Y** calcula grados por paso = real / pasos (minimo 50 pasos) y lo guarda en la flash (`dmp`: `ydg`), y el contador de Y se iguala con el AS5600. **FABRICA** borra las escalas guardadas de X y Y. **La escala de X se corrige en CALIBRAR > LIMITS** con el riel completo (es la medicion mas larga y la mas exacta). Rechaza valores que cambien la escala mas de 5 veces. Todo lo guardado en pasos (topes, puntos, riel) no cambia |
| AJUSTES > VELOCIDAD | Velocidad de **cada eje por separado** (filas **X p/s** y **Y p/s**): clic para editar, girar cambia de 50 en 50 entre 100 y 1000 pasos/s (`SPS_UI_MIN`, `SPS_UI_MAX`, `SPS_UI_STEP`), otro clic la aplica y la **guarda en la flash** (`dmp` / `spsx`, `spsy`). Abajo muestra su equivalente en mm/s y grd/s. El HOME y la calibracion de limits usan la de X |
| CALIBRAR > CALIBRAR LIMITS | Fila **Corr.**: clic y girar elige cuantas corridas (1 a 5, `LIMIT_CAL_MAX_RUNS`) para sacar la media; **INICIAR** arranca. Mientras corre muestra la fase y la ultima muestra (un clic cancela); al terminar muestra "Pasos riel", **"Dif corr."** (diferencia entre la corrida mas larga y la mas corta, en pasos y %: verde <0.2 %, naranja <0.5 %, rojo = esta perdiendo pasos), **"Real"** (clic y girar: se escribe el largo real medido con regla de un final de carrera al otro; propone lo que cree el equipo) y **APLICAR ESCALA** (mm por paso de X = real / pasos del riel, se guarda en flash), "Cree" y las ultimas muestras. **Al terminar el carro queda en el CENTRO del riel y ahi X = 0** (X- negativo, X+ positivo); HOME X y REFERENCIAR X llevan el carro a ese centro |
| ARRANQUE | Si al encender X no tiene referencia, aparece una pantalla que ofrece **REFERENCIAR AHORA** (busca un final de carrera y lleva X al HOME) o **DESPUES**. No se mueve nada hasta el clic |
| CALIBRAR > TOPES MIN / MAX | Filas X y Y para acomodar el laser (sin limites) y botones **X MIN, X MAX, Y MIN (arriba), Y MAX (abajo)**: cada uno guarda ahi ese tope y muestra su valor; **BORRAR TOPES** los quita. Se guardan en la flash (`dmp`: `tmask`, `tx0`, `tx1`, `ty0`, `ty1`); para fijarlos X debe estar calibrado y Y con referencia. Prioridad para MOVER: los 4 min/max; si no, el rectangulo de los puntos de la hoja; si no, libre |
| CALIBRAR > HOJA (4/6/9 PUNTOS) | Fila **Pts**: elige **4** (esquinas), **6** (esquinas + centro arriba y abajo) o **9** (esquinas, centros de los lados y centro). **INICIAR** abre el marcado guiado: la pantalla dice que punto toca ("PUNTO 2/6", "SUP CENTRO" y donde cae en la hoja); filas **X** y **Y** (clic para editar, girar mueve el laser en vivo **sin limite de angulo**) y **OK - GUARDAR PUNTO** guarda la posicion actual y pasa al siguiente; fila **LASER: ON / OFF** para ver el punto al marcar. Orden: fila de arriba de izquierda a derecha, luego la de en medio y la de abajo (izquierda = X-, derecha = X+, arriba = angulo negativo). REGRESAR cancela y deja la calibracion anterior. Al terminar se guarda en la flash (espacio `dmp`, claves `gpts`, `gx0..gx8`, `gy0..gy8`). Hoja carta acostada: 279 mm a lo largo del riel x 216 mm en el sentido del angulo, `SHEET_W_MM` / `SHEET_H_MM`; por dentro (0, 0) = centro, derecha y arriba positivos (los numeros de MOVER se cuentan desde el HOME marcado). Para iniciar, X debe estar calibrado (CALIBRAR LIMITS o HOME) y Y debe tener referencia. **Es la calibracion que corrige todo lo demas dentro de la hoja** (escala de X, grados por paso de Y, altura del laser y perpendicularidad), porque guarda los pasos reales de cada punto. Conversion de IR: **H y V se miden desde los HOME marcados**: **HOME X = V 0** y **HOME Y = H 0** (no el centro de la hoja ni del riel, y el HOME puede estar en cualquier parte del recorrido). **V (mm) mueve el motor X** desde el HOME de X (V=20 es 20 mm de X) y **H (mm) mueve Y** con la **tabla medida** de `SheetMap.h` (angulo contra mm reales, lineal entre puntos; el lado negativo es el espejo; pasado de 100 mm sigue la curva tan con D=169 mm). **IR A H 0, V 0 vuelve exactamente al HOME marcado.** Los signos (hacia donde crece V y H) salen de la hoja marcada. Lo que cae fuera de los topes (o de los puntos de la hoja si no hay topes) se **rechaza** con "V fuera de limites" o "H fuera de limites". Al re-marcar un HOME, los puntos de la hoja, topes y puntos guardados no se mueven fisicamente. Pruebas en la PC: `pio test -e native`. Supone que Y = 0 (HOME Y) es el laser perpendicular a la hoja. Sin topes min/max, el rectangulo de los puntos limita MOVER; el mapa necesita X calibrado (HOME o CALIBRAR LIMITS despues de cada encendido) |
| SENSORES | **Toda la informacion de los componentes en una lista que se recorre con la perilla** (12 renglones a la vez, barra a la derecha; un clic regresa): motor X (velocidad en pasos/s y mm/s, reduccion, mm/paso, pasos por vuelta del motor y de la salida, posicion en pasos y mm, largo del riel guardado en pasos y mm, referencia, sentido, pasos totales), motor Y (velocidad en pasos/s y grd/s, reduccion, grd/paso, pasos por vuelta, posicion, angulo, referencia, pasos totales), finales de carrera, laser, AS5600 (conexion, iman, cero, angulo), las 2 IMUs (conexion, calibracion y **inclinacion** en grados a lo largo de cada eje del chip, mas el promedio y la diferencia entre los dos extremos X- y X+; la calibracion deja ax = ay = 0 en la posicion donde se calibra, asi que hay que calibrarlas sobre una superficie realmente nivelada), hoja (puntos y tamano) y red (punto de acceso y equipos) |
| RED | Red `DMPS3`, clave, IP, equipos conectados |

El ESP32-S3 funciona **solo como punto de acceso** (no necesita router). Nota sobre el angulo: **Y lleva una reduccion de 4:1** (`Y_REDUCTION = 4`), asi que `Y_DEG_PER_STEP = 360 / 4096 / 4 = 0.02197` grados por paso. (Antes era 0.703 / 4 = 0.176: el 0.703 de la tabla es 360 / 512, por cada 8 pasos, y la pantalla marcaba ~8 veces de mas.) **Comprobado el 2026-11-12** con 6 puntos sobre la hoja (1, 2, 3, 4, 5 y 10 cm desde el centro, con el laser perpendicular en el centro; 10 cm = 1390 pasos y la pantalla vieja marcaba 244.1 grados): con 15.7 cm de la **punta** del laser a la hoja salen 0.0231 a 0.0238 grados/paso; el valor teorico cuadra si el **eje de giro** esta ~1.2 cm arriba de la punta (~16.9 cm a la hoja). **Falta medir esa distancia** para confirmarlo. En mm/grados un clic de la perilla en Y = **0.25 grados** (`Y_CLICK_DEG` en `Ui.cpp`; unos 0.7 mm sobre la hoja cerca del centro; girando rapido, 1.25); el objetivo va a multiplos de 0.25 redondeados al paso mas cercano, y se muestra con dos decimales el valor que el motor alcanza. **Holgura de Y:** al cambiar de sentido el motor da `Y_BACKLASH_STEPS` = 10 pasos extra que no se cuentan (medido: al regresar hacian falta 4 a 10 pasos de mas); se ajusta en `dmp_config.h` (0 = sin compensar). Con el AS5600 se ve el angulo real con ~0.09 grados de resolucion. **Al encender, el contador de Y toma su posicion del AS5600** (respecto al cero guardado, que no se modifica), si el sensor tiene iman y cero; si no, Y arranca en 0 donde este. El AS5600 solo da -180 a +180 grados, y HOME COMPLETO / FIJAR Y = 0 AQUI vuelven a poner Y en 0 donde este el laser (eso mueve la referencia con la que se guardaron los puntos de la hoja). Sin perilla conectada, la pantalla funciona igual pero no se puede manejar.

## LED RGB de estado

El LED de la placa indica en que esta el equipo:

| Color | Significado |
|---|---|
| **Azul** | Ejecutando algo: laser encendido, motores moviendose, HOME, CALIBRAR LIMITS, calibracion de las IMUs, **todo el marcado de los puntos de la hoja** (de INICIAR al ultimo OK) o editando un eje con la perilla. |
| **Rojo** | Nada en curso y ningun equipo conectado a la red Wi-Fi del ESP32 (red `DMPS3`). |
| **Verde** | Nada en curso y hay un equipo conectado, **o el punto de acceso esta apagado** (pantalla RED). |

Prioridad: si hay algo en curso, azul (aunque nadie este conectado); si no, rojo cuando el punto de acceso esta encendido y nadie esta conectado; si no, verde. El azul dura al menos 0.4 s para que se vea en acciones cortas.

**Punto de acceso (pantalla RED):** el boton **PUNTO ACCESO: ON / OFF** lo enciende o lo apaga. Se guarda en la flash (espacio `dmp`, clave `apon`) y se respeta al reiniciar. La pantalla entra con REGRESAR elegido, para no apagarlo por accidente. Apagado, el encabezado muestra `W-`. El brillo es `kLevel = 200` de 255 en `StatusLed.cpp` (subelo o bajalo ahi).

Segun la version de la placa, el LED esta en GPIO 48 o en GPIO 38. El firmware manda la misma senal a los dos, asi que funciona sin importar cual sea; por eso **ambos pines quedan reservados**.

## Alimentacion

- ESP32-S3 por **USB** (puerto UART/COM). Una fuente de **5 V** (sin conectar el pin 5V del ESP32 al mismo tiempo que el USB) alimenta solo los dos modulos ULN2003.
- **Todos los GND unidos**: fuente, ESP32-S3, ULN2003, sensores.
- Sensores (IMUs, AS5600, pantalla) a **3V3**, nunca a 5 V.
- Recomendado: condensador de 470 uF en el "+" y GND de los ULN2003, y 100 nF en cada sensor.
- Encendido: primero USB/ESP32, despues la fuente. Apagado: primero la fuente, despues el USB.

## Pines y cables

Todo lo que se conecta al ESP32-S3, dispositivo por dispositivo. Convencion: **rojo = alimentacion (3V3 o 5 V), negro = GND**. Los colores se repiten entre dispositivos distintos (varios son azules, blancos, etc.): se distinguen por el pin al que van.

| Dispositivo | Pin del modulo | GPIO / destino (ESP32-S3) | Color del cable |
|---|---|---|---|
| **Motor X** (ULN2003 + 28BYJ-48) | IN1 | GPIO 4 | Azul |
| | IN2 | GPIO 5 | Rosa |
| | IN3 | GPIO 6 | Amarillo |
| | IN4 | GPIO 7 | Naranja |
| | + | 5 V de la fuente | Rojo |
| | - | GND | Negro |
| **Motor Y** (ULN2003 + 28BYJ-48) | IN1 | GPIO 15 | Azul |
| | IN2 | GPIO 16 | Rosa |
| | IN3 | GPIO 17 | Amarillo |
| | IN4 | GPIO 18 | Naranja |
| | + | 5 V de la fuente | Rojo |
| | - | GND | Negro |
| **Laser** KY-008 | S (senal) | GPIO 14 | Verde |
| | pin central (+) | alimentacion aparte, como esta cableado hoy | Verde |
| | - | GND | Negro |
| **IMU blanco** (0x68) | VCC | 3V3 | Rojo |
| | GND | GND | Negro |
| | SDA | GPIO 8 | Azul |
| | SCL | GPIO 9 | Amarillo |
| | AD0 | GND | Negro |
| **IMU negro** (0x69) | VCC | 3V3 | Rojo |
| | GND | GND | Negro |
| | SDA | GPIO 8 (mismo bus) | Azul |
| | SCL | GPIO 9 (mismo bus) | Amarillo |
| | AD0 | 3V3 | Rojo |
| **AS5600** (0x36) | VCC | 3V3 | Rojo |
| | GND | GND | Negro |
| | SDA | GPIO 8 (mismo bus) | Azul |
| | SCL | GPIO 9 (mismo bus) | Amarillo |
| | DIR | GND | Negro |
| **Final de carrera X-** (inicio del riel) | un lado | GPIO 47 | Gris |
| | otro lado | GND | Negro |
| **Final de carrera X+** (final del riel) | un lado | GPIO 1 | Gris |
| | otro lado | GND | Negro |
| **Perilla KY-040** | + | 3V3 (no 5 V) | Rojo |
| | GND | GND | Negro |
| | CLK | GPIO 39 | Morado |
| | DT | GPIO 40 | Blanco |
| | SW (boton) | GPIO 41 | Cafe |
| **Pantalla TFT ST7735** 128x160 | VCC | 3V3 | Rojo |
| | GND | GND | Negro |
| | SCK | GPIO 12 | Morado |
| | SDA (MOSI) | GPIO 11 | Blanco |
| | CS | GPIO 10 | Azul |
| | DC | GPIO 13 | Cafe |
| | RES | GPIO 21 | Verde |
| | BL (luz de fondo) | 3V3 | Blanco (el segundo) |
| **LED RGB** de la placa | (ya soldado en la placa) | GPIO 48 y 38 | sin cable |

Notas: los finales de carrera son un interruptor entre el GPIO y GND (usar los contactos C y NO); no llevan resistencia porque el ESP32-S3 usa su pull-up interno. Los dos son del mismo color gris: marcarlos con cinta (X- y X+) porque si se invierten el HOME busca el lado contrario. La pantalla no devuelve datos, asi que no usa MISO y el GPIO 13 queda para DC. El pin SDA de la pantalla es su entrada de datos SPI (MOSI), no un bus I2C.

### Mapa por GPIO

| GPIO | Uso | GPIO | Uso |
|---|---|---|---|
| 1 | Final de carrera X+ | 14 | Laser |
| 2 | **libre** | 15 a 18 | Motor Y (IN1 a IN4) |
| 4 a 7 | Motor X (IN1 a IN4) | 21 | TFT RES |
| 8 | I2C SDA (IMUs y AS5600) | 38 y 48 | LED RGB de la placa |
| 9 | I2C SCL (IMUs y AS5600) | 39 | Perilla CLK |
| 10 | TFT CS | 40 | Perilla DT |
| 11 | TFT SDA (MOSI) | 41 | Perilla SW (boton) |
| 12 | TFT SCK | 42 | **libre** |
| 13 | TFT DC | 47 | Final de carrera X- |

Motores: cada jumper IN1 a IN4 lleva el mismo color que el cable de la bobina del 28BYJ-48 al que alimenta (azul, rosa, amarillo, naranja); el rojo es el comun +5 V. El conector blanco del motor solo entra de una forma en el ULN2003.

Convencion general: rojo = VCC (3V3 o 5 V), negro = GND. "IMU blanco" e "IMU negro" son solo los nombres por el color de la placa (el blanco usa AD0 a GND y el negro AD0 a 3V3).

Bus I2C: las IMUs y el AS5600 comparten SDA/SCL con los mismos colores (rojo 3V3, negro GND, azul SDA, amarillo SCL); cada uno tiene su direccion, sin conflicto.

## Modo prueba (sin finales de carrera)

**Estado actual: `TEST_NO_LIMIT_SWITCHES = false`** (HOME real, con los finales de carrera conectados) y **`REQUIRE_HOME_BEFORE_MOVE = false`**: por ahora **no se exige HOME para mover** (el HOME aun se esta definiendo); los finales de carrera siguen deteniendo el carro si lo tocan. Cuando el HOME este definido, poner `REQUIRE_HOME_BEFORE_MOVE = true`. Lo de abajo solo aplica si algun dia se vuelve a `true`.

Mientras los finales de carrera no esten conectados, el interruptor **`TEST_NO_LIMIT_SWITCHES = true`** (al inicio de `lib/DmpCore/src/dmp_config.h`) permite probar todo: se puede **mover sin hacer HOME**, y el **HOME se da por hecho al instante** (X = 0 y Y = 0 donde esten los ejes, sin buscar los interruptores ni medir el riel; el maximo de X queda en 40 cm). La pantalla muestra **TEST** en el encabezado. **Cuando conectes los finales de carrera, cambia ese valor a `false`** y vuelve a subir el firmware (`pio run -t upload`): desde entonces no se mueve nada hasta que el HOME toque X- y X+. Ojo: en modo prueba nada impide que el carro choque con un extremo, porque no hay interruptores que lo detengan.

## Limites del movimiento: solo los finales de carrera

**X no tiene limite de software** (`X_SOFT_LIMITS = false` en `dmp_config.h`): corre en los dos sentidos y la **unica restriccion son los finales de carrera de los dos lados** (`LIMIT_SWITCHES_STOP_MOTION = true`): el carro se detiene si intenta avanzar hacia un interruptor que esta activado. Tampoco hay tope en mm en la perilla ni en el panel (el aviso de "fuera de la hoja" del panel solo avisa, ya no bloquea el boton MOVER). Si algun interruptor marca "activado" por ruido o mal cableado, bloquea ese sentido: hay que revisar el cableado.

**Busqueda de interruptores:** el HOME y la calibracion de limits avanzan **hasta tocar** el interruptor, sin limite de distancia.

**X libre hasta calibrar:** mientras X no este calibrado (despues de encender, o si la calibracion se cancela o falla), los finales de carrera **no cortan el movimiento** (`LIMITS_FREE_UNTIL_CALIBRATED = true`): se puede mover en cualquier sentido aunque el carro este pegado a un interruptor, o aunque ambos marquen "activado", para acomodarlo a mano. Cuando la calibracion de limits termina bien, los interruptores vuelven a detener el carro al tocarlos.

## Calibracion de los finales de carrera (limits)

**Escala de X (medida el 2026-11-12):** con 8:1 la pantalla marcaba 1764 mm y el carro avanzo 280 mm reales (factor 6.3), asi que la reduccion efectiva es 8 x 6.3 = **50.4** (`X_REDUCTION = 50.4` en `dmp_config.h`) y `X_MM_PER_STEP = 0.0613 / 50.4 = 0.0012163` mm por paso. A 1000 pasos/s el carro avanza unos 1.2 mm/s. Es una sola medicion de ~28 cm: conviene repetirla con CALIBRAR LIMITS ("Cree" contra el riel medido) y ajustar `X_REDUCTION` si cambia. **Sentido del motor X:** con los finales de carrera intercambiados (GPIO 47 = X-, GPIO 1 = X+) `X_INVERT = false`: pasos negativos van hacia X-. Si el carro no se despega del interruptor presionado, esta invertido: poner `true` (la calibracion de limits tambien lo detecta y lo corrige sola, ver abajo).

Se activa desde la pantalla del equipo (**CALIBRAR -> CALIBRAR LIMITS**) o por el monitor serie con `L` (115200; ademas `C` cancela y `Z` marca el HOME de Y donde este el laser, igual que CALIBRAR > HOME (MARCAR) > MARCAR Y). **Funciona por lado, no por etiqueta:** en cuanto **cualquier** interruptor toca, el carro se **detiene** y sale en **sentido contrario** al que lo trajo; los pasos de cada tramo se cuentan **desde que se despega el interruptor del que sale hasta que toca el otro** (cada cruce es una **muestra**). **Si empieza con un interruptor presionado, solo sale hacia el otro lado.** Con las **muestras elegidas en la pantalla** (1 a 5; por el monitor serie, `LIMIT_CAL_SAMPLES` = 3) saca el **promedio**, toma X- como X = 0 y se **posiciona en medio** (la mitad del promedio). **No hay ningun limite de distancia**: avanza hasta que el interruptor TOQUE y solo se detiene si se cancela (clic de la perilla o `C` por el monitor serie). **Sentido del motor corregido solo:** si al salir de un interruptor presionado este no se despega tras `LIMIT_DIR_DETECT_STEPS` (13 050 pasos, unos 16 mm con la escala medida; no es un limite de recorrido, solo decide el sentido), la calibracion entiende que los pasos estaban al reves, invierte el motor X, **lo guarda en la flash** (clave `xinv`, tiene prioridad sobre `X_INVERT`) y avisa por el monitor serie. Si tampoco se despega en el otro sentido, se detiene con error (`Final no se despega`, antes `limit switch does not release`: interruptor atorado o motor sin moverse; el monitor serie imprime `[ref]` con cada paso del proceso y los pines). En la pantalla se ve la fase (saliendo de X-, hacia X+ 2/3...), los pasos de la ultima muestra y, al terminar, el promedio y cada muestra en CALIBRAR LIMITS (fila "Pasos riel"); el monitor serie imprime cada muestra y el resultado.

Por el monitor serie (115200) tambien se puede: **`L`** inicia la calibracion y **`C`** la cancela; imprime cada muestra y el resultado.

## Home de X (modo normal)

**Sin home optico:** en el S3 ya no hay sensor optico K400. **Hasta hacer HOME, el equipo no permite mover** (ni desde la pantalla ni desde el panel; responde "Haga HOME primero"), porque al encender no sabe donde esta el carro. Al pulsar HOME, X va hacia atras hasta tocar el fin de carrera X- (**ese punto es X = 0**), avanza hasta tocar el X+ (mide el largo del riel, que se muestra en la pantalla y en el panel) y regresa a 0. El maximo permitido de X pasa a ser ese largo menos 2 mm (`X_END_MARGIN_CM`), sin pasar de 40 cm. Si no encuentra algun final de carrera, el HOME se detiene con error y X sigue sin referencia. El cero de Y es donde esta el laser en ese momento (se pone a mano antes de hacer HOME).

**Rangos:** **X sin limite de software** (solo los finales de carrera) y **Y (angulo) sin limite** (`Y_MIN_DEG` / `Y_MAX_DEG` = +/-100000 grados es solo un tope tecnico; sentido invertido: `ANGLE_SIGN = -1`, solo cambia el signo que se muestra y se escribe; no toca el cableado del motor) desde el home (el 0 es donde quedo el laser al fijar el cero).15 cm); si la colocas en otro lugar, cambia `SHEET_LEFT_MM` en `web/index.html`. El modo triangular (hipotenusa + angulo) mide desde X = 20 cm.

### Pantalla TFT ST7735 128x160 (opcional)

Cableado en la tabla de arriba y pines en `dmp_pins.h`, sin choque con motores ni sensores (SPI2 por hardware). El firmware completo ya la usa. Ademas hay pruebas sueltas, que reemplazan al firmware completo mientras estan cargadas y no mueven motores:

| Entorno | Para que sirve | Comando |
|---|---|---|
| `tft_test` | Colores y texto (revisa el orden RGB) | `pio run -e tft_test -t upload` |
| `tft_gcode` | Terminal de G-code: se escribe en el monitor serie y la pantalla muestra la linea y su interpretacion | `pio run -e tft_gcode -t upload` |
| `tft_ui` | Prototipo antiguo de la interfaz (ya integrada al firmware completo): posicion simulada, sin motores | `pio run -e tft_ui -t upload` |

Las librerias `Adafruit GFX` y `Adafruit ST7735` las baja PlatformIO solas. Para volver al firmware completo: `pio run -t upload`. Los colores de la pantalla ya se verificaron en el equipo. Para volver a ver la prueba de color al arrancar (tres barras ROJO, VERDE, AZUL), poner `SHOW_COLOR_CHECK = true` en `Ui.cpp`; si el rojo se ve azul, `TFT_SWAP_RB = true`; si todo se ve en negativo, `TFT_INVERT = true`.


## Por que estos pines

Pines **prohibidos** (no se usan para nada) y la razon:

| GPIO | Motivo |
|---|---|
| 0, 3, 45, 46 | Arranque (strapping): un nivel mal puesto al encender impide que el chip arranque o cambia su modo |
| 19, 20 | USB nativo (D- / D+) |
| 26 a 32 | Flash y PSRAM internas: usarlos cuelga el chip |
| 33 a 37 | PSRAM octal de las placas N16R8 |
| 43, 44 | UART0: es el monitor serie por el puente CH343 |
| 38, 48 | LED RGB de la placa |

Los pines elegidos (1, 4 a 9, 47, 14 a 18, y 10 a 13 y 21 para la pantalla) son GPIO normales: sin funcion especial al arrancar, con pull-up interno, y sin pulsos raros durante el reset (importante para que los motores y el laser no se disparen al encender). Todo es I/O digital, asi que el Wi-Fi no interfiere.

Encoder KY-040: CLK 39, DT 40 y SW 41 (el modulo lleva pull-ups propios a "+", por eso se alimenta a 3V3). **Quedan libres:** 2 y 42; sin pantalla, ademas 10, 11, 12, 13 y 21 (alcanza para otro KY-040 completo).

## Red

El ESP32-S3 crea su propia red Wi-Fi, con nombre distinto al del DOIT para que puedan estar encendidos al mismo tiempo sin confundirse. No necesita router y **la IP no cambia nunca**.

| | |
|---|---|
| Red | `DMPS3` |
| Contrasena | `PIA2026.` (con el punto; minimo 8 caracteres) |
| IP del ESP32-S3 | `192.168.4.1` (fija) |

Nota: el DOIT usa la red `DMPESP` con la misma IP `192.168.4.1`; si los dos estan encendidos, conectate a la red de la placa que quieras usar.

El nombre y la contrasena se cambian en `lib/DmpCore/src/Net.cpp` o con `AP_SSID` y `AP_PASSWORD` en `include/secrets.h` (ese archivo no se sube al repositorio).

## Altura del laser y velocidad

- **Altura laser fija: 15 cm** (`Cfg::LASER_HEIGHT_CM`). Ya no se captura en la interfaz: se quito el campo y la calibracion de 2 puntos. El comando `G92` ahora solo acepta `A` (compensacion angular).
- **Velocidad:** se elige por eje en la pantalla (AJUSTES > VELOCIDAD, 100 a 1000 pasos/s) y se guarda en la flash; `SPS_MAX = 1000` pasos/s es el valor inicial. **Los pasos los da un temporizador de hardware** (`esp_timer`, resolucion de microsegundos; uno por motor, que se reprograma para el instante exacto del siguiente paso), asi que `SPS_MAX` puede ser **cualquier valor de 10 a 2000** (por ejemplo 850), no solo 1000, 500, 333... La tarea de 1 ms ya no da pasos: solo vigila los finales de carrera, el HOME y la calibracion. `Stepper28` arranca a `RAMP_START_SPS` = 80 pasos/s y **acelera de forma suave** (`RAMP_ACCEL_SPS` = +1.5 pasos/s en cada paso: unos 610 pasos y 1.7 s hasta 1000, en X ~0.75 mm) y frena igual al llegar; un movimiento corto sube y baja sin llegar a la velocidad maxima. Antes arrancaba a 125 y subia 10 por paso (88 pasos, 0.17 s), muy brusco para un motor cargado; X no tiene encoder, asi que el firmware no puede detectar pasos perdidos, solo evitar que se pierdan. Las bobinas se quedan energizadas 2 s despues del ultimo paso (`MOTOR_HOLD_MS`, antes 0.3 s) para que la carga no mueva el eje entre un movimiento y otro. La busqueda de interruptores y la calibracion de limits tambien van a esa velocidad (`HOME_SPS = SPS_MAX`). Un 28BYJ-48 esta cerca de su limite a 1000 pasos/s: si pierde pasos o zumba sin avanzar, bajar `SPS_MAX` poco a poco (900, 800...). Con la escala medida de X (0.0012 mm/paso), 1000 pasos/s son unos 1.2 mm/s (el riel de ~28 cm tarda unos 4 minutos por tramo).
- **Hoja del proyecto: 21.5 x 27.7 cm** (la vista del panel la dibuja con esa medida).
- **Compensacion angular con el AS5600** (sensor magnetico de efecto Hall en el eje del laser):
  1. Poner el laser perpendicular a la hoja y pulsar **FIJAR CERO** en el panel (queda guardado en flash, sobrevive al reinicio).
  2. Activar **Auto con AS5600** y pulsar **CALIBRAR**: el firmware calcula `compensacion = angulo del motor - angulo real medido` y la pone en el campo.
  3. Sin sensor, sin iman o sin cero fijado, CALIBRAR con la casilla activa responde error y no cambia nada; con la casilla apagada, el campo sigue siendo manual.
  - Constantes en `dmp_config.h`: `ENC_INVERT` (si el sensor gira al reves que el motor) y `ENC_RATIO` (vueltas del iman por vuelta del laser; 1.0 si el iman va directo en el eje). Aun sin probar con hardware.
  - **Home del laser (referencia 0 definida el 2026-11-09):** el cero del AS5600 se fijo donde estaba el laser, con el registro en **raw = 2984 (262.27 grados)**, iman en zona ideal (AGC 45). Esa es la posicion de referencia para el angulo; queda guardada en la flash y se conserva al reiniciar o volver a subir el firmware. En ese mismo punto los contadores de los motores X e Y arrancaron en 0 (el home de los dos ejes es la posicion actual). Para redefinirla: pantalla CALIBRAR -> FIJAR CERO AS5600, o el boton FIJAR CERO del panel.

## Por calibrar (`lib/DmpCore/src/dmp_config.h`)

`X_MM_PER_STEP`, `Y_DEG_PER_STEP` (tomados del Readme, no medidos), `X_INVERT`/`Y_INVERT`, `X_LIMIT_MIN_POS_CM` y `SPS_MAX`.

## Problemas conocidos

- **Subida falla con "puerto ocupado" o "no existe":** hay un monitor serie abierto, o el cable esta en el otro USB-C (hay que usar el marcado UART/COM).
- **El ESP32 se reinicia cuando mueven los motores o el Wi-Fi transmite:** falta el condensador de 470 uF o la fuente se queda corta.
- **AS5600 dice "not found":** cablearlo a GPIO 8/9 y acercar el iman diametral a ~2 mm, centrado sobre el chip.
- **El LED no se enciende o el color esta cambiado:** avisar para ajustar el orden de color (`StatusLed.cpp`).
- **Un sensor no responde:** revisar 3V3, GND y que SDA/SCL no esten cruzados.
