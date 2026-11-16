# Conexiones del ESP32-S3

Placa: ESP32-S3 DevKitC-1 N16R8, programada por el USB-C marcado **UART/COM** (en esta PC salio como `COM21`; PlatformIO detecta el puerto solo).
Convencion de cables: **rojo = alimentacion (3V3 o 5 V), negro = GND**. Los colores se repiten entre dispositivos distintos (varios son azules, blancos, etc.): se distinguen por el pin al que van.

## Alimentacion

- El ESP32-S3 se alimenta por **USB** (puerto UART/COM).
- Una fuente de **5 V** alimenta **solo** los dos modulos ULN2003 (no conectar el pin 5V del ESP32 a la fuente mientras este el USB).
- **Todos los GND unidos**: fuente, ESP32-S3, ULN2003 y sensores.
- Sensores, pantalla y perilla a **3V3**, nunca a 5 V.
- Recomendado: condensador de 470 uF en el "+" y GND de los ULN2003.

## Conexiones por dispositivo

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

## Notas de cableado

- **Motores:** cada jumper IN1 a IN4 lleva el mismo color que el cable de la bobina del 28BYJ-48 al que alimenta; el rojo es el comun +5 V. El conector blanco del motor solo entra de una forma en el ULN2003.
- **IMUs y AS5600:** comparten SDA y SCL (GPIO 8 y 9) con los mismos colores; cada uno tiene su direccion, sin conflicto. "IMU blanco" e "IMU negro" son solo los nombres por el color de la placa (el blanco usa AD0 a GND y el negro AD0 a 3V3).
- **Finales de carrera:** son dos interruptores independientes, cada uno entre su GPIO y GND (usar los contactos **C** y **NO**). **No llevan resistencia**: el firmware activa el pull-up interno del ESP32-S3 y los detecta cuando el pin baja a LOW. Los dos son del mismo color gris: marcarlos con cinta (X- y X+), porque si se invierten el HOME busca el lado contrario.
- **Perilla KY-040:** el modulo trae sus propios pull-ups hacia "+", por eso se alimenta a 3V3 (a 5 V mandaria 5 V a los GPIO). Si al girar a la derecha el valor baja, no se recablea: se cambia `KNOB_INVERT` en `Ui.cpp`.
- **Pantalla TFT:** el pin SDA es su entrada de datos SPI (MOSI), no un bus I2C. No usa MISO, asi que el GPIO 13 queda para DC.
- **Laser KY-008:** la senal S acepta los 3.3 V del ESP32-S3; el pin central se alimenta aparte.

## Mapa por GPIO

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

## Pines que no se usan (y por que)

| GPIO | Motivo |
|---|---|
| 0, 3, 45, 46 | Arranque (strapping): un nivel mal puesto al encender impide arrancar |
| 19, 20 | USB nativo |
| 26 a 32 | Flash y PSRAM internas |
| 33 a 37 | PSRAM octal de las placas N16R8 |
| 43, 44 | UART0: es el monitor serie por el puente USB |

Los pines estan definidos en `lib/DmpCore/src/dmp_pins.h`. Si se agrega algo, anotar el pin ahi primero.
