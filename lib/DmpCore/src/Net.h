#pragma once
#include <Arduino.h>

namespace Net {

// Starts the ESP32's own Wi-Fi access point (AP_SSID / AP_PASSWORD, optionally overridden in
// include/secrets.h). Solo sirve para que el LED y la pantalla RED sepan si hay equipos conectados: no hay interfaz web.
// Returns true when it is up.
bool begin();

// Cuantos equipos estan conectados al punto de acceso ahora mismo (0 si esta apagado).
int clients();

// Punto de acceso encendido o apagado (pantalla RED). Se guarda en la flash (espacio "dmp", clave "apon") y se
// respeta al reiniciar. Apagado: el Wi-Fi queda desactivado y el LED no se pone rojo por falta de equipos.
bool enabled();
void setEnabled(bool on);

// Nombre y clave del punto de acceso (para mostrarlos en la pantalla).
const char* ssid();
const char* password();

// IP address of the access point (192.168.4.1 by default).
String ip();

// Local time as "YYYY-MM-DDTHH:MM:SS", or "" when no clock is available (always, in AP mode).
String isoNow();

}  // namespace Net
