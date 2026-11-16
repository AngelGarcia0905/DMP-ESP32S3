#include "Net.h"

#include <Preferences.h>
#include <WiFi.h>

#include "dmp_config.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef AP_SSID
#define AP_SSID "DMPS3"  // distinto al del ESP32 DOIT ("DMPESP") para no confundirse
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "PIA2026."  // WPA2 needs 8 characters or more
#endif

namespace Net {

static bool g_on = true;  // punto de acceso encendido (se guarda en la flash: "dmp" / "apon")

static bool startAp() {
  WiFi.mode(WIFI_AP);
  WiFi.setHostname(Cfg::HOSTNAME);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);  // channel 1, visible, up to 4 clients
  if (ok) {
    Serial.printf("[net] Punto de acceso \"%s\" listo (%s). Sin interfaz web: todo se maneja desde la pantalla.\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[net] Could not start the access point (password must be 8+ characters).");
  }
  return ok;
}

// Access point only: the ESP32 creates its own network, no router needed.
// Si en la pantalla RED se apago, arranca apagado.
bool begin() {
  WiFi.persistent(false);
  Preferences p;
  if (p.begin("dmp", true)) {
    g_on = p.getBool("apon", true);
    p.end();
  }
  if (!g_on) {
    WiFi.mode(WIFI_OFF);
    Serial.println("[net] Punto de acceso APAGADO (se enciende en la pantalla RED).");
    return false;
  }
  return startAp();
}

bool enabled() { return g_on; }

void setEnabled(bool on) {
  if (on == g_on) return;
  g_on = on;
  Preferences p;
  if (p.begin("dmp", false)) {
    p.putBool("apon", on);
    p.end();
  }
  if (on) {
    startAp();
  } else {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[net] Punto de acceso apagado.");
  }
}

int clients() { return g_on ? WiFi.softAPgetStationNum() : 0; }

const char* ssid() { return AP_SSID; }
const char* password() { return AP_PASSWORD; }

String ip() { return WiFi.softAPIP().toString(); }

// No internet in AP mode, so there is no clock to read.
String isoNow() { return ""; }

}  // namespace Net
