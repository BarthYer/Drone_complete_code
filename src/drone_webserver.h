#ifndef DRONE_WEBSERVER_H
#define DRONE_WEBSERVER_H

#include "drone_receiver.h"

/*
 * Start WiFi Soft-AP and HTTP control server in a background thread.
 * Call once from drone_fsm_run() after hardware init.
 *
 * AP SSID : "DroneControl"  (open, no password)
 * ESP32 IP : 192.168.4.1
 * HTTP port : 80
 *
 * Endpoints:
 *   GET  /           → HTML control page (joysticks + telemetry)
 *   GET  /telemetry  → JSON  {"roll":X,"pitch":X,"gyro_z":X,"state":"FLYING"}
 *   POST /control    → JSON  {"throttle":0.0-1.0,"yaw":-1–1,"pitch":-1–1,"roll":-1–1,"active":bool}
 */
void drone_webserver_start(void);

/*
 * Return the latest RC packet received via the web interface.
 * drone_active is forced false if no /control POST was received in the last 500 ms.
 */
struct DataPackage drone_webserver_get_rc(void);

#endif /* DRONE_WEBSERVER_H */
