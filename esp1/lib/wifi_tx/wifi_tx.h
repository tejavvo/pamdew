/**
 * @file wifi_tx.h
 * @brief WiFi station init and HTTP POST helper.
 *
 * Usage
 * -----
 *   wifi_tx_init("MySSID", "MyPassword");   // blocks until IP is obtained
 *   wifi_tx_post_json(json_string);          // POST to configured endpoint
 */
#pragma once

#include <stdbool.h>

/* ── Server endpoint ────────────────────────────────────────────────── */
#define WIFI_TX_HOST  "192.168.211.200"   /* ← replace with server IP    */
#define WIFI_TX_PORT  8080              /* ← replace with server port  */
#define WIFI_TX_PATH  "/api/sensor_data"

/**
 * @brief Connect to WiFi in station mode and wait for an IP address.
 *        Blocks the calling task. Retries indefinitely every 5 s on failure.
 *
 * @param ssid      WiFi network name.
 * @param password  WiFi password (use "" for open networks).
 */
void wifi_tx_init(const char *ssid, const char *password);

/**
 * @brief HTTP POST a JSON string to the configured endpoint.
 *
 * @param json  Null-terminated JSON string.
 * @return      true on HTTP 2xx response, false otherwise.
 */
bool wifi_tx_post_json(const char *json);
