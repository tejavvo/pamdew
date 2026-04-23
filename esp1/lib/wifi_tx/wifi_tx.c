/**
 * @file wifi_tx.c
 * @brief WiFi station + HTTP POST implementation using ESP-IDF components.
 *
 * Components used (all built-in to ESP-IDF / espressif32 platform):
 *   esp_wifi, esp_event, esp_netif, esp_http_client, nvs_flash
 */

#include "wifi_tx.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"

/* ── Private ────────────────────────────────────────────────────────── */
static const char *TAG = "wifi_tx";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    10

static EventGroupHandle_t s_wifi_eg;
static int                s_retry = 0;

/* ── Event handler ──────────────────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
            s_retry++;
            ESP_LOGW(TAG, "WiFi disconnected – retry %d/%d",
                     s_retry, WIFI_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries",
                     WIFI_MAX_RETRIES);
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* ── Public API ─────────────────────────────────────────────────────── */

void wifi_tx_init(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);

    /* NVS is required by the WiFi driver */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_wifi_eg = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* Register both WiFi and IP events on the same handler */
    esp_event_handler_instance_t h_wifi, h_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, &h_wifi);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, &h_ip);

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     ssid,     sizeof(wifi_cfg.sta.ssid)     - 1);
    strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    /* Block until connected or failed */
    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(s_wifi_eg,
                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                   pdFALSE, pdFALSE,
                                   pdMS_TO_TICKS(10000));

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi connected");
            break;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "Connection failed – retrying in 5 s");
            s_retry = 0;
            xEventGroupClearBits(s_wifi_eg, WIFI_FAIL_BIT);
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_wifi_connect();
        }
        /* timeout – keep waiting */
    } while (1);

    /* Clean up event handler instances (connection is persistent) */
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, h_wifi);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, h_ip);
}

bool wifi_tx_post_json(const char *json)
{
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d%s",
             WIFI_TX_HOST, WIFI_TX_PORT, WIFI_TX_PATH);

    esp_http_client_config_t cfg = {
        .url            = url,
        .method         = HTTP_METHOD_POST,
        .timeout_ms     = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, (int)strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    bool ok = false;

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ok = (status >= 200 && status < 300);
        ESP_LOGI(TAG, "POST %s → HTTP %d", url, status);
    } else {
        ESP_LOGE(TAG, "POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return ok;
}
