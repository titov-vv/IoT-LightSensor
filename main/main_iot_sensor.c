/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
// Here is a private file with WiFi connection details
#include "wifi_credentials.h"
//-----------------------------------------------------------------------------
#define	TAG_MAIN	"APP"
#define TAG_WIFI	"WIFI"
//-----------------------------------------------------------------------------
static int wifi_retry = 0;
//-----------------------------------------------------------------------------
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT)
	{
		switch(event_id)
		{
		case WIFI_EVENT_STA_START:
			ESP_LOGI(TAG_WIFI, "Connecting...");
			esp_wifi_connect();
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			ESP_LOGI(TAG_WIFI, "Connected");
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			wifi_retry++;
			ESP_LOGI(TAG_WIFI, "Reconnection attempt %d", wifi_retry);
			esp_wifi_connect();
			break;
		}
	}

	if (event_base == IP_EVENT)
	{
		switch(event_id)
		{
		case IP_EVENT_STA_GOT_IP:
			ESP_LOGI(TAG_WIFI, "Received IP: %s", ip4addr_ntoa(&((ip_event_got_ip_t*)event_data)->ip_info.ip));
			wifi_retry = 0;
			break;
		}
	}
}
//-----------------------------------------------------------------------------
void wifi_start(void)
{
	ESP_LOGI(TAG_WIFI, "Initialization started");
	tcpip_adapter_init();
	wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
	ESP_LOGI(TAG_WIFI, "Initialization completed");

	ESP_LOGI(TAG_WIFI, "Connect to '%s'", WIFI_SSID);
	wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD }};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG_WIFI, "Connection start...");
}
//-----------------------------------------------------------------------------
void app_main(void)
{
	ESP_LOGI(TAG_MAIN, "Light Sensor v1.0 STARTED");
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_LOGI(TAG_MAIN, "Flash initialized");
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_LOGI(TAG_MAIN, "Event loop created");

	wifi_start();
}
//-----------------------------------------------------------------------------
