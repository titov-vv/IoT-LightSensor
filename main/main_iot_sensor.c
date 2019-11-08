/*	ESP32 IoT Light Sensor for Amazon cloud

	- Connect to WiFi
	- Connect to AWS IoT cloud and get settings from shadow
	- Get data from GY-302 (BH1750) ambient light sensor
	- Publish data into cloud with regards to settings
*/
//-----------------------------------------------------------------------------
#include "../build/config/sdkconfig.h"
// Espressif
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
// Project
#include "main.h"
#include "blink.h"
#include "wifi.h"
#include "i2c.h"
#include "thing.h"
//-----------------------------------------------------------------------------
// FreeRTOS event group to to synchronize between tasks
EventGroupHandle_t 	events_group;
// FreeRTOS queue to make a data flow from sensor to thing
QueueHandle_t 		data_queue;
//-----------------------------------------------------------------------------
void app_main(void)
{
	ESP_LOGI(TAG_MAIN, "Light Sensor v1.0 STARTED");
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_LOGI(TAG_MAIN, "Flash initialized");
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_LOGI(TAG_MAIN, "Event loop created");
	events_group = xEventGroupCreate();
	data_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	ESP_LOGI(TAG_MAIN, "Queue and event group initialized");

	blink_start();

	wifi_start();

	i2c_start();

	aws_start();
}
//-----------------------------------------------------------------------------
