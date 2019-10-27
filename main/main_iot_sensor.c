/*	ESP32 IoT Light Sensor for Amazon cloud

	- Connect to WiFi
	- Connect to AWS IoT cloud
	- Get data from GY-302 (BH1750) ambient light sensor
	- Publish data into cloud
*/
#include <stdio.h>
#include <string.h>
#include "../build/config/sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#define 	INCLUDE_xTaskGetHandle		1

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "driver/i2c.h"
#include "cJSON.h"

#include "aws_iot_mqtt_client_interface.h"

#include "aws_config.h"
// Here is a private file with WiFi connection details
// It contains defines for WIFI_SSID and WIFI_PASSWORD
#include "wifi_credentials.h"
//-----------------------------------------------------------------------------
// Define TAGs for log messages
#define	TAG_MAIN	"APP"
#define TAG_WIFI	"WIFI"
#define TAG_I2C		"I2C"
#define TAG_AWS		"AWS"
//-----------------------------------------------------------------------------
#define LED_PIN				GPIO_NUM_2
// Light sensor connection details - use default ESP32 I2C pins
#define SENSOR_SDA_PIN		GPIO_NUM_21
#define SENSOR_SCL_PIN		GPIO_NUM_22
#define SENSOR_ADDR			0x23
// Sensor start operation command after power-on
#define SENSOR_CMD_START	0x01
// Sensor mode - 0x10 high res, 0x13 low res
#define SENSOR_CMD_MODE		0x10
// How often to get data from light sensor, ms
#define I2C_POLL_INTERVAL	5000
// how often to publish data to the cloud, s
#define AWS_PUB_INTERVAL	300
#define MAX_JSON_SIZE		64
//-----------------------------------------------------------------------------
char AWS_host[255] = AWS_HOST;
uint32_t AWS_port = AWS_PORT;
//-----------------------------------------------------------------------------
static int wifi_retry = 0;
/* FreeRTOS event group to to check signals */
static EventGroupHandle_t events_group;
const int IP_UP_BIT = BIT0;	// Bit to check IP link readiness
TaskHandle_t aws_iot_task_handle = NULL;
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
			xEventGroupClearBits(events_group, IP_UP_BIT);
			break;
		case WIFI_EVENT_STA_CONNECTED:
			wifi_retry = 0;
			ESP_LOGI(TAG_WIFI, "Connected");
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			wifi_retry++;
			ESP_LOGI(TAG_WIFI, "Reconnection attempt %d", wifi_retry);
			esp_wifi_connect();
			xEventGroupClearBits(events_group, IP_UP_BIT);
			break;
		}
	}

	if (event_base == IP_EVENT)
	{
		switch(event_id)
		{
		case IP_EVENT_STA_GOT_IP:
			xEventGroupSetBits(events_group, IP_UP_BIT);
			ESP_LOGI(TAG_WIFI, "Received IP: %s", ip4addr_ntoa(&((ip_event_got_ip_t*)event_data)->ip_info.ip));
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
// TODO
// ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
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
static esp_err_t i2c_get_data(uint8_t *data_h, uint8_t *data_l)
{
	int res = 0;
	i2c_cmd_handle_t cmd;

// 1. set operation mode
// |-------|---------------------|----------------|------|
// | start | addr + wr_bit + ack | op_mode + ack  | stop |
// |-------|---------------------|----------------|------|
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_ADDR << 1 | I2C_MASTER_WRITE, 0x01 /*expect ACK*/);
    i2c_master_write_byte(cmd, SENSOR_CMD_MODE, 0x01 /*expect ACK*/);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (res != ESP_OK)
        return res;

// 2. wait more than 24 ms
    vTaskDelay(180 / portTICK_RATE_MS);

// 3. read data
// |-------|---------------------|--------------------|--------------------|------|
// | start | addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
// |-------|---------------------|--------------------|--------------------|------|
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_ADDR << 1 | I2C_MASTER_READ, 0x01 /*expect ACK*/);
    i2c_master_read_byte(cmd, data_h, 0x00 /*ACK*/);
    i2c_master_read_byte(cmd, data_l, 0x01 /*NACK*/);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return res;
}
//-----------------------------------------------------------------------------
void read_sensor_task(void *arg)
{
	int res;
	uint8_t sensor_data_h, sensor_data_l;
	uint32_t raw_sensor_data;

	// do job forever
	while(1)
	{
		res = i2c_get_data(&sensor_data_h, &sensor_data_l);
		if (res != ESP_OK)
			ESP_LOGE(TAG_I2C, "Command failure, 0x%x", res);
		else
		{
			raw_sensor_data = sensor_data_h << 8 | sensor_data_l;
			ESP_LOGI(TAG_I2C, "Sensor raw: %d", raw_sensor_data);

			if (aws_iot_task_handle != NULL)
			{
				xTaskNotify(aws_iot_task_handle, raw_sensor_data, eSetValueWithOverwrite);
				ESP_LOGI(TAG_I2C, "AWS task notified");
			}
			else
				ESP_LOGI(TAG_I2C, "No AWS task to notify");
        }
//		// blink led
//		gpio_set_level(LED_PIN, led);
//		if (led == 1)
//			led = 0;
//		else
//			led = 1;

		vTaskDelay(I2C_POLL_INTERVAL / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}
//-----------------------------------------------------------------------------
void i2c_start(void)
{
	int res;
	i2c_config_t cfg;
	i2c_cmd_handle_t cmd;

	ESP_LOGI(TAG_I2C, "Initialization started");
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SENSOR_SDA_PIN;
    cfg.scl_io_num = SENSOR_SCL_PIN;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &cfg);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, cfg.mode, 0, 0, 0));

	ESP_LOGI(TAG_I2C, "Power-on");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, SENSOR_ADDR << 1 | I2C_MASTER_WRITE, 0x01 /*expect ACK*/));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, SENSOR_CMD_START, 0x01 /*expect ACK*/));
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (res != ESP_OK)
    	ESP_LOGE(TAG_I2C, "Power-on failure, 0x%x", res);
    i2c_cmd_link_delete(cmd);

	xTaskCreate(read_sensor_task, "i2c_bh1750_task", 2048, (void *)0, 10, NULL);
	ESP_LOGI(TAG_I2C, "Task created");
}
//-----------------------------------------------------------------------------
void aws_disconnect_handler(AWS_IoT_Client *pClient, void *data)
{
    ESP_LOGW(TAG_AWS, "MQTT Disconnect");
}
//-----------------------------------------------------------------------------
void aws_iot_task(void *arg)
{
    IoT_Error_t rc = FAILURE;
    AWS_IoT_Client aws_client;
	IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
	IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

	ESP_LOGI(TAG_AWS, "MQTT init started");
	mqttInitParams.enableAutoReconnect = false; // We enable this later below
	mqttInitParams.pHostURL = AWS_host;
	mqttInitParams.port = AWS_port;
	mqttInitParams.pRootCALocation = aws_root_ca_pem;
    mqttInitParams.pDeviceCertLocation = certificate_pem_crt;
    mqttInitParams.pDevicePrivateKeyLocation = private_pem_key;
    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = aws_disconnect_handler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&aws_client, &mqttInitParams);
    if (rc != SUCCESS)
    {
        ESP_LOGE(TAG_AWS, "MQTT init failure: %d ", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_AWS, "MQTT init success");

    ESP_LOGI(TAG_AWS, "Wait for IP link");
    xEventGroupWaitBits(events_group, IP_UP_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG_AWS, "IP link is up");

    ESP_LOGI(TAG_AWS, "MQTT connect started");
    connectParams.keepAliveIntervalInSec = 60;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    connectParams.pClientID = AWS_CLIENTID;
    connectParams.clientIDLen = (uint16_t) strlen(AWS_CLIENTID);
    connectParams.isWillMsgPresent = false;
    do
    {
        rc = aws_iot_mqtt_connect(&aws_client, &connectParams);
        if(rc != SUCCESS) {
            ESP_LOGE(TAG_AWS, "MQTT error: %d", rc);
            vTaskDelay(5000 / portTICK_RATE_MS);
        }
    }
    while(rc != SUCCESS);
    ESP_LOGI(TAG_AWS, "MQTT connected");

    rc = aws_iot_mqtt_autoreconnect_set_status(&aws_client, true);
    if(rc == SUCCESS)
    	ESP_LOGI(TAG_AWS, "MQTT auto-reconnect enabled");
    else
    	ESP_LOGE(TAG_AWS, "MQTT auto-reconnect setup failure: %d ", rc);

    char cPayload[128];
    uint32_t sensor_data;
    IoT_Publish_Message_Params paramsQOS0;
    cJSON *root;
    char JSON_buffer[MAX_JSON_SIZE];

    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *) cPayload;
    paramsQOS0.isRetained = 0;
    const int topic_len = strlen(AWS_TOPIC);
    while(1)
    {
    	rc = aws_iot_mqtt_yield(&aws_client, 100);
    	if (rc != SUCCESS)
    		continue;

    	xTaskNotifyWait(0x00, ULONG_MAX, &sensor_data, 0);
    	ESP_LOGI(TAG_AWS, "Got from I2C task %d", sensor_data);
        // Make a test with JSON simple message:
        //        {
        //        	"data": 12.345
        //        }
        root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "data", sensor_data);
        if (!cJSON_PrintPreallocated(root, JSON_buffer, MAX_JSON_SIZE, 1 /* formatted */))
        {
        	ESP_LOGW(TAG_AWS, "JSON buffer too small");
            JSON_buffer[0] = 0;
        }
        cJSON_Delete(root);
        ESP_LOGI(TAG_AWS, "JSON message: %s", JSON_buffer);

        paramsQOS0.payload = (void *) JSON_buffer;
        paramsQOS0.payloadLen = strlen(JSON_buffer);
        rc = aws_iot_mqtt_publish(&aws_client, AWS_TOPIC, topic_len, &paramsQOS0);
        if (rc == SUCCESS)
        	ESP_LOGI(TAG_AWS, "MQTT message published");
        else
          	ESP_LOGE(TAG_AWS, "MQTT publish failure: %d ", rc);

        ESP_LOGI(TAG_AWS, "Stack remaining: %d bytes", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(AWS_PUB_INTERVAL * 1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}
//-----------------------------------------------------------------------------
void aws_start(void)
{
	xTaskCreate(aws_iot_task, "aws_iot_task", 10240, (void *)0, 5, &aws_iot_task_handle);
}
//-----------------------------------------------------------------------------
void app_main(void)
{
	ESP_LOGI(TAG_MAIN, "Light Sensor v1.0 STARTED");
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_LOGI(TAG_MAIN, "Flash initialized");
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_LOGI(TAG_MAIN, "Event loop created");
	events_group = xEventGroupCreate();
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

	wifi_start();

	i2c_start();

	aws_start();
}
//-----------------------------------------------------------------------------
