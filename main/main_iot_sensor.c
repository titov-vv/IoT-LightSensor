/*	ESP32 IoT Light Sensor for Amazon cloud

	- Connect to WiFi
	- Connect to AWS IoT cloud
	- Get data from GY-302 (BH1750) ambient light sensor
	- Publish data into cloud
*/
#include <stdio.h>
#include <string.h>
#include "../build/config/sdkconfig.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"

//Project
#include "main.h"
#include "blink.h"
#include "wifi.h"
#include "i2c.h"
//-----------------------------------------------------------------------------
// FreeRTOS event group to to synchronize between tasks
EventGroupHandle_t events_group;

#include "aws_iot_mqtt_client_interface.h"

#include "aws_config.h"
//-----------------------------------------------------------------------------

// how often to publish data to the cloud, s
#define AWS_PUB_INTERVAL	300
#define MAX_JSON_SIZE		64
//-----------------------------------------------------------------------------
char AWS_host[255] = AWS_HOST;
uint32_t AWS_port = AWS_PORT;
//-----------------------------------------------------------------------------
TaskHandle_t aws_iot_task_handle = NULL;
//-----------------------------------------------------------------------------
void aws_notify_task_with_sensor_data(uint32_t raw_sensor_data)
{
	if (aws_iot_task_handle != NULL)
	{
		xTaskNotify(aws_iot_task_handle, raw_sensor_data, eSetValueWithOverwrite);
		ESP_LOGI(TAG_I2C, "AWS task notified");
	}
	else
		ESP_LOGI(TAG_I2C, "No AWS task to notify");
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

	wifi_start();

	i2c_start();

	aws_start();
}
//-----------------------------------------------------------------------------
