/*
 * i2c.c
 *
 *  Created on: Nov 6, 2019
 *      Author: vtitov
 */
//-----------------------------------------------------------------------------
#include "main.h"
#include "i2c.h"

#include "esp_log.h"
#include "driver/i2c.h"
//-----------------------------------------------------------------------------
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

			aws_notify_task_with_sensor_data(raw_sensor_data);
        }

		vTaskDelay(I2C_POLL_INTERVAL / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}
//-----------------------------------------------------------------------------
void i2c_start()
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
