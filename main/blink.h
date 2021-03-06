/*
 * blink.h
 *
 *  Created on: Oct 29, 2019
 *      Author: vtitov
 */

#ifndef MAIN_BLINK_H_
#define MAIN_BLINK_H_
//-----------------------------------------------------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//-----------------------------------------------------------------------------
#define BLINK_OFF		0x0
#define BLINK_ON		0xFFFFFFFF
#define BLINK_FAST		0x55555555
#define BLINK_SLOW		0x0000FFFF
//-----------------------------------------------------------------------------
// ESP32 internal LED GPIO
#define LED_PIN		GPIO_NUM_2
// low long is each blink interval, ms
#define BLINK_TICK	125
//-----------------------------------------------------------------------------
// Function to initiate blinking background task
void blink_start();
// Function to update blinking pattern
void set_blink_pattern(uint32_t pattern);
//-----------------------------------------------------------------------------
#endif /* MAIN_BLINK_H_ */
