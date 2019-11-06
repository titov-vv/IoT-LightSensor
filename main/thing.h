/*
 * thing.h
 *
 *  Created on: Nov 1, 2019
 *      Author: vtitov
 */

#ifndef MAIN_THING_H_
#define MAIN_THING_H_
//-----------------------------------------------------------------------------
// THING SHADOW EXAMPLE
//{
//  "desired": {
//    "verbose": 0,
//    "high_margin": 100.0,
//    "low_margin": 25.0,
//    "interval": 60
//  },
//  "reported": {
//    "verbose": 0,
//    "high_margin": 100.0,
//    "low_margin": 25.0,
//    "interval": 60
//  }
//}
//-----------------------------------------------------------------------------
// Function to initiate AWS IOT task and handle MQTT exchange with the Cloud
// READY_BIT is used to track device readiness (IP is up and Time is set)
void aws_start();
//-----------------------------------------------------------------------------
// TO BE DELETED
void aws_notify_task_with_sensor_data(uint32_t raw_sensor_data);
//-----------------------------------------------------------------------------
#endif /* MAIN_THING_H_ */
