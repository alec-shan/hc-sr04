/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-05-07     shany       the first version
 */
#ifndef __SENSOR_HC_SR04_H_
#define __SENSOR_HC_SR04_H_

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sensor.h"

struct sr04_device
{
    /* mount on a hwtimer to get accurate time */
    char *hwtimer;
    /* user configure */
    rt_base_t trig_pin;
    rt_base_t echo_pin;
    /* error code */
    rt_base_t err;
};
typedef struct sr04_device *sr04_device_t;

int rt_hw_sr04_init(const char *name, struct rt_sensor_config *cfg);

#endif /* __SENSOR_HC_SR04_H_ */
