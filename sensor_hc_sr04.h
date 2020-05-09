/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-05-07     shany       the first version
 */
#ifndef APPLICATIONS_SENSOR_HC_SR04_H_
#define APPLICATIONS_SENSOR_HC_SR04_H_

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sensor.h"

struct sr04_device
{
    /* mount on a hwtimer to get accurate time */
    char *hwtimer;

    rt_base_t trig_pin;
    rt_base_t echo_pin;

    rt_base_t err;

    rt_mutex_t lock;
};
typedef struct sr04_device *sr04_device_t;

//int32_t sr04_get_distance(struct sr04_device *dev);
int rt_hw_sr04_init(const char *name, struct rt_sensor_config *cfg);

#endif /* APPLICATIONS_SENSOR_HC_SR04_H_ */
