/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-05-09     shany       the first version
 */

#include <stdlib.h>
#include <rtthread.h>

#include "board.h"
#include "sensor.h"
#include "sensor_hc_sr04.h"

/* Modify this pin according to the actual wiring situation */
#define SR04_TRIG_PIN GET_PIN(C, 4)
#define SR04_ECHO_PIN GET_PIN(A, 4)

int sr04_read_distance_sample(void);
int rt_hw_sr04_port(void);


static void sr04_read_distance_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL) {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        rt_kprintf("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);

    while (1) {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1) {
            rt_kprintf("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else {
            rt_kprintf("distance:%3d.%dcm, timestamp:%5d\n", sensor_data.data.proximity / 10, sensor_data.data.proximity % 10, sensor_data.timestamp);
        }
        rt_thread_mdelay(2000);
    }
}

int sr04_read_distance_sample(void)
{
    rt_thread_t sr04_thread;

    sr04_thread = rt_thread_create("sr04",
                                   sr04_read_distance_entry,
                                   "pr_sr04",
                                   1024,
                                   RT_THREAD_PRIORITY_MAX / 2,
                                   20);
    if (sr04_thread != RT_NULL) {
        rt_thread_startup(sr04_thread);
    }

    return RT_EOK;
}
INIT_APP_EXPORT(sr04_read_distance_sample);

int rt_hw_sr04_port(void)
{
    struct rt_sensor_config cfg;
    rt_base_t pins[2] = {SR04_TRIG_PIN, SR04_ECHO_PIN};

    cfg.intf.dev_name = "timer15";
    cfg.intf.user_data = (void *)pins;
    rt_hw_sr04_init("sr04", &cfg);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_sr04_port);
