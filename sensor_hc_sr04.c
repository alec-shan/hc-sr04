/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-05-07     shany       the first version
 */

#include "sensor_hc_sr04.h"
#include "sensor.h"
#include "board.h"
#include <rtdbg.h>

#define DBG_TAG "sensor.hc.sr04"
#define DBG_LVL DBG_INFO

#define SENSOR_DISTANCE_RANGE_MAX (400)
#define SENSOR_DISTANCE_RANGE_MIN (2)

static struct sr04_device *sr04_dev;

RT_WEAK void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t delta;

    us = us * (SysTick->LOAD / (1000000 / RT_TICK_PER_SECOND));
    delta = SysTick->VAL;

    while (delta - SysTick->VAL < us) continue;
}

static rt_err_t _sr04_hwtimer_cb(rt_device_t dev, rt_size_t size)
{
    sr04_dev->err = 0x88;

    return 0;
}

static rt_device_t _sr04_hwtimer_init(void)
{
    rt_device_t dev;
    rt_err_t ret = RT_EOK;

    dev = rt_device_find(sr04_dev->hwtimer);
    if (dev == RT_NULL) {
        rt_kprintf("can't find %s device!\n", sr04_dev->hwtimer);
        return RT_NULL;
    }

    ret = rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("open hwtimer device failed!\n");
        return RT_NULL;
    }

    rt_device_set_rx_indicate(dev, _sr04_hwtimer_cb);

    static rt_hwtimer_mode_t hw_mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &hw_mode);
    if (ret != RT_EOK) {
        rt_kprintf("set hwtimer mode failed!\n");
        return RT_NULL;
    }

    return dev;
}

static rt_err_t _sr04_hwtimer_start(rt_device_t dev)
{
    rt_hwtimerval_t hw_val;

    hw_val.sec = 1;
    hw_val.usec = 0;
    if (rt_device_write(dev, 0, &hw_val, sizeof(hw_val)) != sizeof(hw_val)) {
        rt_kprintf("set value failed\n");
        return RT_ERROR;
    }

    return RT_EOK;
}

static int32_t _sr04_hwtimer_stop(rt_device_t dev)
{
    rt_hwtimerval_t hw_val;

    rt_device_read(dev, 0, &hw_val, sizeof(hw_val));
    // rt_kprintf("read: sec = %d, usec = %d\n", hw_val.sec, hw_val.usec);

    rt_device_close(dev);

    return (int32_t)(hw_val.sec * 1000000 + hw_val.usec);
}

int32_t sr04_get_distance(void)
{
    int32_t duration = 0, distance = 0;
    rt_device_t dev;

    dev = _sr04_hwtimer_init();
    rt_pin_write(sr04_dev->trig_pin, PIN_LOW);
    rt_hw_us_delay(2);
    rt_pin_write(sr04_dev->trig_pin, PIN_HIGH);
    rt_hw_us_delay(10);
    rt_pin_write(sr04_dev->trig_pin, PIN_LOW);
    while ((rt_pin_read(sr04_dev->echo_pin) == PIN_LOW));
    _sr04_hwtimer_start(dev);
    while ((rt_pin_read(sr04_dev->echo_pin) == PIN_HIGH) && (sr04_dev->err != 0x88));
    duration = _sr04_hwtimer_stop(dev);

    distance = (int32_t)(duration * 340.0 / 2000.0 + 0.5);

    return distance;
}

static rt_size_t _sr04_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    rt_int32_t distance_x10;

    if (sensor->info.type == RT_SENSOR_CLASS_PROXIMITY) {
        distance_x10 = sr04_get_distance();
        data->data.proximity = distance_x10;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_size_t sr04_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING) {
        return _sr04_polling_get_data(sensor, buf);
    }
    else {
        return 0;
    }
}

static rt_err_t sr04_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t ret = RT_EOK;

    return ret;
}

static struct rt_sensor_ops sensor_ops =
{
    sr04_fetch_data,
    sr04_control
};

static sr04_device_t _sr04_init(struct rt_sensor_config *cfg)
{
    rt_base_t *pins;
    sr04_device_t dev;

    dev = rt_calloc(1, sizeof(struct sr04_device));
    if (dev == RT_NULL) {
        LOG_E("Can't allocate memory for sr04 device on '%s'.\n", cfg->intf.dev_name);
        return RT_NULL;
    }

    dev->hwtimer = cfg->intf.dev_name;
    rt_kprintf("hwtimer: %s\n", dev->hwtimer);

    pins = (rt_base_t *)cfg->intf.user_data;
    // rt_kprintf("trig: %d, echo: %d\n", pins[0], pins[1]);
    dev->trig_pin = pins[0];
    dev->echo_pin = pins[1];
    // rt_kprintf("trig: %d, echo: %d\n", dev->trig_pin, dev->echo_pin);
    rt_pin_mode(dev->trig_pin, PIN_MODE_OUTPUT);
    rt_pin_mode(dev->echo_pin, PIN_MODE_INPUT);

    return dev;
}

int rt_hw_sr04_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_sr04 = RT_NULL;

    sr04_dev = _sr04_init(cfg);
    /* sr04 sensor register */
    sensor_sr04 = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_sr04 == RT_NULL) {
        return -1;
    }

    sensor_sr04->info.type       = RT_SENSOR_CLASS_PROXIMITY;
    sensor_sr04->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor_sr04->info.model      = "sr04";
    sensor_sr04->info.unit       = RT_SENSOR_UNIT_CM;
    sensor_sr04->info.intf_type  = RT_SENSOR_INTF_ONEWIRE;
    sensor_sr04->info.range_max  = SENSOR_DISTANCE_RANGE_MAX;
    sensor_sr04->info.range_min  = SENSOR_DISTANCE_RANGE_MIN;
    sensor_sr04->info.period_min = 5;

    rt_memcpy(&sensor_sr04->config, cfg, sizeof(struct rt_sensor_config));
    sensor_sr04->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_sr04, name, RT_DEVICE_FLAG_RDONLY, RT_NULL);
    if (result != RT_EOK) {
        LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor_sr04)
        rt_free(sensor_sr04);
    return -RT_ERROR;
}
