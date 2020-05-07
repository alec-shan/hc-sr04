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
#define SENSOR_DISTANCE_RANGE_MIN (4)

RT_WEAK void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t delta;

    us = us * (SysTick->LOAD / (1000000 / RT_TICK_PER_SECOND));
    delta = SysTick->VAL;

    while (delta - SysTick->VAL < us) continue;
}

#define HWTIMER_DEV_NAME "timer15"  // 定时器名称

static rt_device_t hw_dev = RT_NULL;  // 定时器设备句柄
static rt_hwtimerval_t hw_val;  // 定时器超时值
static rt_hwtimer_mode_t hw_mode;  // 定时器模式
static rt_uint8_t sr04_gg = 0;

static rt_err_t hwtimer_cb(rt_device_t dev, rt_size_t size)
{
    return 0;
}

static rt_err_t hwtimer_init(const char *name)
{
    rt_err_t ret = RT_EOK;

    /* 查找定时器设备 */
    hw_dev = rt_device_find(name);
    if (hw_dev == RT_NULL) {
        rt_kprintf("can't find %s device!\n", name);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("open %s device failed!\n", name);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, hwtimer_cb);

    /* 设置模式为周期性定时器 */
    hw_mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &hw_mode);
    if (ret != RT_EOK) {
        rt_kprintf("set mode failed! ret is: %d\n", ret);
        return ret;
    }

    return ret;
}

static rt_err_t hwtimer_start(void)
{
    /* 设置定时器超时值为1s并启动定时器 */
    hw_val.sec = 1;  // 秒
    hw_val.usec = 0;  // 微秒

    if (rt_device_write(hw_dev, 0, &hw_val, sizeof(hw_val)) != sizeof(hw_val))
    {
        rt_kprintf("set value failed\n");
        return RT_ERROR;
    }

    return RT_EOK;
}

static void hwtimer_stop(void)
{
    /* 读取定时器当前值 */
    rt_device_read(hw_dev, 0, &hw_val, sizeof(hw_val));
    rt_kprintf("Read: Sec = %d, Usec = %d\n", hw_val.sec, hw_val.usec);
    /* 关闭设备 */
    rt_device_close(hw_dev);
}

static void sr04_reset(sr04_device_t dev)
{
    rt_pin_mode(dev->trig_pin, PIN_MODE_OUTPUT);
    rt_pin_mode(dev->echo_pin, PIN_MODE_INPUT);

    hwtimer_init(dev->hwtimer);
}

void sr04_start(rt_base_t pin)
{
    ds18b20_reset(pin);
    ds18b20_connect(pin);
    ds18b20_write_byte(pin, 0xcc);  /* skip rom */
    ds18b20_write_byte(pin, 0x44);  /* convert */
}

uint8_t sr04_init(sr04_device_t dev)
{
    uint8_t ret = 0;

    ds18b20_reset(pin);
    ret = ds18b20_connect(pin);

    return ret;
}

int32_t sr04_get_distance(rt_base_t pin)
{
    uint8_t TL, TH;
    int32_t tem;

    ds18b20_start(pin);
    ds18b20_init(pin);
    ds18b20_write_byte(pin, 0xcc);
    ds18b20_write_byte(pin, 0xbe);
    TL = ds18b20_read_byte(pin);    /* LSB first */
    TH = ds18b20_read_byte(pin);
    if (TH > 7)
    {
        TH =~ TH;
        TL =~ TL;
        tem = TH;
        tem <<= 8;
        tem += TL;
        tem = (int32_t)(tem * 0.0625 * 10 + 0.5);
        return -tem;
    }
    else
    {
        tem = TH;
        tem <<= 8;
        tem += TL;
        tem = (int32_t)(tem * 0.0625 * 10 + 0.5);
        return tem;
    }
}

static rt_size_t _sr04_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    rt_int32_t distance_x10;
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        distance_x10 = sr04_get_distance((rt_base_t)sensor->config.intf.user_data);
        data->data.temp = distance_x10;
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_size_t sr04_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _sr04_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t sr04_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    sr04_fetch_data,
    sr04_control
};

int rt_hw_sr04_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_proximity = RT_NULL;

    if (!ds18b20_init((rt_base_t)cfg->intf.user_data))
    {
        /* temperature sensor register */
        sensor_proximity = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_proximity == RT_NULL)
            return -1;

        sensor_proximity->info.type       = RT_SENSOR_CLASS_PROXIMITY;
        sensor_proximity->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_proximity->info.model      = "sr04";
        sensor_proximity->info.unit       = RT_SENSOR_UNIT_CM;
        sensor_proximity->info.intf_type  = RT_SENSOR_INTF_ONEWIRE;
        sensor_proximity->info.range_max  = SENSOR_DISTANCE_RANGE_MAX;
        sensor_proximity->info.range_min  = SENSOR_DISTANCE_RANGE_MIN;
        sensor_proximity->info.period_min = 5;

        rt_memcpy(&sensor_proximity->config, cfg, sizeof(struct rt_sensor_config));
        sensor_proximity->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_proximity, name, RT_DEVICE_FLAG_RDONLY, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }

    }
    return RT_EOK;

__exit:
    if (sensor_proximity)
        rt_free(sensor_proximity);
    return -RT_ERROR;
}
