# HC-SR04 超声波测距模块

## 简介

HC-SR04 超声波测距模块可提供 2cm - 400cm 的非接触式距离感测功能，测距精度可以达到 ±3mm，模块包括超声波发射器、接收器与控制电路。HC-SR04 常用于智能小车的测距或其他一些距离感测项目中。
注意，本模块需要使用 5v 供电，厂家建议控制 IO 也使用 5v 电平输入，但实际使用 STM32 的 3.3V 电平输入也没有问题。
HC-SR04 软件包提供了使用超声波模块 `hc-sr04` 基本功能，并提供了温度补偿可选功能。本软件包已经对接到了 RT-Thread Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。

## 使用说明

### 依赖

- RT-Thread v4.0.0+
- Sensor 组件
- HWTIMER 设备：HC-SR04 使用硬件定时器计时，需要 HWTIMER 设备支持

### 使用软件包

hc-sr04 软件包初始化函数如下：

```
int rt_hw_sr04_init(const char *name, struct rt_sensor_config *cfg);
```

此函数需要用户自行调用，主要实现功能如下：

- 设备配置和初始化
- 注册相应的传感器设备，完成 hc-sr04 设备的注册

### 初始化示例

```c
#include "sensor_hc_sr04.h"
/* Modify this device name according to the actual using situation */
#define HWTIMER_DEV_NAME    "timer15"
/* Modify this pin according to the actual wiring situation */
#define SR04_TRIG_PIN       GET_PIN(C, 4)
#define SR04_ECHO_PIN       GET_PIN(A, 4)

int rt_hw_sr04_port(void)
{
    struct rt_sensor_config cfg;
    rt_base_t pins[2] = {SR04_TRIG_PIN, SR04_ECHO_PIN};

    cfg.intf.dev_name = HWTIMER_DEV_NAME;
    cfg.intf.user_data = (void *)pins;
    rt_hw_sr04_init("sr04", &cfg);

    return RT_EOK;
}
INIT_ENV_EXPORT(rt_hw_aht10_port);
```

## 联系方式

- 维护：alec-shan
- 主页：https://github.com/RT-Thread-packages/hc-sr04