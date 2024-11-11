# BSP README 模板

## 简介

本文档为 xxx 开发板的 BSP (板级支持包) 说明。

主要内容如下：

- 开发板资源介绍
- BSP 快速上手
- 进阶使用方法

通过阅读快速上手章节开发者可以快速地上手该 BSP，将 RT-Thread 运行在开发板上。在进阶使用指南章节，将会介绍更多高级功能，帮助开发者利用 RT-Thread 驱动更多板载资源。

## 开发板介绍

开发板外观如下图所示：

![buy.png](C:\Users\lenovo\Downloads\buy.png)

该开发板常用 **板载资源** 如下：

- MCU：STM32H723，1MByte Flash、564 KBytes RAM、550 MHz CPU、ART 加速器
- 外部 QSPI Flash：W25Q64JV，8MBytes
- 外设
  - 板载指示灯
  - 蜂鸣器
  - 按键
  - 板载IMU
- 接口：
  - CAN接口
  - RS485接口
  - USB接口
  - SBUS接口
  - PWM接口
  - 串口
  - LCD拓展接口
  - 摄像头接口
  - 用户拓展IO
  - 可控电源接口
- 调试接口，SWD (Serial Wire Debug) 接口

开发板更多详细信息请参考达妙淘宝店铺 [达妙STM32开发板H723 DM-MC02机器人轮足控制板机械臂板载BMI088](https://item.taobao.com/item.htm?abbucket=18&id=814954787248&ns=1&priceTId=213e385b17312947968688397e8587&skuId=5681498675796&spm=a21n57.1.item.2.6adb523cwWzIUq&utparam=%7B%22aplus_abtest%22%3A%22dfae81afa534d376bf2ca37e9cc3e414%22%7D&xxc=taobaoSearch)。

## 外设支持

本 BSP 目前对外设的支持情况如下：

| **板载外设**   | **支持情况** | **备注** |
|:---------- |:--------:|:------ |
| CAN        | 暂不支持     |        |
| RS485      | 暂不支持     |        |
| USB        | 暂不支持     |        |
| SBUS       | 暂不支持     |        |
| PWM        | 暂不支持     |        |
| 串口         | 暂不支持     |        |
| 板载指示灯      | 暂不支持     |        |
| 蜂鸣器        | 暂不支持     |        |
| 按键         | 暂不支持     |        |
| 板载IMU      | 暂不支持     |        |
| LCD        | 暂不支持     |        |
| QSPI Flash | 暂不支持     |        |
| 摄像头        | 暂不支持     |        |
| GPIO       | 暂不支持     |        |

## 使用说明

使用说明分为如下两个章节：

- 快速上手
  
    本章节是为刚接触 RT-Thread 的新手准备的使用说明，遵循简单的步骤即可将 RT-Thread 操作系统运行在该开发板上，看到实验效果 。

- 进阶使用
  
    本章节是为需要在 RT-Thread 操作系统上使用更多开发板资源的开发者准备的。通过使用 ENV 工具对 BSP 进行配置，可以开启更多板载资源，实现更多高级功能。

### 快速上手

本 BSP 为开发者提供 GCC 工程，并且支持 MDK4，MDK5，IAR等 开发环境。下面以 GCC , Clion, OpenOCD开发环境为例，介绍如何将系统运行起来。

#### 硬件连接

使用数据线连接开发板到 PC，打开电源开关。

#### 编译下载

工程根目录下右键通过Clion打开，编译并下载程序到开发板（OpenOCD不支持J-LINK的原生驱动，需要使用工具将jlink的原生驱动转为winusb驱动才可识别。详情参考[解决openocd无法识别jlink的问题LIBUSB_ERROR_NOT_SUPPORTED_Qrpucp-开放原子开发者工作坊](https://openatomworkshop.csdn.net/664edd5cb12a9d168eb6f757.html)，或者使用J-Flash、Ozone等工具）

> 工程默认配置使用 J-LINK 仿真器下载程序，在通过 J-LINK 连接开发板的基础上，使用J-Flash或Ozone下载至开发板

#### 运行结果

下载程序成功之后，系统会自动运行，指示灯闪烁。

连接开发板对应串口到 PC , 在终端工具里打开相应的串口（115200-8-1-N），复位设备后，可以看到 RT-Thread 的输出信息:

```bash
 \ | /
- RT -     Thread Operating System
 / | \     4.0.1 build Mar 5 2019
 2006 - 2019 Copyright by rt-thread team
msh >
```

### 进阶使用

此 BSP 默认只开启了 GPIO 和 串口1 的功能，如果需使用 Flash，通讯接口 等更多高级功能，需要利用 ENV 工具对BSP 进行配置，步骤如下：

1. 在 bsp 下打开 env 工具。

2. 输入`menuconfig`命令配置工程，配置好之后保存退出。

3. 输入`pkgs --update`命令更新软件包。

4. 输入`scons --target=cmake` 命令重新生成工程。

本章节更多详细的介绍请参考 [STM32 系列 BSP 外设驱动使用教程](../docs/STM32系列BSP外设驱动使用教程.md)。

## 注意事项

- xxx

## 联系人信息

维护人:

- [Scienheim · GitHub](https://github.com/Scienheim), 邮箱：<telselfishness@163.com>