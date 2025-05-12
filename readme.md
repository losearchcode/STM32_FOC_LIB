# [STM32_FOC_LIB](https://github.com/losearchcode/STM32_FOC_LIB)

本项目是基于STM32(STM32F401CC)的C语言版FOC库。

This project is based on the STM32 (STM32F401CC) C version of the FOC library.

## 项目内容介绍

### 硬件：

第一代单路电机驱动板（基于DRV8313驱动）原理图和PCB。

### 软件：

项目功能主体主要在工程空间的Middlewares文件夹中：

- EncoderEC11：
  - 通过轮询方式读取EC11编码器基本状态（左、右旋及按下），结合历史状态位判断当前的触发状态。（有点状态机的效果）
- LCD：
  - 移植例程7PIN1.3寸LCD屏幕（ST7789驱动），模拟SPI通讯。
  - GUI.c:自己写的简易UI界面。
- protocol：
  - 宏定义PID_ASSISTANT_EN = 1时，移植的是野火上位机PID调试助手，但没有尝试浮点数发送。
  - PID_ASSISTANT_EN = 0时，发送自定义串口数据，接收自定义命令，可以参考[loop222](https://blog.csdn.net/loop222)博主的simpleFOC发送命令。
- SimpleFoc：
  - BLDCMotor.c:主要文件，包含主要的FOC外调接口。
  - FOC_Para_All_t（C Struct）：主要结构体，包含所有所需参数。

## 功能介绍

| 功能         | 实现 |
| ------------ | ---- |
| 开环角度控制 | √   |
| 开环速度控制 | √   |
| 闭环角度控制 | √   |
| 闭环速度控制 | √   |
| 闭环力矩控制 | √   |

## 项目编译

本项目基于Keil平台，请在Keil平台编译。

This project is based on the Keil platform, please compile it on the Keil platform.

## 硬件接口

### 驱动：DRV8313

| Signal       | STM32 Pin Name | Function |
| ------------ | -------------- | -------- |
| Motor_PWM_A  | PA6            | TIM3_CH1 |
| Motor_PWM_B  | PA7            | TIM3_CH2 |
| Motor_PWM_C  | PB0            | TIM3_CH3 |
| Motor_Enable | PB2            | GPIO_OUT |

### 磁编码器：AS5600(I2C)

| Signal     | STM32 Pin Name | Function |
| ---------- | -------------- | -------- |
| AS5600_SDA | PB4            | I2C3_SDA |
| AS5600_SCL | PA8            | I2C3_SCL |

### LCD：ST7789驱动(模拟SPI)

| Signal  | STM32 Pin Name | Function |
| ------- | -------------- | -------- |
| LCD_SCL | PB9            | GPIO_OUT |
| LCD_SCL | PB8            | GPIO_OUT |
| LCD_RES | PB7            | GPIO_OUT |
| LCD_DC  | PB6            | GPIO_OUT |
| LCD_BLK | PB5            | GPIO_OUT |

其他硬件接口可以参考Single-way motor verification board文件夹下的原理图和PCB。

# 补充

V1.4之前是参考灯哥开源的DengFOC的自写库截至位置-速度闭环，之后版本接入simpleFOC，并完成有感电流环，基本完成有感专项。

Before V1.4, it was a reference to Dengfoc's open-source self-writing library cut-off position-speed closed loop, and then the version was connected to simpleFOC, and the induced current loop was completed, and the inducted special project was basically completed.

**最后一版main.c文件已丢失，具体内容可参考历史版本V1.9。**

**The last version of the main.c file has been lost, please refer to the previous version V1.9 for details.**

2025-5-11

双轮FOC案例基于另一个项目，接口与本项目不同，可以参考其中main文件思路进行拓展或移植。

The two-wheel FOC case is based on another project, the interface is different from this project, you can refer to the main file of which ideas for expansion or transplantation.

# 小请求

期待各位的Star，您的每一份Star都是我开源的动力。

Looking forward to all of you's Star, every one of your Star is the driving force for my open source.


## Star History

<a href="https://www.star-history.com/#losearchcode/STM32_FOC_LIB&Date">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=losearchcode/STM32_FOC_LIB&type=Date&theme=dark" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=losearchcode/STM32_FOC_LIB&type=Date" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=losearchcode/STM32_FOC_LIB&type=Date" />
 </picture>
</a>


# 参考资料

项目参考[野火FreeRTOS](https://doc.embedfire.com/rtos/freertos/zh/latest/index.html)、灯哥开源的[DengFOC](http://dengfoc.com/#/)、[SimpleFOC 中文](http://simplefoc.cn/#/)和[CSDN loop222博主的simpleFOC移植教程](https://blog.csdn.net/loop222/article/details/119220638)，在此由衷感谢。

Project references [Embedfire FreeRTOS](https://doc.embedfire.com/rtos/freertos/zh/latest/index.html), Dengge Open Source [DengFOC](http://dengfoc.com/#/), [SimpleFOC Chinese](http://simplefoc.cn/#/) and [CSDN loop222 blogger&#39;s simpleFOC porting tutorial](https://blog.csdn.net/loop222/article/details/119220638), I would like to thank you from the bottom of my heart.

# 最后的最后

所有资源源于网络，如有侵权，请联系作者

All resources come from the Internet, if there is any infringement, please contact the author.
