/**
 * @file mydefine.h
 * @brief 全局头文件定义 - 统一包含所有系统头文件和应用模块
 */

// STM32 HAL库头文件
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

// 标准C库头文件
#include "math.h"
#include "stdarg.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// 应用模块头文件
#include "adc_app.h"
#include "encoder_app.h"
#include "JY901S_app.h"
#include "motor_app.h"
#include "oled_app.h"
#include "scheduler.h"
#include "usart_app.h"
#include "gary_app.h"
#include "pid_control.h"

// 第三方组件头文件
#include "ssd1306.h"
#include "ssd1306_conf_template.h"
#include "ssd1306_fonts.h"

#include "gw_grayscale_sensor.h"
#include "hardware_iic.h"

#include "pid.h"

// ==================== 编码器系统配置区块 ====================
// 编码器基础参数
#define ENCODER_PPR           500      // 编码器每转脉冲数
#define ENCODER_QUADRATURE    4        // 四倍频系数
#define ENCODER_SAMPLE_TIME   50       // 采样时间间隔(ms)
#define ENCODER_FILTER_SIZE   5        // 滤波窗口大小

// 机械参数
#define GEAR_RATIO           20.0f     // 减速比 (编码器转速 / 电机转速)
#define WHEEL_DIAMETER       0.048f    // 轮子直径(m) - 48mm
#define WHEEL_BASE           0.15f     // 轮距(m) - 150mm (左右轮中心距离)

// 自适应采样配置
#define ENCODER_ADAPTIVE_ENABLE       1           // 启用自适应采样
#define ENCODER_HIGH_SPEED_SAMPLE     20          // 高速采样时间(ms)
#define ENCODER_LOW_SPEED_SAMPLE      100         // 低速采样时间(ms)
#define ENCODER_SPEED_THRESHOLD_H     6.0f        // 高速阈值(RPS) - 约35%PWM以上
#define ENCODER_SPEED_THRESHOLD_L     1.0f        // 低速阈值(RPS) - 约3%PWM以下

// 预计算优化常量（编译时确定，减少运行时除法）
#define TOTAL_PPR            (ENCODER_PPR * ENCODER_QUADRATURE)    // 总脉冲数 = 2000
#define WHEEL_CIRCUMFERENCE  (3.14159f * WHEEL_DIAMETER)          // 轮子周长 = 0.151m
#define INV_TOTAL_PPR        (1.0f / TOTAL_PPR)                   // 总脉冲数倒数
#define INV_GEAR_RATIO       (1.0f / GEAR_RATIO)                  // 减速比倒数
#define MS_TO_S_FACTOR       1000.0f                              // 毫秒转秒系数
#define SPEED_CALC_FACTOR    (INV_TOTAL_PPR * INV_GEAR_RATIO * MS_TO_S_FACTOR) // 速度计算组合常量

// ==================== Gary灰度传感器配置区块 ====================
// Gary传感器基础参数 (使用I2C3接口)
#define GARY_I2C_ADDR         0x4C         // 感为8通道灰度传感器I2C地址
#define GARY_SAMPLE_TIME      30           // 采样时间间隔(ms)
#define GARY_COMM_TIMEOUT     100          // I2C通信超时时间(ms)
#define GARY_MAX_RETRY        3            // 最大重试次数
#define GARY_FILTER_SIZE      3            // 滤波窗口大小

// Gary传感器特性参数 (白场高电平，黑场低电平)
#define GARY_WHITE_LEVEL      1            // 白场电平状态
#define GARY_BLACK_LEVEL      0            // 黑场电平状态
#define GARY_ANALOG_MAX       255          // 模拟值最大值
#define GARY_ANALOG_MIN       0            // 模拟值最小值

// 循线检测参数
#define GARY_LINE_THRESHOLD   128          // 数字化阈值
#define GARY_CENTER_CHANNELS  0x18         // 中央通道掩码 (00011000)
#define GARY_ALL_CHANNELS     0xFF         // 全通道掩码

// 循线位置权重数组 (用于位置偏差计算)
#define GARY_LINE_WEIGHTS     {-4.0f, -3.0f, -2.0f, -1.0f, 1.0f, 2.0f, 3.0f, 4.0f}  // 8通道位置权重

// PID控制接口参数 (基于平均值算法，范围±4.0)
#define GARY_ERROR_MAX        4.0f         // 最大偏差值
#define GARY_ERROR_MIN        -4.0f        // 最小偏差值

// 循线状态判定阈值参数 (基于平均值算法{-4.0,-3.0,-2.0,-1.0,1.0,2.0,3.0,4.0}优化)
#define GARY_CENTER_THRESHOLD     0.5f     // 中央状态阈值 (±0.5)
#define GARY_SLIGHT_THRESHOLD     1.5f     // 轻微偏移阈值 (0.5-1.5)
#define GARY_MODERATE_THRESHOLD   2.5f     // 中度偏移阈值 (1.5-2.5)
#define GARY_SHARP_THRESHOLD      3.5f     // 急剧偏移阈值 (>2.5)



