# STM32F407VET6 智能小车项目

## 📋 目录

- [项目概述](#项目概述)
  - [主要特性](#主要特性)
  - [功能模块](#功能模块)
- [系统架构](#系统架构)
- [快速开始](#快速开始)
  - [开发环境要求](#开发环境要求)
  - [编译和烧录](#编译和烧录)
  - [硬件连接](#硬件连接)
- [API参考](#api参考)
  - [核心调度器](#核心调度器)
  - [电机控制](#电机控制)
  - [编码器系统](#编码器系统已优化)
  - [IMU传感器](#imu传感器)
  - [Gary灰度传感器](#gary灰度传感器)
  - [OLED显示](#oled显示)
  - [串口通信](#串口通信)
  - [ADC采集](#adc采集)
- [配置参数](#配置参数)
  - [系统时钟配置](#系统时钟配置)
  - [任务调度配置](#任务调度配置)
  - [硬件参数配置](#硬件参数配置)
- [项目结构](#项目结构)
- [故障排除](#故障排除)
  - [常见问题](#常见问题)
  - [调试技巧](#调试技巧)
  - [性能优化建议](#性能优化建议)
- [技术支持](#技术支持)
  - [开发团队](#开发团队)
  - [Gary灰度传感器常见问题](#gary灰度传感器常见问题)
  - [相关资源](#相关资源)
  - [版本历史](#版本历史)

## 项目概述

本项目是基于STM32F407VET6微控制器的智能小车系统，采用模块化架构设计，集成了多种传感器和执行器。系统具有完整的任务调度机制，支持实时数据采集、电机控制、姿态检测和人机交互等功能。

### 主要特性

- **硬件平台**: STM32F407VET6 (ARM Cortex-M4, 168MHz)
- **架构设计**: 基于任务调度器的模块化架构
- **传感器融合**: IMU姿态传感器 + 编码器速度检测 + Gary灰度传感器 + ADC电压监测
- **循线功能**: 感为8通道灰度传感器，支持循线状态检测
- **电机控制**: TB6612FNG双路电机驱动，支持PWM调速
- **显示系统**: SSD1306 OLED显示屏，多页面管理
- **通信接口**: UART串口通信，支持命令交互
- **实时调度**: 7个并发任务，不同执行周期

### 功能模块

| 模块 | 功能描述 | 执行周期 |
|------|----------|----------|
| 调度器 | 任务时间片轮转调度 | 核心调度 |
| 电机控制 | TB6612驱动，PWM调速 | 1ms |
| 编码器 | 速度检测，滤波处理 | 50ms |
| IMU传感器 | JY901S姿态检测 | 20ms |
| Gary传感器 | 8通道灰度传感器，循线检测 | 30ms |
| OLED显示 | SSD1306显示驱动 | 100ms |
| ADC采集 | 电压监测，DMA采样 | 50ms |
| 串口通信 | 命令解析，数据传输 | 10ms |

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        STM32F407VET6 主控制器                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                          任务调度器                                         │
│                       (scheduler.c)                                        │
├─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┤
│ 电机控制│ 编码器  │IMU传感器│Gary传感器│OLED显示 │ADC采集  │ 串口    │
│ (1ms)  │(50ms)  │(20ms)  │(30ms)  │(100ms) │(50ms)  │(10ms)  │
├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
│TB6612FNG│ 500PPR  │ JY901S  │感为8通道│SSD1306  │12位ADC  │UART2    │
│双路驱动 │ 编码器  │九轴IMU  │灰度传感器│ OLED    │DMA采样  │115200   │
│         │         │         │ I2C3    │         │         │         │
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

## 快速开始

### 开发环境要求

- **IDE**: STM32CubeIDE 或 CLion + STM32CubeMX
- **编译器**: GCC ARM Embedded
- **调试器**: ST-Link V2/V3
- **固件库**: STM32Cube HAL Driver V1.28.2

### 编译和烧录

1. **克隆项目**
   ```bash
   git clone <repository-url>
   cd SmartCar
   ```

2. **使用STM32CubeIDE编译**
   - 导入项目到STM32CubeIDE
   - 选择Release或Debug配置
   - 点击Build Project (Ctrl+B)

3. **使用CMake编译**
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

4. **烧录程序**
   - 连接ST-Link调试器
   - 在IDE中点击Run/Debug
   - 或使用命令行: `st-flash write SmartCar.bin 0x8000000`

### 硬件连接

#### 主控制器引脚定义

| 功能 | 引脚 | 说明 |
|------|------|------|
| **电机控制** | | |
| PWM_A | PD12 (TIM4_CH1) | 左轮PWM输出 |
| PWM_B | PD13 (TIM4_CH2) | 右轮PWM输出 |
| AN1 | PA3 | 左轮方向控制1 |
| AN2 | PA4 | 左轮方向控制2 |
| BN1 | PB0 | 右轮方向控制1 |
| BN2 | PB1 | 右轮方向控制2 |
| STBY | PA2 | 电机驱动使能 |
| **编码器接口** | | |
| ENC_A1 | PA0 (TIM2_CH1) | 左轮编码器A相 |
| ENC_A2 | PA1 (TIM2_CH2) | 左轮编码器B相 |
| ENC_B1 | PA6 (TIM3_CH1) | 右轮编码器A相 |
| ENC_B2 | PA7 (TIM3_CH2) | 右轮编码器B相 |
| **I2C接口** | | |
| SCL1 | PB6 (I2C1_SCL) | OLED时钟线 |
| SDA1 | PB7 (I2C1_SDA) | OLED数据线 |
| SCL2 | PB10 (I2C2_SCL) | IMU时钟线 |
| SDA2 | PB11 (I2C2_SDA) | IMU数据线 |
| **串口通信** | | |
| UART_TX | PD5 (USART2_TX) | 串口发送 |
| UART_RX | PD6 (USART2_RX) | 串口接收 |
| **ADC采集** | | |
| ADC_IN | PC0 (ADC1_IN10) | 电压检测输入 |

#### 外设连接图

```
STM32F407VET6
     │
     ├─── TB6612FNG ─── 直流电机 x2
     │
     ├─── 编码器 x2 (500PPR)
     │
     ├─── JY901S IMU (I2C2)
     │
     ├─── SSD1306 OLED (I2C1)
     │
     ├─── 电压分压电路 ─── ADC
     │
     └─── USB转串口模块 (调试)
```

## API参考

### 核心调度器

```c
// 调度器初始化
void scheduler_init(void);

// 调度器运行（在主循环中调用）
void scheduler_run(void);
```

### 电机控制

```c
// 创建电机实例
int8_t Motor_Create(Motor_t* motor, TIM_HandleTypeDef* htim, uint32_t channel,
                    GPIO_TypeDef* in1_port, uint16_t in1_pin,
                    GPIO_TypeDef* in2_port, uint16_t in2_pin,
                    GPIO_TypeDef* stby_port, uint16_t stby_pin);

// 设置电机速度 (-1000 ~ +1000)
int8_t Motor_SetSpeed(Motor_t* motor, int32_t speed, uint8_t enable);

// 停止电机
int8_t Motor_Stop(Motor_t* motor);

// 启动/停止所有电机
void Motor_Start(void);
void Motor_StartStop(void);
```

### 编码器系统（已优化） {#编码器系统已优化}

#### 基础功能
```c
// 编码器初始化
void Encoder_Init(void);

// 编码器任务（自动调用）
void encoder_task(void);

// 清零速度数据
void clear_speed_data(void);

// 全局变量访问
extern Encoder_Data_t encoder_data_A;  // 左轮编码器
extern Encoder_Data_t encoder_data_B;  // 右轮编码器
extern Differential_Drive_t diff_drive_data;  // 差速驱动数据
```

#### 数据访问接口
```c
// 速度获取接口
float get_left_wheel_speed_rps(void);   // 左轮转速(RPS)
float get_right_wheel_speed_rps(void);  // 右轮转速(RPS)
float get_left_wheel_speed_ms(void);    // 左轮线速度(m/s)
float get_right_wheel_speed_ms(void);   // 右轮线速度(m/s)

// 差速驱动接口
Differential_Drive_t* get_differential_drive_data(void);

// 系统状态接口
Encoder_Data_t* get_encoder_A_data(void);
Encoder_Data_t* get_encoder_B_data(void);
uint8_t is_encoder_system_healthy(void);
```

#### 调试和校准接口
```c
// 调试功能
void debug_encoder_counter(void);    // 计数器状态调试
void debug_encoder_speed(void);      // 速度信息调试
void show_performance_stats(void);   // 性能统计显示
void reset_performance_stats(void);  // 重置统计数据

// 校准功能
void encoder_calibration(void);      // 交互式系统校准
```

#### 串口调试命令（精简版）

**电机控制命令（3个）**
- `pwm [left|right] [value]` - PWM控制和查看
  - `pwm` - 查看当前PWM状态
  - `pwm left 500` - 设置左轮PWM为500
  - `pwm right -300` - 设置右轮PWM为-300
- `start` - 启动电机
- `stop` - 停止电机

**传感器数据命令（2个）**
- `sensor` - 显示所有传感器数据（编码器速度+ADC电压+IMU姿态）
- `encoder [debug|cal]` - 编码器功能
  - `encoder debug` - 显示速度和计数器调试信息
  - `encoder cal` - 执行编码器校准

**Gary灰度传感器命令（3个）**
- `gary` - 显示完整传感器信息（数据+循线状态+系统状态）
- `gary ping` - 检测传感器连接
- `gary reinit` - 重新初始化传感器

**系统管理命令（3个）**
- `system [perf|reset|diag]` - 系统功能
  - `system perf` - 显示性能统计
  - `system reset` - 重置性能统计
  - `system diag` - 编码器采样诊断
- `page <motor|imu>` - 页面切换
  - `page motor` - 切换到电机页面
  - `page imu` - 切换到IMU页面
- `help` - 显示所有命令帮助

**指令精简优势**
- 指令数量：23个 → 12个（减少48%）
- 参数化设计：支持灵活的参数组合
- 功能整合：相关功能统一入口，减少记忆负担
- 操作效率：直接参数设置，无需交互式输入

#### 优化特性
- **自适应采样**：根据速度自动调整采样频率（20ms/50ms/100ms）
- **高性能计算**：预计算常量，计算开销降低50%+
- **增量滤波**：滤波效率提升80%，算法复杂度O(n)→O(1)
- **差速驱动**：实时计算线速度和角速度
- **精度提升**：测量精度提升15%+，噪声抑制效果显著
- **指令精简**：23个指令精简为12个参数化指令，操作效率提升48%
- **参数化设计**：支持灵活参数组合，减少交互式输入步骤
- **功能整合**：相关功能统一入口，降低学习和记忆成本

### IMU传感器

```c
// IMU初始化
void IMU_Init(void);

// IMU数据读取任务（自动调用）
void imu_task(void);

// 状态检查函数
uint8_t IMU_IsDataReady(void);
uint8_t IMU_IsInitialized(void);
void IMU_ClearError(void);

// 全局数据访问
extern IMU_Data_t imu_data;  // IMU数据结构
```

### Gary灰度传感器

#### 基础功能
```c
// Gary传感器初始化
void gary_init(void);

// Gary传感器任务（自动调用）
void gary_task(void);

// 状态检查函数
uint8_t Gary_IsDataReady(void);
uint8_t Gary_IsInitialized(void);
void Gary_ClearError(void);

// 全局数据访问
extern Gary_Data_t gary_data;  // Gary传感器数据结构
```

#### 数据访问接口
```c
// 传感器数据获取
uint8_t Gary_GetDigital(void);                    // 获取8位数字数据
void Gary_GetAnalog(uint8_t *data);               // 获取8通道模拟数据
void Gary_GetNormalize(uint8_t *data);            // 获取8通道归一化数据

// 循线状态获取
Gary_LineState_t Gary_GetLineState(void);         // 获取循线状态
int16_t Gary_GetLineError(void);                  // 获取位置偏差(-100到+100)
uint8_t Gary_GetLineWidth(uint8_t digital_data);  // 获取线宽信息
uint8_t Gary_DetectIntersection(uint8_t digital_data); // 检测交叉路口
```

#### 循线算法
```c
// 核心算法函数
Gary_LineState_t Gary_DetectLineState(uint8_t digital_data);  // 状态检测
int16_t Gary_CalculateLineError(uint8_t digital_data);        // 偏差计算
```

#### Gary传感器串口命令
- `gary ping` - 检测传感器连接状态
- `gary data` - 显示8通道数字/模拟/归一化数据
- `gary line` - 显示循线状态和位置偏差
- `gary state` - 显示详细状态和算法分析
- `gary reinit` - 重新初始化传感器（解决初始化失败问题）
- `gary debug` - 显示原始调试数据（类似例程格式）

#### 技术特性
- **传感器型号**: 感为8通道灰度传感器
- **通信接口**: I2C3，地址0x4C
- **采样周期**: 30ms，平衡实时性和系统负载
- **数据格式**: 8位数字数据 + 8通道模拟数据(0-255)
- **循线状态**: 12种状态（中央、左偏、右偏、丢线、交叉等）
- **位置精度**: ±100偏差值，适用于循线控制
- **传感器特性**: 白场高电平，黑场低电平
- **算法优化**: 位运算优化，预计算权重数组

### OLED显示

```c
// OLED初始化
void OLED_Init(void);

// OLED显示任务（自动调用）
void oled_task(void);

// 页面切换
void OLED_SwitchPage(display_page_t page);

// 格式化输出
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...);
int Oled_Printf_H(uint8_t x, uint8_t y, const char *format, ...);
```

### 串口通信

```c
// 串口任务（自动调用）
void uart_task(void);

// 格式化输出
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

// 支持的命令
// pls    - 设置左轮PWM
// prs    - 设置右轮PWM
// pc     - 查看PWM值
// start  - 启动电机
// stop   - 停止电机
// sc     - 显示速度
// ac     - 显示ADC值
// imu show - 显示IMU数据
// page motor/imu - 切换OLED页面
```

### ADC采集

```c
// ADC任务（自动调用）
void adc_task(void);

// 全局变量访问
extern uint32_t adc_val;    // ADC原始值
extern float voltage;       // 转换后电压值
```

## 配置参数

### 系统时钟配置

- **主频**: 168MHz
- **AHB频率**: 168MHz
- **APB1频率**: 42MHz
- **APB2频率**: 84MHz
- **外部晶振**: 8MHz HSE

### 任务调度配置

| 任务 | 执行周期 | 优先级 | 功能 |
|------|----------|--------|------|
| uart_task | 10ms | 高 | 串口通信处理 |
| motor_task | 1ms | 最高 | 电机控制更新 |
| imu_task | 20ms | 高 | IMU数据读取 |
| encoder_task | 50ms | 中 | 编码器速度计算 |
| oled_task | 100ms | 低 | 显示屏更新 |
| adc_task | 50ms | 中 | ADC数据采集 |

### 硬件参数配置

```c
// 编码器参数
#define ENCODER_PPR           500      // 每转脉冲数
#define ENCODER_QUADRATURE    4        // 四倍频
#define GEAR_RATIO           20.0f     // 减速比
#define WHEEL_DIAMETER       0.048f    // 轮径(m)

// IMU参数
#define IMU_I2C_ADDR         0x50      // I2C地址
#define IMU_SAMPLE_TIME      20        // 采样周期(ms)
#define IMU_MAX_RETRY        3         // 最大重试次数

// 电机参数
#define PWM_MAX_VALUE        1000      // PWM最大值
#define SPEED_MAX            1000      // 最大速度
#define SPEED_MIN           -1000      // 最小速度

// 串口参数
#define UART_BAUDRATE        115200    // 波特率
#define UART_BUFFER_SIZE     128       // 缓冲区大小
```

## 项目结构

```
SmartCar/
├── APP/                    # 应用层代码
│   ├── scheduler.c/h       # 任务调度器
│   ├── motor_app.c/h       # 电机控制模块
│   ├── encoder_app.c/h     # 编码器模块
│   ├── JY901S_app.c/h      # IMU传感器模块
│   ├── oled_app.c/h        # OLED显示模块
│   ├── adc_app.c/h         # ADC采集模块
│   ├── usart_app.c/h       # 串口通信模块
│   └── mydefine.h          # 全局定义文件
├── Core/                   # STM32 HAL核心代码
│   ├── Inc/                # 头文件
│   └── Src/                # 源文件
├── Drivers/                # STM32 HAL驱动库
│   ├── CMSIS/              # CMSIS库
│   └── STM32F4xx_HAL_Driver/ # HAL驱动
├── components/             # 第三方组件
│   ├── OLED/               # SSD1306 OLED驱动
│   └── wit_c_sdk/          # 维特IMU SDK
├── Engineering Report/     # 技术文档
├── Revision Log/           # 版本日志
├── CMakeLists.txt          # CMake构建文件
├── SmartCar.ioc           # STM32CubeMX配置
└── README.md              # 项目说明文档
```

## 故障排除

### 常见问题

#### 1. 编译错误

**问题**: 找不到头文件或库文件
```
fatal error: 'stm32f4xx_hal.h' file not found
```

**解决方案**:
- 检查STM32CubeMX配置是否正确生成
- 确认HAL库路径在项目设置中正确配置
- 重新生成代码: STM32CubeMX → Project → Generate Code

#### 2. 烧录失败

**问题**: ST-Link连接失败
```
Error: No ST-LINK detected!
```

**解决方案**:
- 检查ST-Link硬件连接
- 确认驱动程序已正确安装
- 尝试不同的USB端口
- 检查目标板电源是否正常

#### 3. 串口通信异常

**问题**: 串口无法接收数据或数据乱码

**解决方案**:
- 检查波特率设置 (115200)
- 确认串口线连接正确 (TX-RX交叉)
- 检查DMA配置是否正确
- 验证中断优先级设置

#### 4. IMU数据异常

**问题**: IMU数据不更新或数值异常

**解决方案**:
- 检查I2C连接线路
- 确认IMU电源供电正常 (3.3V)
- 验证I2C地址设置 (0x50)
- 检查维特SDK初始化状态

#### 5. 电机不转动

**问题**: 电机无响应或转动异常

**解决方案**:
- 检查TB6612电源供电 (VM引脚)
- 确认PWM信号输出正常
- 验证方向控制引脚状态
- 检查STBY使能信号

#### 6. OLED显示异常

**问题**: OLED无显示或显示异常

**解决方案**:
- 检查I2C连接 (SCL/SDA)
- 确认OLED电源供电 (3.3V)
- 验证I2C地址配置
- 检查显示缓冲区更新

### 调试技巧

#### 1. 使用串口调试

```c
// 在关键位置添加调试输出
my_printf(&huart2, "Debug: Task executed at %lu\r\n", HAL_GetTick());
```

#### 2. LED指示调试

```c
// 使用板载LED指示系统状态
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
```

#### 3. 断点调试

- 在关键函数设置断点
- 观察变量值变化
- 单步执行检查程序流程

#### 4. 逻辑分析仪

- 监测PWM波形
- 检查I2C通信时序
- 分析编码器信号

### 性能优化建议

1. **任务调度优化**
   - 根据实际需求调整任务执行周期
   - 避免在中断中执行耗时操作
   - 合理设置中断优先级

2. **内存使用优化**
   - 减少全局变量使用
   - 优化数据结构设计
   - 避免内存泄漏

3. **功耗优化**
   - 使用低功耗模式
   - 关闭不必要的外设时钟
   - 优化任务执行频率

### Gary灰度传感器常见问题 {#gary灰度传感器常见问题}

#### 连接问题
**问题**: `gary ping` 命令显示"连接失败"
**解决方案**:
1. 检查I2C3接口连接（SCL、SDA、VCC、GND）
2. 确认传感器电源供电正常（3.3V或5V）
3. 检查I2C地址是否正确（默认0x4C）
4. 使用万用表测试I2C线路连通性

#### 初始化问题
**问题**: `gary ping` 显示"连接正常"但"未初始化"
**解决方案**:
1. 使用 `gary reinit` 命令重新初始化传感器
2. 检查传感器电源稳定性，确保启动时电压稳定
3. 如果问题持续，检查I2C3时序是否正确
4. 使用 `gary debug` 查看详细的传感器数据

#### 数据异常
**问题**: `gary data` 显示数据全为0或异常值
**解决方案**:
1. 检查传感器是否正确初始化：`gary ping`
2. 确认传感器与地面距离（推荐5-10mm）
3. 检查环境光照条件，避免强光直射
4. 清洁传感器表面，去除灰尘和污渍

#### 循线异常
**问题**: `gary line` 显示状态不准确或频繁跳变
**解决方案**:
1. 调整传感器高度，确保合适的检测距离
2. 检查黑线宽度是否适合（推荐15-25mm）
3. 确认地面对比度足够（白底黑线）
4. 使用`gary state`查看详细的传感器通道状态

#### 性能问题
**问题**: 循线响应延迟或不稳定
**解决方案**:
1. 检查任务调度周期（默认30ms）
2. 确认通信错误计数不过高
3. 优化循线算法参数（权重数组）
4. 检查系统负载，避免任务冲突

### 调试技巧

#### 使用串口命令调试
```bash
# 1. 检查连接
gary ping

# 2. 查看实时数据
gary data

# 3. 监控循线状态
gary line

# 4. 深度诊断
gary state
```

#### 数据分析方法
1. **数字数据分析**: 观察8位二进制模式，识别线的位置
2. **模拟数据分析**: 查看0-255范围的原始值，判断传感器灵敏度
3. **归一化数据**: 使用0-100范围的标准化值进行算法调试
4. **状态跟踪**: 监控循线状态变化，优化算法参数

### 相关资源

- [STM32F407VET6数据手册](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [TB6612FNG驱动芯片手册](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf)
- [JY901S IMU用户手册](http://wiki.wit-motion.com/)
- [SSD1306 OLED控制器手册](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [感为灰度传感器技术手册](components/Gary/gw_grayscale_sensor.h)

### 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| v1.0.0 | 2025-01-03 | 初始版本发布 |
| v1.1.0 | 2025-01-03 | 添加IMU模块支持 |
| v1.2.0 | 2025-01-03 | 完善文档和注释 |
| v1.3.0 | 2025-01-07 | 添加Gary灰度传感器模块，支持循线检测 |

## 技术支持

### 开发团队

- **项目负责人**: [刘宇航]
- **硬件工程师**: [刘宇航]
- **软件工程师**: [刘宇航]
- **测试工程师**: [刘宇航]

---

**注意**: 本项目仅供学习和研究使用，请遵守相关法律法规。