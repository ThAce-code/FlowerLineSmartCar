# OLED显示系统技术要点

## 概述

本文档详细分析STM32F407VET6智能小车项目中的OLED显示系统实现。系统采用SSD1306控制器的128×64像素OLED显示屏，通过I2C接口与STM32通信，实现多页面显示管理、字体渲染和格式化输出功能。支持电机状态和IMU姿态数据的实时显示。

## 硬件架构

### SSD1306 OLED控制器特性

SSD1306是一款专用于小尺寸OLED显示屏的控制器芯片，具有以下特点：

- **显示分辨率**: 128×64像素 (单色)
- **显示技术**: 有机发光二极管 (OLED)
- **通信接口**: I2C / SPI (本项目使用I2C)
- **工作电压**: 3.3V逻辑电平，内置升压电路
- **显示缓冲**: 1KB内部GDDRAM (128×64÷8 = 1024字节)
- **页面模式**: 8页×128列的页面寻址模式

### 硬件连接配置

```c
// I2C接口配置
#define SSD1306_I2C_PORT        hi2c1      // I2C1接口
#define SSD1306_I2C_ADDR        (0x3C << 1)  // I2C地址 0x78

// 显示屏规格
#define SSD1306_WIDTH           128        // 显示宽度(像素)
#define SSD1306_HEIGHT          64         // 显示高度(像素)
#define SSD1306_BUFFER_SIZE     (SSD1306_WIDTH * SSD1306_HEIGHT / 8)  // 1024字节
```

### 引脚连接表

| 功能 | OLED引脚 | STM32引脚 | 说明 |
|------|----------|-----------|------|
| 电源 | VCC | 3.3V | 逻辑电源 |
| 地线 | GND | GND | 公共地 |
| 时钟 | SCL | PB6 (I2C1_SCL) | I2C时钟线 |
| 数据 | SDA | PB7 (I2C1_SDA) | I2C数据线 |

## SSD1306控制原理

### 显示内存结构

SSD1306采用页面寻址模式，将128×64像素的显示区域分为8个页面：

```
页面结构 (每页8行像素):
Page 0: Row 0-7     ┌─────────────────────────────┐
Page 1: Row 8-15    │  128 × 8 像素               │
Page 2: Row 16-23   │  (1页 = 128字节)            │
Page 3: Row 24-31   │                             │
Page 4: Row 32-39   │  总共8页                    │
Page 5: Row 40-47   │  8 × 128 = 1024字节         │
Page 6: Row 48-55   │                             │
Page 7: Row 56-63   └─────────────────────────────┘
```

### 像素寻址方式

```c
// 像素坐标到缓冲区地址的转换
uint16_t pixel_to_buffer_index(uint8_t x, uint8_t y) {
    uint8_t page = y / 8;           // 计算页面号 (0-7)
    uint8_t bit_position = y % 8;   // 计算页面内位位置 (0-7)
    uint16_t buffer_index = page * SSD1306_WIDTH + x;  // 缓冲区索引
    
    return buffer_index;
}

// 设置像素的位操作
void set_pixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
    uint16_t index = pixel_to_buffer_index(x, y);
    uint8_t bit_mask = 1 << (y % 8);
    
    if (color == White) {
        SSD1306_Buffer[index] |= bit_mask;   // 设置位
    } else {
        SSD1306_Buffer[index] &= ~bit_mask;  // 清除位
    }
}
```

## 数据结构设计

### SSD1306设备结构

```c
typedef struct {
    uint16_t CurrentX;      // 当前光标X坐标
    uint16_t CurrentY;      // 当前光标Y坐标
    uint8_t Initialized;    // 初始化标志
} SSD1306_t;

// 全局设备实例
static SSD1306_t SSD1306;
```

### 显示缓冲区

```c
// 静态显示缓冲区 (1024字节)
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

// 颜色枚举
typedef enum {
    Black = 0x00,   // 像素关闭 (黑色)
    White = 0x01    // 像素点亮 (白色)
} SSD1306_COLOR;
```

### 页面管理系统

```c
// 显示页面枚举
typedef enum {
    PAGE_MOTOR = 0,    // 电机控制页面
    PAGE_IMU = 1       // IMU姿态页面
} display_page_t;

// 全局页面状态
extern display_page_t current_page;
```

## I2C通信协议

### 通信时序

SSD1306的I2C通信分为命令和数据两种模式：

```c
// 发送命令 (Control Byte = 0x00)
void ssd1306_WriteCommand(uint8_t byte) {
    HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 
                      0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

// 发送数据 (Control Byte = 0x40)
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 
                      0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}
```

### 初始化序列

```c
void ssd1306_Init(void) {
    // 1. 硬件复位
    ssd1306_Reset();
    HAL_Delay(100);
    
    // 2. 关闭显示
    ssd1306_SetDisplayOn(0);
    
    // 3. 设置内存寻址模式
    ssd1306_WriteCommand(0x20);  // Set Memory Addressing Mode
    ssd1306_WriteCommand(0x00);  // Horizontal Addressing Mode
    
    // 4. 设置页面起始地址
    ssd1306_WriteCommand(0xB0);  // Set Page Start Address (0-7)
    
    // 5. 设置扫描方向
    ssd1306_WriteCommand(0xC8);  // Set COM Output Scan Direction
    
    // 6. 设置列地址
    ssd1306_WriteCommand(0x00);  // Set low column address
    ssd1306_WriteCommand(0x10);  // Set high column address
    
    // 7. 设置起始行
    ssd1306_WriteCommand(0x40);  // Set start line address
    
    // 8. 设置对比度
    ssd1306_SetContrast(0xFF);   // Maximum contrast
    
    // 9. 设置段重映射
    ssd1306_WriteCommand(0xA1);  // Set segment re-map 0 to 127
    
    // 10. 设置正常显示
    ssd1306_WriteCommand(0xA6);  // Set normal color
    
    // 11. 设置复用比
    ssd1306_WriteCommand(0xA8);  // Set multiplex ratio
    ssd1306_WriteCommand(0x3F);  // 1/64 duty (64 lines)
    
    // 12. 设置显示偏移
    ssd1306_WriteCommand(0xD3);  // Set display offset
    ssd1306_WriteCommand(0x00);  // No offset
    
    // 13. 设置时钟分频
    ssd1306_WriteCommand(0xD5);  // Set display clock divide ratio
    ssd1306_WriteCommand(0xF0);  // Set divide ratio
    
    // 14. 设置预充电周期
    ssd1306_WriteCommand(0xD9);  // Set pre-charge period
    ssd1306_WriteCommand(0x22);  // Pre-charge period
    
    // 15. 设置COM引脚配置
    ssd1306_WriteCommand(0xDA);  // Set com pins hardware configuration
    ssd1306_WriteCommand(0x12);  // Alternative COM pin config
    
    // 16. 设置VCOMH电压
    ssd1306_WriteCommand(0xDB);  // Set vcomh
    ssd1306_WriteCommand(0x20);  // 0.77xVcc
    
    // 17. 启用电荷泵
    ssd1306_WriteCommand(0x8D);  // Set DC-DC enable
    ssd1306_WriteCommand(0x14);  // Enable charge pump
    
    // 18. 开启显示
    ssd1306_SetDisplayOn(1);
    
    // 19. 清屏并初始化状态
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    SSD1306.Initialized = 1;
}
```

## 显示缓冲区管理

### 缓冲区操作

```c
// 填充整个屏幕
void ssd1306_Fill(SSD1306_COLOR color) {
    memset(SSD1306_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

// 更新屏幕显示
void ssd1306_UpdateScreen(void) {
    // 逐页发送数据到OLED
    for(uint8_t i = 0; i < SSD1306_HEIGHT/8; i++) {
        ssd1306_WriteCommand(0xB0 + i);  // 设置页面地址
        ssd1306_WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);  // 设置列地址低位
        ssd1306_WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);  // 设置列地址高位
        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH*i], SSD1306_WIDTH);  // 发送一页数据
    }
}

// 设置光标位置
void ssd1306_SetCursor(uint8_t x, uint8_t y) {
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}
```

### 双缓冲机制

系统采用单缓冲区设计，所有绘图操作都在内存缓冲区中进行，通过ssd1306_UpdateScreen()统一刷新到显示屏：

```c
// 典型的显示更新流程
void display_update_cycle(void) {
    // 1. 清除缓冲区
    ssd1306_Fill(Black);
    
    // 2. 在缓冲区中绘制内容
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Hello World", Font_6x8, White);
    
    // 3. 一次性刷新到屏幕
    ssd1306_UpdateScreen();
}
```

## 字体系统

### 字体数据结构

```c
// 字体描述符结构
typedef struct {
    const uint8_t FontWidth;    // 字符宽度
    uint8_t FontHeight;         // 字符高度
    const uint16_t *data;       // 字体数据指针
} FontDef;

// 系统支持的字体
extern FontDef Font_6x8;       // 6×8像素字体 (小字体)
extern FontDef Font_7x10;      // 7×10像素字体 (中字体)
extern FontDef Font_11x18;     // 11×18像素字体 (大字体)
extern FontDef Font_16x26;     // 16×26像素字体 (超大字体)
```

### 字符渲染原理

```c
// 字符渲染函数
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color) {
    uint32_t i, b, j;

    // 检查字符是否在可显示范围内
    if (SSD1306.CurrentX + Font.FontWidth >= SSD1306_WIDTH ||
        SSD1306.CurrentY + Font.FontHeight >= SSD1306_HEIGHT) {
        return 0;  // 超出显示范围
    }

    // 渲染字符的每一列
    for(i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];  // 获取字符数据

        // 渲染每一列的每个像素
        for(j = 0; j < Font.FontWidth; j++) {
            if((b << j) & 0x8000) {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, SSD1306.CurrentY + i, color);
            } else {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, SSD1306.CurrentY + i, !color);
            }
        }
    }

    // 更新光标位置
    SSD1306.CurrentX += Font.FontWidth;
    return ch;
}

// 字符串渲染函数
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color) {
    while (*str) {
        if (ssd1306_WriteChar(*str, Font, color) != *str) {
            return *str;  // 渲染失败
        }
        str++;
    }
    return *str;
}
```

### 字体特性对比

| 字体 | 尺寸 | 字符数/行 | 行数 | 适用场景 |
|------|------|-----------|------|----------|
| Font_6x8 | 6×8px | 21字符 | 8行 | 状态信息、数值显示 |
| Font_7x10 | 7×10px | 18字符 | 6行 | 主要内容显示 |
| Font_11x18 | 11×18px | 11字符 | 3行 | 标题、重要信息 |
| Font_16x26 | 16×26px | 8字符 | 2行 | 大号数字显示 |

## 页面管理系统

### 页面定义

```c
// 页面枚举定义
typedef enum {
    PAGE_MOTOR = 0,    // 电机控制页面
    PAGE_IMU = 1       // IMU姿态页面
} display_page_t;

// 全局页面状态
display_page_t current_page = PAGE_IMU;  // 默认显示IMU页面
```

### 页面切换机制

```c
void OLED_SwitchPage(display_page_t page) {
    // 检查页面参数有效性
    if (page >= PAGE_MOTOR && page <= PAGE_IMU) {
        current_page = page;

        // 立即清屏并更新显示，避免页面切换时的残留内容
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
    }
}
```

### 页面内容渲染

```c
void oled_task(void) {
    // 根据当前页面显示不同内容
    switch (current_page) {
        case PAGE_MOTOR:
            render_motor_page();
            break;

        case PAGE_IMU:
            render_imu_page();
            break;

        default:
            current_page = PAGE_IMU;  // 默认页面
            break;
    }

    // 更新屏幕显示
    ssd1306_UpdateScreen();
}

// 电机页面渲染
void render_motor_page(void) {
    // 第一行：显示左轮线速度
    Oled_Printf_H(5, 10, "L: %.2fm/s  ", encoder_data_A.speed_m_s);

    // 第二行：显示右轮线速度
    Oled_Printf_H(5, 20, "R: %.2fm/s  ", encoder_data_B.speed_m_s);

    // 第三行：显示ADC电压
    Oled_Printf_H(5, 30, "V:%.2fV  ", voltage);
}

// IMU页面渲染
void render_imu_page(void) {
    if (IMU_IsDataReady()) {
        // 第一行：显示Roll角度
        Oled_Printf_H(5, 10, "Roll: %.1f°  ", imu_data.roll);

        // 第二行：显示Pitch角度
        Oled_Printf_H(5, 20, "Pitch:%.1f°  ", imu_data.pitch);

        // 第三行：显示Yaw角度
        Oled_Printf_H(5, 30, "Yaw: %.1f°  ", imu_data.yaw);
    } else {
        // IMU数据未就绪，显示提示信息
        Oled_Printf_H(5, 10, "IMU: system init");
        Oled_Printf_H(5, 20, "Waiting for IMU...");
        Oled_Printf_H(5, 30, "            ");
    }
}
```

## 应用层实现

### 格式化输出函数

```c
// 小字体格式化输出 (Font_6x8)
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...) {
    char buffer[128];
    va_list args;
    int len;

    // 格式化字符串
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 设置光标并显示
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(buffer, Font_6x8, White);

    return len;
}

// 中字体格式化输出 (Font_7x10)
int Oled_Printf_H(uint8_t x, uint8_t y, const char *format, ...) {
    char buffer[128];
    va_list args;
    int len;

    // 格式化字符串
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 设置光标并显示
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(buffer, Font_7x10, White);

    return len;
}
```

### 任务调度集成

```c
void OLED_Init(void) {
    // 初始化SSD1306驱动
    ssd1306_Init();

    // 清屏
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

void oled_task(void) {
    // 100ms周期执行，更新显示内容
    // 具体实现见页面管理系统部分
}
```

### 串口控制集成

```c
// 通过串口命令切换页面
void handle_oled_commands(char* cmd) {
    if (strcmp(cmd, "page motor") == 0) {
        OLED_SwitchPage(PAGE_MOTOR);
        my_printf(&huart1, "Switched to motor page\r\n");
    } else if (strcmp(cmd, "page imu") == 0) {
        OLED_SwitchPage(PAGE_IMU);
        my_printf(&huart1, "Switched to IMU page\r\n");
    }
}
```

## 性能分析

### 显示更新性能

| 操作 | 执行时间 | 说明 |
|------|----------|------|
| ssd1306_Fill() | ~50μs | 内存填充操作 |
| ssd1306_UpdateScreen() | ~8ms | I2C传输1024字节 |
| ssd1306_WriteChar() | ~100μs | 单字符渲染 |
| ssd1306_WriteString() | ~2ms | 典型字符串(20字符) |
| oled_task() | ~10ms | 完整页面更新 |

### I2C通信分析

```c
// I2C传输时间计算
I2C_Clock = 100kHz (标准模式)
单字节传输时间 = 10位 / 100kHz = 100μs
完整屏幕更新 = (1024字节 + 命令开销) × 100μs ≈ 8ms

// 优化后的快速模式
I2C_Clock = 400kHz (快速模式)
完整屏幕更新 = (1024字节 + 命令开销) × 25μs ≈ 2ms
```

### 内存使用分析

```c
// 静态内存占用
SSD1306_Buffer = 1024字节 (显示缓冲区)
SSD1306结构体 = 8字节 (设备状态)
字体数据 = ~2KB (Font_6x8 + Font_7x10)
总计静态内存 ≈ 3KB

// 动态内存 (栈空间)
Oled_Printf() = 128字节 (格式化缓冲区)
oled_task() = ~32字节 (局部变量)
```

## 调试与测试

### 调试方法

1. **I2C通信检测**
   ```c
   void debug_i2c_communication(void) {
       HAL_StatusTypeDef status;

       // 检测设备是否响应
       status = HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 3, 100);

       if (status == HAL_OK) {
           my_printf(&huart1, "SSD1306 device ready\r\n");
       } else {
           my_printf(&huart1, "SSD1306 device not found! Status: %d\r\n", status);
       }
   }
   ```

2. **显示测试模式**
   ```c
   void oled_test_pattern(void) {
       // 测试1：全屏点亮
       ssd1306_Fill(White);
       ssd1306_UpdateScreen();
       HAL_Delay(1000);

       // 测试2：全屏关闭
       ssd1306_Fill(Black);
       ssd1306_UpdateScreen();
       HAL_Delay(1000);

       // 测试3：棋盘模式
       for (int y = 0; y < SSD1306_HEIGHT; y++) {
           for (int x = 0; x < SSD1306_WIDTH; x++) {
               SSD1306_COLOR color = ((x + y) % 2) ? White : Black;
               ssd1306_DrawPixel(x, y, color);
           }
       }
       ssd1306_UpdateScreen();
       HAL_Delay(2000);
   }
   ```

3. **字体渲染测试**
   ```c
   void test_font_rendering(void) {
       ssd1306_Fill(Black);

       // 测试不同字体
       ssd1306_SetCursor(0, 0);
       ssd1306_WriteString("Font_6x8", Font_6x8, White);

       ssd1306_SetCursor(0, 15);
       ssd1306_WriteString("Font_7x10", Font_7x10, White);

       ssd1306_SetCursor(0, 30);
       ssd1306_WriteString("Test123", Font_11x18, White);

       ssd1306_UpdateScreen();
   }
   ```

4. **页面切换测试**
   ```c
   void test_page_switching(void) {
       // 自动页面切换测试
       for (int i = 0; i < 10; i++) {
           OLED_SwitchPage(PAGE_MOTOR);
           HAL_Delay(2000);

           OLED_SwitchPage(PAGE_IMU);
           HAL_Delay(2000);
       }
   }
   ```

### 故障排除

#### 1. 显示屏无显示

**可能原因**:
- I2C连接问题
- 电源供电不足
- 初始化序列错误
- 设备地址错误

**排查步骤**:
```c
// 检查I2C连接
void diagnose_no_display(void) {
    // 1. 检查I2C设备响应
    debug_i2c_communication();

    // 2. 检查电源电压
    my_printf(&huart1, "Check 3.3V power supply\r\n");

    // 3. 尝试不同的I2C地址
    uint8_t addresses[] = {0x78, 0x7A};  // 0x3C<<1, 0x3D<<1
    for (int i = 0; i < 2; i++) {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addresses[i], 3, 100);
        my_printf(&huart1, "Address 0x%02X: %s\r\n",
                  addresses[i], (status == HAL_OK) ? "OK" : "FAIL");
    }
}
```

#### 2. 显示内容异常

**可能原因**:
- 缓冲区数据损坏
- 字体数据错误
- 坐标计算错误

**解决方案**:
```c
// 缓冲区完整性检查
void check_buffer_integrity(void) {
    // 检查缓冲区是否被意外修改
    static uint32_t buffer_checksum = 0;
    uint32_t current_checksum = 0;

    for (int i = 0; i < SSD1306_BUFFER_SIZE; i++) {
        current_checksum += SSD1306_Buffer[i];
    }

    if (buffer_checksum != 0 && current_checksum != buffer_checksum) {
        my_printf(&huart1, "Buffer corruption detected!\r\n");
    }

    buffer_checksum = current_checksum;
}
```

#### 3. 刷新率过低

**可能原因**:
- I2C时钟频率过低
- 任务调度周期过长
- 不必要的重复更新

**优化方案**:
```c
// 提高I2C时钟频率
void optimize_i2c_speed(void) {
    // 在STM32CubeMX中将I2C时钟设置为400kHz
    my_printf(&huart1, "Set I2C clock to 400kHz for better performance\r\n");
}

// 减少不必要的更新
void optimize_display_updates(void) {
    static uint32_t last_update_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 限制最大刷新率为10Hz
    if (current_time - last_update_time >= 100) {
        oled_task();
        last_update_time = current_time;
    }
}
```

## 优化建议

### 性能优化

1. **局部更新优化**
   ```c
   // 实现局部刷新，只更新变化的区域
   typedef struct {
       uint8_t x_start, x_end;
       uint8_t y_start, y_end;
       uint8_t dirty;
   } update_region_t;

   void ssd1306_UpdateRegion(update_region_t* region) {
       if (!region->dirty) return;

       for (uint8_t page = region->y_start/8; page <= region->y_end/8; page++) {
           ssd1306_WriteCommand(0xB0 + page);
           ssd1306_WriteCommand(0x00 + (region->x_start & 0x0F));
           ssd1306_WriteCommand(0x10 + ((region->x_start >> 4) & 0x0F));

           uint8_t width = region->x_end - region->x_start + 1;
           ssd1306_WriteData(&SSD1306_Buffer[page * SSD1306_WIDTH + region->x_start], width);
       }

       region->dirty = 0;
   }
   ```

2. **DMA传输优化**
   ```c
   // 使用DMA进行I2C传输，减少CPU占用
   void ssd1306_UpdateScreen_DMA(void) {
       // 配置DMA传输
       HAL_I2C_Mem_Write_DMA(&hi2c1, SSD1306_I2C_ADDR, 0x40, 1,
                             SSD1306_Buffer, SSD1306_BUFFER_SIZE);
   }
   ```

3. **双缓冲实现**
   ```c
   // 实现双缓冲，避免显示撕裂
   static uint8_t SSD1306_Buffer_Front[SSD1306_BUFFER_SIZE];
   static uint8_t SSD1306_Buffer_Back[SSD1306_BUFFER_SIZE];
   static uint8_t* current_buffer = SSD1306_Buffer_Back;

   void ssd1306_SwapBuffers(void) {
       // 交换前后缓冲区
       uint8_t* temp = SSD1306_Buffer_Front;
       SSD1306_Buffer_Front = SSD1306_Buffer_Back;
       SSD1306_Buffer_Back = temp;
       current_buffer = SSD1306_Buffer_Back;

       // 更新显示
       ssd1306_WriteData(SSD1306_Buffer_Front, SSD1306_BUFFER_SIZE);
   }
   ```

### 功能扩展

1. **动画效果**
   ```c
   // 实现简单的滚动文字效果
   void scroll_text_animation(char* text, uint8_t y) {
       static int16_t x_offset = SSD1306_WIDTH;

       ssd1306_SetCursor(x_offset, y);
       ssd1306_WriteString(text, Font_7x10, White);

       x_offset -= 2;  // 滚动速度
       if (x_offset < -strlen(text) * 7) {
           x_offset = SSD1306_WIDTH;  // 重新开始
       }
   }
   ```

2. **图形绘制**
   ```c
   // 添加基本图形绘制功能
   void ssd1306_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
   void ssd1306_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR color);
   void ssd1306_DrawCircle(uint8_t x, uint8_t y, uint8_t r, SSD1306_COLOR color);
   ```

3. **多级菜单系统**
   ```c
   // 实现多级菜单导航
   typedef struct menu_item {
       char* title;
       void (*action)(void);
       struct menu_item* submenu;
       struct menu_item* parent;
   } menu_item_t;

   void menu_system_init(void);
   void menu_navigate_up(void);
   void menu_navigate_down(void);
   void menu_select(void);
   void menu_back(void);
   ```

## 总结

STM32F407VET6智能小车的OLED显示系统具有以下特点：

### 技术优势
1. **高对比度**: OLED自发光技术，对比度高，视觉效果好
2. **低功耗**: 黑色像素不发光，整体功耗较低
3. **响应快**: 无背光延迟，响应时间快
4. **接口简单**: I2C两线接口，连接简便
5. **功能完整**: 支持多字体、多页面、格式化输出

### 技术指标
- **分辨率**: 128×64像素
- **显示颜色**: 单色 (白/黑)
- **通信接口**: I2C (100kHz/400kHz)
- **刷新率**: 10Hz (可优化至50Hz)
- **内存占用**: 3KB (缓冲区+字体)
- **响应时间**: 100ms (任务周期)

### 应用场景
该OLED显示系统适用于各种嵌入式设备的人机交互界面：
- 实时数据监控
- 系统状态显示
- 参数配置界面
- 简单的图形用户界面

通过合理的页面设计和性能优化，能够提供良好的用户体验和稳定的显示效果。
```
```
