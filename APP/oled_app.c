/**
 * @file oled_app.c
 * @brief OLED显示模块实现 - SSD1306 OLED显示驱动与多页面管理
 */
#include "oled_app.h"

display_page_t current_page = PAGE_MOTOR; // 全局页面变量，默认显示IMU页面
uint32_t num;                           // 数值变量
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
    char buffer[128];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(buffer, Font_6x8, White);
    return len;
}

int Oled_Printf_H(uint8_t x, uint8_t y, const char *format, ...)
{
    char buffer[128];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(buffer, Font_7x10, White);
    return len;
}

void OLED_Init(void)
{
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
}

void oled_task(void)
{
    // 根据当前页面显示不同内容
    switch (current_page) {
        case PAGE_MOTOR:
            // 电机控制页面
            // 第一行：显示左轮（编码器A）线速度
            Oled_Printf_H(5,10,"L: %.2fm/s  ",encoder_data_A.speed_m_s);

            // 第二行：显示右轮（编码器B）线速度
            Oled_Printf_H(5,20,"R: %.2fm/s  ",encoder_data_B.speed_m_s);

            // 第三行：显示ADC电压
            Oled_Printf_H(5,30,"V:%.2fV  ",voltage);

            // 第四行：显示Gary传感器8通道状态

            Oled_Printf_H(5,40,"G:%d%d%d%d%d%d%d%d",
                (gary_data.digital_data>>0)&0x01,
                (gary_data.digital_data>>1)&0x01,
                (gary_data.digital_data>>2)&0x01,
                (gary_data.digital_data>>3)&0x01,
                (gary_data.digital_data>>4)&0x01,
                (gary_data.digital_data>>5)&0x01,
                (gary_data.digital_data>>6)&0x01,
                (gary_data.digital_data>>7)&0x01);
            break;

        case PAGE_IMU:
            // IMU姿态页面
            // 检查IMU数据是否就绪
            if (IMU_IsDataReady()) {
                // 第一行：显示Roll角度
                Oled_Printf_H(5,10,"Roll: %.1f°  ",imu_data.roll);

                // 第二行：显示Pitch角度
                Oled_Printf_H(5,20,"Pitch:%.1f°  ",imu_data.pitch);

                // 第三行：显示Yaw角度
                Oled_Printf_H(5,30,"Yaw: %.1f°  ",imu_data.yaw);
            } else {
                // IMU数据未就绪，显示提示信息
                Oled_Printf_H(5,10,"IMU: system init");
                Oled_Printf_H(5,20,"Waiting for IMU...    ");
                Oled_Printf_H(5,30,"            ");
            }
            break;

        default:
            // 默认显示电机页面
            current_page = PAGE_MOTOR;
            break;
    }

    // 更新屏幕显示
    ssd1306_UpdateScreen();
}

/**
 * @brief 页面切换函数
 * @param page 要切换到的页面
 */
void OLED_SwitchPage(display_page_t page)
{
    // 检查页面参数有效性
    if (page >= PAGE_MOTOR && page <= PAGE_IMU) {
        current_page = page;

        // 立即清屏并更新显示，避免页面切换时的残留内容
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
    }
}







