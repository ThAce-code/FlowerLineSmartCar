/**
 * @file oled_app.h
 * @brief OLED显示模块头文件 - SSD1306 OLED显示驱动与页面管理系统
 */
#ifndef OLED_APP_H
#define OLED_APP_H

#include "mydefine.h"

// 显示页面枚举
typedef enum {
    PAGE_MOTOR = 0,    // 电机控制页面
    PAGE_IMU = 1       // IMU姿态页面
} display_page_t;

extern display_page_t current_page; // 全局页面变量

/**
 * @brief OLED初始化函数
 */
void OLED_Init(void);

/**
 * @brief OLED任务函数
 */
void oled_task(void);

/**
 * @brief OLED格式化输出函数
 */
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...);

/**
 * @brief OLED高亮格式化输出函数
 */
int Oled_Printf_H(uint8_t x, uint8_t y, const char *format, ...);

/**
 * @brief 页面切换函数
 */
void OLED_SwitchPage(display_page_t page);

#endif
