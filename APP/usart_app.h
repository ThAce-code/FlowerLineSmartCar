/**
 * @file usart_app.h
 * @brief 串口通信模块头文件 - UART2 DMA接收与命令解析系统
 */
#ifndef USART_APP_H
#define USART_APP_H

#include "mydefine.h"

extern uint8_t uart_rx_dma_buffer[128]; // UART接收DMA缓冲区
extern uint8_t uart_dma_buffer[128];    // UART DMA缓冲区

// 命令状态枚举
typedef enum {
    CMD_STATE_IDLE = 0,          // 空闲状态
    CMD_STATE_WAIT_LEFT_PWM,     // 等待左轮PWM
    CMD_STATE_WAIT_RIGHT_PWM,    // 等待右轮PWM
    CMD_STATE_WAIT_DIRECTION     // 等待方向
}cmd_state_t;

/**
 * @brief 自定义printf函数
 */
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @brief 串口任务函数
 */
void uart_task(void);

/**
 * @brief 参数解析函数 - 解析空格分隔的命令参数
 */
int parse_command_params(char *input, char **cmd, char **params, int max_params);

/**
 * @brief 传感器数据合并显示函数 - 统一显示所有传感器数据
 */
void handle_SENSOR_command(void);

/**
 * @brief PWM参数化指令处理函数 - 支持pwm [left|right] [value]格式
 */
void handle_PWM_command_with_params(char** params, int param_count);

/**
 * @brief 编码器参数化指令处理函数 - 支持encoder [debug|cal]格式
 */
void handle_ENCODER_command_with_params(char** params, int param_count);

/**
 * @brief 系统功能参数化指令处理函数 - 支持system [perf|reset|diag]格式
 */
void handle_SYSTEM_command_with_params(char** params, int param_count);

/**
 * @brief Gary传感器合并显示函数 - 统一显示所有Gary传感器信息
 */
void handle_GARY_command(void);

/**
 * @brief 页面参数化指令处理函数 - 支持page <motor|imu>格式
 */
void handle_PAGE_command_with_params(char** params, int param_count);

/**
 * @brief IMU显示命令处理函数
 */
void handle_SHOW_IMU_command(void);



/**
 * @brief 帮助命令处理函数
 */
void handle_HELP_command(void);

/**
 * @brief Gary传感器连接检测命令
 */
void handle_GARY_PING_command(void);

/**
 * @brief Gary传感器数据显示命令
 */
void handle_GARY_DATA_command(void);

/**
 * @brief Gary循线状态显示命令
 */
void handle_GARY_LINE_command(void);

/**
 * @brief Gary详细状态显示命令
 */
void handle_GARY_STATE_command(void);

/**
 * @brief Gary传感器重新初始化命令
 */
void handle_GARY_REINIT_command(void);

/**
 * @brief Gary传感器调试命令
 */
void handle_GARY_DEBUG_command(void);

/**
 * @brief 基础速度设置命令处理函数 - 支持speed [value]格式
 */
void handle_SPEED_command_with_params(char** params, int param_count);

/**
 * @brief PID参数设置命令处理函数 - 支持pid [controller] [kp] [ki] [kd]格式
 */
void handle_PID_command_with_params(char** params, int param_count);

#endif