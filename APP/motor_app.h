/**
 * @file motor_app.h
 * @brief 电机控制模块头文件 - TB6612FNG双路电机驱动控制
 */
#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include "mydefine.h"

// 电机状态枚举
typedef enum {
    MOTOR_STATE_STOP = 0,     // 停止
    MOTOR_STATE_FORWARD,      // 正转
    MOTOR_STATE_BACKWARD,     // 反转
    MOTOR_STATE_ERROR         // 错误
} Motor_State_t;

// 电机ID枚举
typedef enum {
    MOTOR_ID_LEFT = 0,
    MOTOR_ID_RIGHT
}Motor_ID_t;

// 电机硬件配置结构体 - 适配TB6612
typedef struct {
    TIM_HandleTypeDef* htim;    // PWM定时器句柄
    uint32_t channel;           // PWM通道
    GPIO_TypeDef* in1_port;     // IN1控制端口
    uint16_t in1_pin;           // IN1控制引脚
    GPIO_TypeDef* in2_port;     // IN2控制端口
    uint16_t in2_pin;           // IN2控制引脚
    GPIO_TypeDef* stby_port;    // STBY使能端口
    uint16_t stby_pin;          // STBY使能引脚
} Motor_Hardware_t;

// 电机实体结构体
typedef struct {
    Motor_ID_t motor_id;
    Motor_Hardware_t hw;      // 硬件配置
    int32_t speed;            // 当前速度 (-1000 到 +1000)
    Motor_State_t state;      // 当前状态
    uint8_t enable;           // 使能标志
} Motor_t;

/**
 * @brief 电机初始化函数
 */
int8_t Motor_Create(Motor_t* motor,
                    TIM_HandleTypeDef* htim,
                    uint32_t channel,
                    GPIO_TypeDef* in1_port,
                    uint16_t in1_pin,
                    GPIO_TypeDef* in2_port,
                    uint16_t in2_pin,
                    GPIO_TypeDef* stby_port,
                    uint16_t stby_pin);

/**
 * @brief 电机速度设置函数
 */
int8_t Motor_SetSpeed(Motor_t* motor, int32_t speed, uint8_t enable);

/**
 * @brief 电机停止函数
 */
int8_t Motor_Stop(Motor_t* motor);

/**
 * @brief 启动电机（串口命令用）
 */
void Motor_Start(void);

/**
 * @brief 停止电机（串口命令用）
 */
void Motor_StartStop(void);

/**
 * @brief 独立控制单个电机（用于测试）
 */
int8_t Motor_SetSpeedIndependent(Motor_t* motor, int32_t speed);


extern uint8_t enable;             // 电机使能标志
extern int32_t PWM_left_value;     // 左轮PWM值
extern int32_t PWM_right_value;    // 右轮PWM值
extern Motor_t motor1,motor2;      // 电机实例

/**
 * @brief 电机模块初始化函数
 */
void motor_init(void);

/**
 * @brief 电机任务函数
 */
void motor_task(void);


#endif