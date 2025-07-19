/**
 * @file JY901S_app.h
 * @brief IMU模块头文件 - JY901S九轴姿态传感器驱动与数据处理系统
 */
#ifndef JY901S_APP_H
#define JY901S_APP_H

#include "mydefine.h"
#include "wit_c_sdk.h"
#include "REG.h"

// IMU配置参数
#define IMU_I2C_ADDR          0x50         // JY901S I2C设备地址
#define IMU_SAMPLE_TIME       20           // 采样时间间隔(ms)
#define IMU_COMM_TIMEOUT      100          // I2C通信超时时间(ms)
#define IMU_MAX_RETRY         3            // 最大重试次数

// 角度转换参数
#define IMU_ANGLE_SCALE       (180.0f / 32768.0f)  // 寄存器值转角度系数

// IMU数据结构
typedef struct {
    float roll;                    // 横滚角(度)
    float pitch;                   // 俯仰角(度)
    float yaw;                     // 偏航角(度)
    uint8_t data_ready;            // 数据就绪标志
    uint32_t last_update_time;     // 上次更新时间(ms)
    uint8_t comm_error_count;      // 通信错误计数
    uint8_t init_status;           // 初始化状态标志
} IMU_Data_t;

extern IMU_Data_t imu_data; // 全局IMU数据

/**
 * @brief IMU初始化函数
 */
void IMU_Init(void);

/**
 * @brief 设置IMU为6轴模式(纯陀螺仪+加速度计，不使用磁力计)
 */
int32_t IMU_SetGyroOnlyMode(void);

/**
 * @brief 读取当前算法模式
 */
uint8_t IMU_GetAlgorithmMode(void);

/**
 * @brief IMU数据读取任务
 */
void imu_task(void);

/**
 * @brief 检查数据是否就绪
 */
uint8_t IMU_IsDataReady(void);

/**
 * @brief 清除错误状态
 */
void IMU_ClearError(void);

/**
 * @brief 检查初始化状态
 */
uint8_t IMU_IsInitialized(void);

/**
 * @brief I2C写入函数
 */
int32_t IMU_I2C_Write(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/**
 * @brief I2C读取函数
 */
int32_t IMU_I2C_Read(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/**
 * @brief 维特SDK寄存器更新回调函数
 */
void IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiRegNum);

/**
 * @brief 延时函数
 */
void IMU_DelayMs(uint16_t ucMs);

#endif