/**
 * @file JY901S_app.c
 * @brief IMU模块实现 - JY901S九轴姿态传感器数据读取与处理
 */
#include "JY901S_app.h"

// 全局IMU数据定义
IMU_Data_t imu_data = {
    .roll = 0.0f,             // 横滚角初始值
    .pitch = 0.0f,            // 俯仰角初始值
    .yaw = 0.0f,              // 偏航角初始值
    .data_ready = 0,          // 数据就绪标志
    .last_update_time = 0,    // 上次更新时间
    .comm_error_count = 0,    // 通信错误计数
    .init_status = 0          // 初始化状态
};

/**
 * @brief IMU初始化函数
 * 初始化维特SDK，注册I2C读写函数和回调函数
 */
void IMU_Init(void)
{
    int32_t result;

    // 初始化IMU数据结构
    imu_data.roll = 0.0f;
    imu_data.pitch = 0.0f;
    imu_data.yaw = 0.0f;
    imu_data.data_ready = 0;
    imu_data.last_update_time = HAL_GetTick();
    imu_data.comm_error_count = 0;
    imu_data.init_status = 0;

    // 1. 注册I2C读写函数到维特SDK
    result = WitI2cFuncRegister(IMU_I2C_Write, IMU_I2C_Read);
    if (result != WIT_HAL_OK) {
        // I2C函数注册失败
        return;
    }

    // 2. 注册数据更新回调函数
    result = WitRegisterCallBack(IMU_RegUpdateCallback);
    if (result != WIT_HAL_OK) {
        // 回调函数注册失败
        return;
    }

    // 3. 注册延时函数
    result = WitDelayMsRegister(IMU_DelayMs);
    if (result != WIT_HAL_OK) {
        // 延时函数注册失败
        return;
    }

    // 4. 初始化维特SDK为I2C协议模式，设备地址0x50
    result = WitInit(WIT_PROTOCOL_I2C, IMU_I2C_ADDR);
    if (result != WIT_HAL_OK) {
        // SDK初始化失败
        return;
    }

    // 初始化成功
    imu_data.init_status = 1;
}

/**
 * @brief 设置IMU为6轴模式(纯陀螺仪+加速度计，不使用磁力计)
 * 避免磁场干扰，提高平移运动时的角度稳定性
 * @return 1-成功，0-失败
 */
int32_t IMU_SetGyroOnlyMode(void)
{
    int32_t result;
    uint8_t unlock_key = 0x69;  // KEY_UNLOCK值
    uint8_t axis6_mode = ALGRITHM6;  // 6轴算法模式

    // 1. 解锁寄存器
    result = WitWriteReg(KEY, unlock_key);
    if (result != WIT_HAL_OK) {
        return 0;  // 解锁失败
    }

    // 延时等待解锁生效
    HAL_Delay(20);

    // 2. 设置为6轴算法模式（不使用磁力计）
    result = WitWriteReg(AXIS6, axis6_mode);
    if (result != WIT_HAL_OK) {
        return 0;  // 设置失败
    }

    // 延时等待设置生效
    HAL_Delay(20);

    // 3. 保存配置到FLASH
    result = WitWriteReg(SAVE, 0x00);
    if (result != WIT_HAL_OK) {
        return 0;  // 保存失败
    }

    // 延时等待保存完成
    HAL_Delay(100);

    return 1;  // 成功
}

/**
 * @brief 读取当前算法模式
 * @return 0-9轴模式，1-6轴模式，0xFF-读取失败
 */
uint8_t IMU_GetAlgorithmMode(void)
{
    int32_t result;
    uint8_t mode_value;

    // 读取AXIS6寄存器
    result = WitReadReg(AXIS6, 1);
    if (result != WIT_HAL_OK) {
        return 0xFF;  // 读取失败
    }

    // 从SDK寄存器数组获取值
    extern int16_t sReg[];
    mode_value = (uint8_t)sReg[AXIS6];

    return mode_value;
}

/**
 * @brief IMU数据读取任务
 * 周期性读取JY901S欧拉角数据，20ms执行周期
 */
void imu_task(void)
{
    static uint8_t retry_count = 0;
    int32_t result;

    // 检查初始化状态
    if (imu_data.init_status == 0) {
        return;  // SDK未初始化，直接返回
    }

    // 1. 调用WitReadReg读取Roll、Pitch、Yaw寄存器
    // 从Roll寄存器开始，连续读取3个寄存器（Roll、Pitch、Yaw）
    result = WitReadReg(Roll, 3);

    // 2. 检查读取结果
    if (result == WIT_HAL_OK) {
        // 读取成功，数据会通过回调函数IMU_RegUpdateCallback自动更新
        // 重置重试计数
        retry_count = 0;

        // 3. 更新时间戳（数据在回调函数中已更新）
        imu_data.last_update_time = HAL_GetTick();

    } else {
        // 4. 处理通信失败和重试机制
        retry_count++;

        // 记录通信错误
        if (imu_data.comm_error_count < 255) {
            imu_data.comm_error_count++;
        }

        // 检查重试次数
        if (retry_count >= IMU_MAX_RETRY) {
            // 超过最大重试次数，清除数据就绪标志
            imu_data.data_ready = 0;
            retry_count = 0;  // 重置重试计数，下次继续尝试
        }
    }
}

/**
 * @brief 检查IMU数据是否就绪
 * @return 1-数据就绪，0-数据未就绪
 */
uint8_t IMU_IsDataReady(void)
{
    return imu_data.data_ready;
}

/**
 * @brief 清除IMU错误状态
 */
void IMU_ClearError(void)
{
    imu_data.comm_error_count = 0;
    imu_data.data_ready = 0;
}

/**
 * @brief 检查IMU是否已初始化
 * @return 1-已初始化，0-未初始化
 */
uint8_t IMU_IsInitialized(void)
{
    return imu_data.init_status;
}

// ==================== 维特SDK回调函数 ====================

/**
 * @brief 维特SDK寄存器更新回调函数
 * 当SDK读取到新的寄存器数据时会调用此函数
 * @param uiReg 起始寄存器地址
 * @param uiRegNum 寄存器数量
 */
void IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiRegNum)
{
    // 声明外部sReg数组（维特SDK中定义）
    extern int16_t sReg[];

    // 检查是否包含欧拉角寄存器
    if (uiReg <= Yaw && (uiReg + uiRegNum) > Roll) {
        // 转换Roll角度 (0x3d)
        if (uiReg <= Roll && (uiReg + uiRegNum) > Roll) {
            imu_data.roll = (float)sReg[Roll] * IMU_ANGLE_SCALE;
        }

        // 转换Pitch角度 (0x3e)
        if (uiReg <= Pitch && (uiReg + uiRegNum) > Pitch) {
            imu_data.pitch = (float)sReg[Pitch] * IMU_ANGLE_SCALE;
        }

        // 转换Yaw角度 (0x3f)
        if (uiReg <= Yaw && (uiReg + uiRegNum) > Yaw) {
            imu_data.yaw = (float)sReg[Yaw] * IMU_ANGLE_SCALE;
        }

        // 更新数据状态
        imu_data.data_ready = 1;
        imu_data.last_update_time = HAL_GetTick();
    }
}

/**
 * @brief 延时函数回调
 * 为维特SDK提供延时功能
 * @param ucMs 延时毫秒数
 */
void IMU_DelayMs(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

// ==================== I2C适配层函数 ====================

/**
 * @brief I2C写函数适配层
 * 将HAL I2C接口桥接到维特SDK
 * @param ucAddr 设备地址(7位地址，函数内部会左移1位)
 * @param ucReg 寄存器地址
 * @param p_ucVal 要写入的数据指针
 * @param uiLen 数据长度
 * @return 1-成功，0-失败
 */
int32_t IMU_I2C_Write(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    HAL_StatusTypeDef status;

    // 参数检查
    if (p_ucVal == NULL || uiLen == 0) {
        return 0;
    }

    // 使用HAL_I2C_Mem_Write进行I2C写操作
    // ucAddr已经是7位地址，HAL函数会自动处理地址左移
    status = HAL_I2C_Mem_Write(&hi2c2, ucAddr, ucReg, I2C_MEMADD_SIZE_8BIT,
                               p_ucVal, uiLen, IMU_COMM_TIMEOUT);

    if (status == HAL_OK) {
        return 1;  // 成功
    } else {
        // 记录错误
        if (imu_data.comm_error_count < 255) {
            imu_data.comm_error_count++;
        }
        return 0;  // 失败
    }
}

/**
 * @brief I2C读函数适配层
 * 将HAL I2C接口桥接到维特SDK
 * @param ucAddr 设备地址(7位地址，函数内部会左移1位)
 * @param ucReg 寄存器地址
 * @param p_ucVal 读取数据存储指针
 * @param uiLen 要读取的数据长度
 * @return 1-成功，0-失败
 */
int32_t IMU_I2C_Read(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    HAL_StatusTypeDef status;

    // 参数检查
    if (p_ucVal == NULL || uiLen == 0) {
        return 0;
    }

    // 使用HAL_I2C_Mem_Read进行I2C读操作
    // ucAddr已经是7位地址，HAL函数会自动处理地址左移
    status = HAL_I2C_Mem_Read(&hi2c2, ucAddr, ucReg, I2C_MEMADD_SIZE_8BIT,
                              p_ucVal, uiLen, IMU_COMM_TIMEOUT);

    if (status == HAL_OK) {
        return 1;  // 成功
    } else {
        // 记录错误
        if (imu_data.comm_error_count < 255) {
            imu_data.comm_error_count++;
        }
        return 0;  // 失败
    }
}