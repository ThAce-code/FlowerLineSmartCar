/**
 * @file gary_app.c
 * @brief Gary灰度传感器模块实现 - 感为8通道灰度传感器数据读取与处理
 */
#include "gary_app.h"
#include "hardware_iic.h"
#include <stddef.h>

// 全局Gary数据定义
Gary_Data_t gary_data = {
    .digital_data = 0,            // 8通道数字数据初始值
    .analog_data = {0},           // 8通道模拟数据初始值
    .normalize_data = {0},        // 8通道归一化数据初始值
    .line_state = LINE_LOST,      // 循线状态初始值
    .line_error = 0,              // 位置偏差初始值
    .line_width = 0,              // 线宽初始值
    .data_ready = 0,              // 数据就绪标志
    .last_update_time = 0,        // 上次更新时间
    .comm_error_count = 0,        // 通信错误计数
    .init_status = 0              // 初始化状态
};

/**
 * @brief Gary传感器初始化函数
 */
void gary_init(void)
{
    uint8_t retry_count = 0;
    const uint8_t max_retries = 10;  // 最大重试次数

    // 初始化Gary数据结构
    gary_data.digital_data = 0;
    gary_data.line_state = LINE_LOST;
    gary_data.line_error = 0;
    gary_data.line_width = 0;
    gary_data.data_ready = 0;
    gary_data.last_update_time = HAL_GetTick();
    gary_data.comm_error_count = 0;
    gary_data.init_status = 0;

    // 清零模拟和归一化数据数组
    for(uint8_t i = 0; i < 8; i++) {
        gary_data.analog_data[i] = 0;
        gary_data.normalize_data[i] = 0;
    }

    // 循环检测传感器连接，参考例程做法
    while(Ping() != 0 && retry_count < max_retries) {
        HAL_Delay(10);  // 等待10ms后重试
        retry_count++;
        gary_data.comm_error_count++;
    }

    if(retry_count < max_retries) {
        // 初始化成功，配置传感器
        gary_data.init_status = 1;

        // 关闭归一化模式，确保初始状态一致
        IIC_Anolog_Normalize(0x00);
        HAL_Delay(10);  // 等待传感器处理
    } else {
        gary_data.init_status = 0;  // 初始化失败
    }
}

/**
 * @brief Gary传感器任务函数
 */
void gary_task(void)
{
    static uint8_t retry_count = 0;
    static uint8_t normalize_cycle = 0;  // 归一化周期计数
    uint8_t result;

    // 检查初始化状态
    if (gary_data.init_status == 0) {
        return;  // 传感器未初始化，直接返回
    }

    // 1. 读取数字模式数据
    gary_data.digital_data = IIC_Get_Digtal();

    // 2. 读取模拟模式数据
    result = IIC_Get_Anolog(gary_data.analog_data, 8);

    // 3. 检查读取结果
    if (result == 1) {  // 读取成功
        // 重置重试计数
        retry_count = 0;

        // 设置数据就绪标志
        gary_data.data_ready = 1;

        // 更新时间戳
        gary_data.last_update_time = HAL_GetTick();

        // 4. 每隔几个周期读取一次真正的归一化数据
        normalize_cycle++;
        if (normalize_cycle >= 3) {  // 每3个周期读取一次归一化数据
            normalize_cycle = 0;

            // 开启归一化模式，参考例程做法
            if (IIC_Anolog_Normalize(0xFF)) {  // 开启所有通道归一化
                HAL_Delay(10);  // 等待传感器处理，参考例程

                // 读取归一化数据
                if (IIC_Get_Anolog(gary_data.normalize_data, 8)) {
                    // 归一化数据读取成功
                }

                // 关闭归一化模式
                IIC_Anolog_Normalize(0x00);
            }
        } else {
            // 非归一化周期，使用简单映射
            for(uint8_t i = 0; i < 8; i++) {
                gary_data.normalize_data[i] = (gary_data.analog_data[i] * 100) / 255;
            }
        }

        // 更新循线状态、偏差和线宽
        gary_data.line_state = Gary_DetectLineState(gary_data.digital_data);
        gary_data.line_error = Gary_CalculateLineError(gary_data.digital_data);
        gary_data.line_width = Gary_GetLineWidth(gary_data.digital_data);

    } else {  // 读取失败
        // 处理通信失败和重试机制
        retry_count++;

        // 记录通信错误
        if (gary_data.comm_error_count < 255) {
            gary_data.comm_error_count++;
        }

        // 检查重试次数
        if (retry_count >= GARY_MAX_RETRY) {
            // 超过最大重试次数，清除数据就绪标志
            gary_data.data_ready = 0;
            retry_count = 0;  // 重置重试计数，下次继续尝试
        }
    }
}

// ==================== 状态检查函数 ====================

/**
 * @brief 检查Gary数据是否就绪
 */
uint8_t Gary_IsDataReady(void)
{
    return gary_data.data_ready;
}

/**
 * @brief 检查Gary是否已初始化
 */
uint8_t Gary_IsInitialized(void)
{
    return gary_data.init_status;
}

/**
 * @brief 清除Gary错误状态
 */
void Gary_ClearError(void)
{
    gary_data.comm_error_count = 0;
    gary_data.data_ready = 0;
}

// ==================== 数据访问函数 ====================

/**
 * @brief 获取数字模式数据
 */
uint8_t Gary_GetDigital(void)
{
    return gary_data.digital_data;
}

/**
 * @brief 获取模拟模式数据
 */
void Gary_GetAnalog(uint8_t *data)
{
    if(data != NULL) {
        for(uint8_t i = 0; i < 8; i++) {
            data[i] = gary_data.analog_data[i];
        }
    }
}

/**
 * @brief 获取归一化数据
 */
void Gary_GetNormalize(uint8_t *data)
{
    if(data != NULL) {
        for(uint8_t i = 0; i < 8; i++) {
            data[i] = gary_data.normalize_data[i];
        }
    }
}

/**
 * @brief 获取循线状态
 */
Gary_LineState_t Gary_GetLineState(void)
{
    return gary_data.line_state;
}

/**
 * @brief 获取位置偏差值 (PID控制接口)
 */
float Gary_GetLineError(void)
{
    return gary_data.line_error;
}

// ==================== 循线检测算法 ====================

/**
 * @brief 检测循线状态 (优化版本)
 * @note 基于新权重数组{-4,-3,-2,-1,1,2,3,4}重新优化阈值设置
 *       - 扩大中央状态范围：±10 → ±30
 *       - 调整中间状态阈值：30/60 → 60/85
 *       - 权重范围缩小43%，配合更宽松的阈值设置
 *       - 解决实际使用中只出现三种状态的问题
 */
Gary_LineState_t Gary_DetectLineState(uint8_t digital_data)
{
    // 基于8位数字数据进行状态检测
    // 注意：白场高电平(1)，黑场低电平(0)
    // 所以黑线会产生0，白场会产生1

    uint8_t line_bits = ~digital_data;  // 取反，让黑线变成1，白场变成0

    // 快速检测特殊情况
    if(line_bits == 0x00) {
        return LINE_LOST;  // 没有检测到线
    }

    // 使用位运算快速计算线的数量
    uint8_t line_count = 0;
    uint8_t temp = line_bits;
    while(temp) {
        line_count++;
        temp &= (temp - 1);  // 清除最低位的1
    }

    // 检测交叉路口或宽线
    if(line_count >= 6) {
        return LINE_INTERSECTION;
    }

    // 检测T型路口
    if((line_bits & 0xF0) == 0xF0) {  // 左侧4位全为1
        return LINE_T_LEFT;
    }
    if((line_bits & 0x0F) == 0x0F) {  // 右侧4位全为1
        return LINE_T_RIGHT;
    }

    // 直接调用偏差计算函数进行状态判定
    float center = Gary_CalculateLineError(digital_data);

    // 使用优化的阈值判断 (基于平均值算法调整)
    // 中央状态判定
    if(center >= -GARY_CENTER_THRESHOLD && center <= GARY_CENTER_THRESHOLD) {
        return LINE_CENTER;                    // [-0.5, +0.5]
    }

    // 右偏状态判定 (从小到大)
    if(center > GARY_CENTER_THRESHOLD) {
        if(center <= GARY_SLIGHT_THRESHOLD) {
            return LINE_SLIGHT_RIGHT;          // (0.5, 1.5]
        }
        else if(center <= GARY_MODERATE_THRESHOLD) {
            return LINE_MODERATE_RIGHT;        // (1.5, 2.5]
        }
        else {
            return LINE_SHARP_RIGHT;           // (2.5, +4.0]
        }
    }

    // 左偏状态判定 (从小到大，绝对值)
    if(center < -GARY_CENTER_THRESHOLD) {
        if(center >= -GARY_SLIGHT_THRESHOLD) {
            return LINE_SLIGHT_LEFT;           // [-1.5, -0.5)
        }
        else if(center >= -GARY_MODERATE_THRESHOLD) {
            return LINE_MODERATE_LEFT;         // [-2.5, -1.5)
        }
        else {
            return LINE_SHARP_LEFT;            // [-4.0, -2.5)
        }
    }

    return LINE_LOST;
}

/**
 * @brief 计算位置偏差 (平均值算法)
 * @note 基于平均值算法，输出范围[-4.0, +4.0]
 *       - 使用加权平均而非归一化计算
 *       - 解决单传感器±100极值问题
 *       - 实现更平滑的状态过渡
 */
float Gary_CalculateLineError(uint8_t digital_data)
{
    uint8_t line_bits = ~digital_data;  // 取反，让黑线变成1

    // 快速检测无线情况
    if(line_bits == 0x00) {
        return 0.0f;  // 没有检测到线，返回0偏差
    }

    // 使用预计算权重数组
    static const float weights[8] = GARY_LINE_WEIGHTS;  // {-4.0,-3.0,-2.0,-1.0,1.0,2.0,3.0,4.0}
    float weighted_sum = 0.0f;
    uint8_t black_line_count = 0;

    // 平均值计算循环
    for(uint8_t i = 0; i < 8; i++) {
        if(line_bits & (1 << i)) {
            weighted_sum += weights[i];
            black_line_count++;
        }
    }

    if(black_line_count > 0) {
        float error = weighted_sum / (float)black_line_count;

        // 范围限制
        if(error > GARY_ERROR_MAX) error = GARY_ERROR_MAX;
        if(error < GARY_ERROR_MIN) error = GARY_ERROR_MIN;

        return error;
    }

    return 0.0f;  // 异常情况，返回0偏差
}

/**
 * @brief 检测线宽信息
 */
uint8_t Gary_GetLineWidth(uint8_t digital_data)
{
    uint8_t line_bits = ~digital_data;  // 取反，让黑线变成1
    uint8_t line_width = 0;

    // 计算连续的黑线位数
    for(uint8_t i = 0; i < 8; i++) {
        if(line_bits & (1 << i)) {
            line_width++;
        }
    }

    return line_width;
}

/**
 * @brief 检测交叉路口
 */
uint8_t Gary_DetectIntersection(uint8_t digital_data)
{
    uint8_t line_bits = ~digital_data;  // 取反，让黑线变成1
    uint8_t line_count = 0;

    // 计算检测到的线的数量
    for(uint8_t i = 0; i < 8; i++) {
        if(line_bits & (1 << i)) {
            line_count++;
        }
    }

    // 如果检测到6个或以上传感器有线，认为是交叉路口
    return (line_count >= 6) ? 1 : 0;
}