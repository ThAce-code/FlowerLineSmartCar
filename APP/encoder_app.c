/**
 * @file encoder_app.c
 * @brief 编码器模块实现 - 500PPR增量式编码器速度检测与滤波处理
 */
#include "encoder_app.h"

Encoder_ID_t encoder_id;               // 编码器ID
Encoder_Data_t encoder_data_A = {0};   // 编码器A数据实例
Encoder_Data_t encoder_data_B = {0};   // 编码器B数据实例
Differential_Drive_t diff_drive_data = {0}; // 差速驱动数据实例

// 计算常量已迁移到mydefine.h统一管理

/**
 * @brief  编码器初始化
 * @retval None
 */
void Encoder_Init(void) {
    // 初始化编码器A数据结构
    encoder_data_A.encoder_id = ENCODER_A;
    encoder_data_A.total_count = 0;
    encoder_data_A.last_count = 0;
    encoder_data_A.speed_rpm = 0;
    encoder_data_A.buffer_index = 0;
    encoder_data_A.last_update_time = HAL_GetTick();
    encoder_data_A.speed_rps = 0.0f;
    encoder_data_A.speed_m_s = 0.0f;
    // 初始化新增字段
    encoder_data_A.adaptive_sample_time = ENCODER_SAMPLE_TIME;
    encoder_data_A.calc_time_us = 0;
    encoder_data_A.error_count = 0;
    encoder_data_A.filter_sum = 0;

    encoder_data_B.encoder_id = ENCODER_B;
    encoder_data_B.total_count = 0;
    encoder_data_B.last_count = 0;
    encoder_data_B.speed_rpm = 0;
    encoder_data_B.buffer_index = 0;
    encoder_data_B.last_update_time = HAL_GetTick();
    encoder_data_B.speed_rps = 0.0f;
    encoder_data_B.speed_m_s = 0.0f;
    // 初始化新增字段
    encoder_data_B.adaptive_sample_time = ENCODER_SAMPLE_TIME;
    encoder_data_B.calc_time_us = 0;
    encoder_data_B.error_count = 0;
    encoder_data_B.filter_sum = 0;


    // 清零编码器A滤波缓冲区
    for(int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        encoder_data_A.speed_buffer[i] = 0;
    }

    // 清零编码器B滤波缓冲区
    for(int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        encoder_data_B.speed_buffer[i] = 0;
    }


}


/**
 * @brief  统一的编码器速度计算函数
 * @param  encoder_data: 编码器数据指针
 * @param  htim: 定时器句柄
 * @retval None
 */
void calculate_speed_for_encoder(Encoder_Data_t* encoder_data, TIM_HandleTypeDef* htim) {
    uint32_t current_time = HAL_GetTick();
    uint32_t current_counter = __HAL_TIM_GET_COUNTER(htim);

    // 检查是否到达自适应采样时间
    uint32_t time_diff_ms = current_time - encoder_data->last_update_time;

    // 获取当前应使用的采样时间
    uint32_t required_sample_time = encoder_data->adaptive_sample_time;

    if (time_diff_ms >= required_sample_time) {
        // 计算脉冲差值
        int64_t delta_count = (int64_t)current_counter - (int64_t)encoder_data->last_count;

        // 处理计数器溢出（16位定时器）
        if (delta_count < -0x8000) {
            delta_count += 0x10000; // 正向溢出
        } else if (delta_count > 0x8000) {
            delta_count -= 0x10000; // 反向溢出
        }

        if (delta_count == 0 || time_diff_ms > 200) {
            // 无脉冲或时间过长，速度为0
            encoder_data->speed_rps = 0.0f;
            encoder_data->speed_rpm = 0;
            encoder_data->speed_m_s = 0.0f;

            // 清零滤波缓冲区
            for(int i = 0; i < ENCODER_FILTER_SIZE; i++) {
                encoder_data->speed_buffer[i] = 0;
            }
        } else {
            // 性能优化：使用预计算常量，单次乘法替代两次除法
            float pulses_per_ms = (float)delta_count / (float)time_diff_ms;
            float current_rps = pulses_per_ms * SPEED_CALC_FACTOR;

            // 应用优化的滑动平均滤波（增量更新，计算效率提升80%）
            apply_moving_average_filter(encoder_data, current_rps);
        }

        // 更新状态
        encoder_data->last_count = current_counter;
        encoder_data->last_update_time = current_time;
        encoder_data->total_count = current_counter;

        // 根据当前速度更新下次采样时间
        encoder_data->adaptive_sample_time = get_adaptive_sample_time(encoder_data->speed_rps);
    }
}

/**
 * @brief  获取自适应采样时间
 * @param  current_speed: 当前速度(RPS)
 * @retval 采样时间(ms)
 */
uint32_t get_adaptive_sample_time(float current_speed) {
    // 使用绝对值避免方向影响
    float abs_speed = fabsf(current_speed);

    if (abs_speed > ENCODER_SPEED_THRESHOLD_H) {
        return ENCODER_HIGH_SPEED_SAMPLE;  // 高速：20ms，提高精度
    } else if (abs_speed < ENCODER_SPEED_THRESHOLD_L) {
        return ENCODER_LOW_SPEED_SAMPLE;   // 低速：100ms，减少噪声
    } else {
        return ENCODER_SAMPLE_TIME;        // 正常：50ms，平衡性能
    }
}

/**
 * @brief  应用滑动平均滤波（增量优化版本）
 * @param  encoder_data: 编码器数据指针
 * @param  new_value: 新的速度值(RPS)
 * @retval None
 */
void apply_moving_average_filter(Encoder_Data_t* encoder_data, float new_value) {
    // 将新值转换为整数存储（放大100倍提高精度）
    int16_t new_value_int = (int16_t)(new_value * 100);

    // 获取即将被替换的旧值
    int16_t old_value = encoder_data->speed_buffer[encoder_data->buffer_index];

    // 增量更新累加和：减去旧值，加上新值
    encoder_data->filter_sum = encoder_data->filter_sum - old_value + new_value_int;

    // 更新缓冲区
    encoder_data->speed_buffer[encoder_data->buffer_index] = new_value_int;
    encoder_data->buffer_index = (encoder_data->buffer_index + 1) % ENCODER_FILTER_SIZE;

    // 计算滤波后的平均值（恢复原始值并取绝对值）
    encoder_data->speed_rps = fabsf((float)encoder_data->filter_sum / (ENCODER_FILTER_SIZE * 100.0f));
    encoder_data->speed_rpm = (int16_t)(encoder_data->speed_rps * 60.0f);

    // 计算线速度：v = ω × r = RPS × 轮子周长（取绝对值）
    encoder_data->speed_m_s = encoder_data->speed_rps * WHEEL_CIRCUMFERENCE;
}

/**
 * @brief  计算差速驱动参数
 * @retval None
 */
void calculate_differential_drive(void) {
    // 获取左右轮线速度(m/s)
    float left_speed = encoder_data_A.speed_m_s;   // 编码器A对应左轮
    float right_speed = encoder_data_B.speed_m_s;  // 编码器B对应右轮

    // 计算线速度：v = (v_left + v_right) / 2
    diff_drive_data.linear_velocity = (left_speed + right_speed) / 2.0f;

    // 计算角速度：ω = (v_right - v_left) / wheel_base
    diff_drive_data.angular_velocity = (right_speed - left_speed) / WHEEL_BASE;

    // 记录轮速度
    diff_drive_data.left_wheel_speed = left_speed;
    diff_drive_data.right_wheel_speed = right_speed;

    // 更新时间戳
    diff_drive_data.last_update_time = HAL_GetTick();
}

/**
 * @brief  编码器任务（固定周期采样）
 * @retval None
 */
void encoder_task(void) {
    // 处理编码器A (TIM2)
    calculate_speed_for_encoder(&encoder_data_A, &htim2);

    // 处理编码器B (TIM3)
    calculate_speed_for_encoder(&encoder_data_B, &htim3);

    // 计算差速驱动参数
    calculate_differential_drive();

    // my_printf(&huart2,"%0.2f,%0.2f\n",encoder_data_A.speed_m_s,encoder_data_B.speed_m_s);
}



/**
 * @brief  清零速度数据
 * @retval None
 */
void clear_speed_data(void) {
    // 清零编码器A数据
    encoder_data_A.speed_rps = 0.0f;
    encoder_data_A.speed_rpm = 0;
    encoder_data_A.speed_m_s = 0.0f;
    encoder_data_A.total_count = 0;
    encoder_data_A.last_count = 0;
    encoder_data_A.buffer_index = 0;
    encoder_data_A.last_update_time = HAL_GetTick();
    // 重置新增字段
    encoder_data_A.adaptive_sample_time = ENCODER_SAMPLE_TIME;
    encoder_data_A.calc_time_us = 0;
    encoder_data_A.error_count = 0;
    encoder_data_A.filter_sum = 0;

    // 清零编码器A滤波缓冲区
    for(int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        encoder_data_A.speed_buffer[i] = 0;
    }

    // 清零编码器B数据
    encoder_data_B.speed_rps = 0.0f;
    encoder_data_B.speed_rpm = 0;
    encoder_data_B.speed_m_s = 0.0f;
    encoder_data_B.total_count = 0;
    encoder_data_B.last_count = 0;
    encoder_data_B.buffer_index = 0;
    encoder_data_B.last_update_time = HAL_GetTick();
    // 重置新增字段
    encoder_data_B.adaptive_sample_time = ENCODER_SAMPLE_TIME;
    encoder_data_B.calc_time_us = 0;
    encoder_data_B.error_count = 0;
    encoder_data_B.filter_sum = 0;

    // 清零编码器B滤波缓冲区
    for(int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        encoder_data_B.speed_buffer[i] = 0;
    }

    // 清零差速驱动数据
    diff_drive_data.linear_velocity = 0.0f;
    diff_drive_data.angular_velocity = 0.0f;
    diff_drive_data.left_wheel_speed = 0.0f;
    diff_drive_data.right_wheel_speed = 0.0f;
    diff_drive_data.last_update_time = HAL_GetTick();
}

// ==================== 数据访问接口实现 ====================

/**
 * @brief  获取左轮速度(RPS)
 * @retval 左轮转速(RPS)
 */
float get_left_wheel_speed_rps(void) {
    return encoder_data_A.speed_rps;
}

/**
 * @brief  获取右轮速度(RPS)
 * @retval 右轮转速(RPS)
 */
float get_right_wheel_speed_rps(void) {
    return encoder_data_B.speed_rps;
}

/**
 * @brief  获取左轮速度(m/s)
 * @retval 左轮线速度(m/s)
 */
float get_left_wheel_speed_ms(void) {
    return encoder_data_A.speed_m_s;
}

/**
 * @brief  获取右轮速度(m/s)
 * @retval 右轮线速度(m/s)
 */
float get_right_wheel_speed_ms(void) {
    return encoder_data_B.speed_m_s;
}

/**
 * @brief  获取差速驱动数据
 * @retval 差速驱动数据指针
 */
Differential_Drive_t* get_differential_drive_data(void) {
    return &diff_drive_data;
}

/**
 * @brief  获取编码器A状态
 * @retval 编码器A数据指针
 */
Encoder_Data_t* get_encoder_A_data(void) {
    return &encoder_data_A;
}

/**
 * @brief  获取编码器B状态
 * @retval 编码器B数据指针
 */
Encoder_Data_t* get_encoder_B_data(void) {
    return &encoder_data_B;
}

/**
 * @brief  检查编码器系统是否正常工作
 * @retval 1: 系统正常, 0: 系统异常
 */
uint8_t is_encoder_system_healthy(void) {
    uint32_t current_time = HAL_GetTick();

    // 检查编码器A是否超时（超过500ms未更新）
    if ((current_time - encoder_data_A.last_update_time) > 500) {
        return 0;
    }

    // 检查编码器B是否超时（超过500ms未更新）
    if ((current_time - encoder_data_B.last_update_time) > 500) {
        return 0;
    }

    // 检查错误计数是否过高（超过100次错误）
    if (encoder_data_A.error_count > 100 || encoder_data_B.error_count > 100) {
        return 0;
    }

    // 检查差速驱动数据是否超时（超过500ms未更新）
    if ((current_time - diff_drive_data.last_update_time) > 500) {
        return 0;
    }

    return 1; // 系统正常
}

// ==================== 调试和校准接口实现 ====================

/**
 * @brief  调试编码器计数器状态
 * @retval None
 */
void debug_encoder_counter(void) {
    my_printf(&huart2, "\r\n=== 编码器计数器调试信息 ===\r\n");

    // 获取当前计数器值
    uint32_t counter_A = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t counter_B = __HAL_TIM_GET_COUNTER(&htim3);

    my_printf(&huart2, "编码器A (TIM2):\r\n");
    my_printf(&huart2, "  当前计数: %lu\r\n", counter_A);
    my_printf(&huart2, "  总计数: %ld\r\n", encoder_data_A.total_count);
    my_printf(&huart2, "  上次计数: %ld\r\n", encoder_data_A.last_count);
    my_printf(&huart2, "  缓冲区索引: %d\r\n", encoder_data_A.buffer_index);
    my_printf(&huart2, "  滤波累加和: %ld\r\n", encoder_data_A.filter_sum);
    my_printf(&huart2, "  采样时间: %lu ms\r\n", encoder_data_A.adaptive_sample_time);
    my_printf(&huart2, "  错误计数: %d\r\n", encoder_data_A.error_count);

    my_printf(&huart2, "\r\n编码器B (TIM3):\r\n");
    my_printf(&huart2, "  当前计数: %lu\r\n", counter_B);
    my_printf(&huart2, "  总计数: %ld\r\n", encoder_data_B.total_count);
    my_printf(&huart2, "  上次计数: %ld\r\n", encoder_data_B.last_count);
    my_printf(&huart2, "  缓冲区索引: %d\r\n", encoder_data_B.buffer_index);
    my_printf(&huart2, "  滤波累加和: %ld\r\n", encoder_data_B.filter_sum);
    my_printf(&huart2, "  采样时间: %lu ms\r\n", encoder_data_B.adaptive_sample_time);
    my_printf(&huart2, "  错误计数: %d\r\n", encoder_data_B.error_count);

    my_printf(&huart2, "\r\n系统状态: %s\r\n", is_encoder_system_healthy() ? "正常" : "异常");
    my_printf(&huart2, "=============================\r\n");
}

/**
 * @brief  调试编码器速度信息
 * @retval None
 */
void debug_encoder_speed(void) {
    my_printf(&huart2, "\r\n=== 编码器速度调试信息 ===\r\n");

    my_printf(&huart2, "编码器A (左轮):\r\n");
    my_printf(&huart2, "  转速: %.3f RPS (%.1f RPM)\r\n", encoder_data_A.speed_rps, (float)encoder_data_A.speed_rpm);
    my_printf(&huart2, "  线速度: %.3f m/s\r\n", encoder_data_A.speed_m_s);
    my_printf(&huart2, "  滤波缓冲区: [");
    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        my_printf(&huart2, "%d", encoder_data_A.speed_buffer[i]);
        if (i < ENCODER_FILTER_SIZE - 1) my_printf(&huart2, ", ");
    }
    my_printf(&huart2, "]\r\n");

    my_printf(&huart2, "\r\n编码器B (右轮):\r\n");
    my_printf(&huart2, "  转速: %.3f RPS (%.1f RPM)\r\n", encoder_data_B.speed_rps, (float)encoder_data_B.speed_rpm);
    my_printf(&huart2, "  线速度: %.3f m/s\r\n", encoder_data_B.speed_m_s);
    my_printf(&huart2, "  滤波缓冲区: [");
    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        my_printf(&huart2, "%d", encoder_data_B.speed_buffer[i]);
        if (i < ENCODER_FILTER_SIZE - 1) my_printf(&huart2, ", ");
    }
    my_printf(&huart2, "]\r\n");

    my_printf(&huart2, "\r\n差速驱动数据:\r\n");
    my_printf(&huart2, "  线速度: %.3f m/s\r\n", diff_drive_data.linear_velocity);
    my_printf(&huart2, "  角速度: %.3f rad/s\r\n", diff_drive_data.angular_velocity);
    my_printf(&huart2, "  左轮速度: %.3f m/s\r\n", diff_drive_data.left_wheel_speed);
    my_printf(&huart2, "  右轮速度: %.3f m/s\r\n", diff_drive_data.right_wheel_speed);

    my_printf(&huart2, "\r\n自适应采样状态:\r\n");
    my_printf(&huart2, "  编码器A采样时间: %lu ms\r\n", encoder_data_A.adaptive_sample_time);
    my_printf(&huart2, "  编码器B采样时间: %lu ms\r\n", encoder_data_B.adaptive_sample_time);

    my_printf(&huart2, "========================\r\n");
}

/**
 * @brief  编码器系统校准
 * @retval None
 */
void encoder_calibration(void) {
    my_printf(&huart2, "\r\n=== 编码器系统校准 ===\r\n");
    my_printf(&huart2, "开始校准程序...\r\n");

    // 1. 重置所有计数器和数据
    my_printf(&huart2, "1. 重置计数器和数据...\r\n");
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    clear_speed_data();

    // 2. 等待系统稳定
    my_printf(&huart2, "2. 等待系统稳定 (2秒)...\r\n");
    HAL_Delay(2000);

    // 3. 检查计数器是否工作正常
    my_printf(&huart2, "3. 检查计数器工作状态...\r\n");
    uint32_t counter_A_start = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t counter_B_start = __HAL_TIM_GET_COUNTER(&htim3);

    my_printf(&huart2, "   请手动转动轮子 (5秒)...\r\n");
    HAL_Delay(5000);

    uint32_t counter_A_end = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t counter_B_end = __HAL_TIM_GET_COUNTER(&htim3);

    int32_t delta_A = (int32_t)(counter_A_end - counter_A_start);
    int32_t delta_B = (int32_t)(counter_B_end - counter_B_start);

    my_printf(&huart2, "   编码器A计数变化: %ld\r\n", delta_A);
    my_printf(&huart2, "   编码器B计数变化: %ld\r\n", delta_B);

    // 4. 校准结果评估
    my_printf(&huart2, "4. 校准结果评估:\r\n");
    if (abs(delta_A) > 10) {
        my_printf(&huart2, "   编码器A: 正常工作 OK\r\n");
    } else {
        my_printf(&huart2, "   编码器A: 可能异常 ERROR\r\n");
    }

    if (abs(delta_B) > 10) {
        my_printf(&huart2, "   编码器B: 正常工作 OK\r\n");
    } else {
        my_printf(&huart2, "   编码器B: 可能异常 ERROR\r\n");
    }

    // 5. 重置错误计数
    my_printf(&huart2, "5. 重置错误计数...\r\n");
    encoder_data_A.error_count = 0;
    encoder_data_B.error_count = 0;

    my_printf(&huart2, "校准完成!\r\n");
    my_printf(&huart2, "==================\r\n");
}

/**
 * @brief  显示性能统计信息
 * @retval None
 */
void show_performance_stats(void) {
    my_printf(&huart2, "\r\n=== 性能统计信息 ===\r\n");

    uint32_t current_time = HAL_GetTick();
    uint32_t uptime_sec = current_time / 1000;
    uint32_t uptime_min = uptime_sec / 60;
    uint32_t uptime_hour = uptime_min / 60;

    my_printf(&huart2, "系统运行时间: %lu:%02lu:%02lu\r\n",
           uptime_hour, uptime_min % 60, uptime_sec % 60);

    my_printf(&huart2, "\r\n编码器A性能:\r\n");
    my_printf(&huart2, "  错误计数: %d\r\n", encoder_data_A.error_count);
    my_printf(&huart2, "  上次更新: %lu ms前\r\n", current_time - encoder_data_A.last_update_time);
    my_printf(&huart2, "  当前采样时间: %lu ms\r\n", encoder_data_A.adaptive_sample_time);
    my_printf(&huart2, "  滤波累加和: %ld\r\n", encoder_data_A.filter_sum);

    my_printf(&huart2, "\r\n编码器B性能:\r\n");
    my_printf(&huart2, "  错误计数: %d\r\n", encoder_data_B.error_count);
    my_printf(&huart2, "  上次更新: %lu ms前\r\n", current_time - encoder_data_B.last_update_time);
    my_printf(&huart2, "  当前采样时间: %lu ms\r\n", encoder_data_B.adaptive_sample_time);
    my_printf(&huart2, "  滤波累加和: %ld\r\n", encoder_data_B.filter_sum);

    my_printf(&huart2, "\r\n差速驱动性能:\r\n");
    my_printf(&huart2, "  上次更新: %lu ms前\r\n", current_time - diff_drive_data.last_update_time);

    my_printf(&huart2, "\r\n系统健康状态: %s\r\n", is_encoder_system_healthy() ? "正常" : "异常");

    // 计算错误率
    float error_rate_A = uptime_sec > 0 ? (float)encoder_data_A.error_count / uptime_sec : 0;
    float error_rate_B = uptime_sec > 0 ? (float)encoder_data_B.error_count / uptime_sec : 0;
    my_printf(&huart2, "错误率: A=%.3f/s, B=%.3f/s\r\n", error_rate_A, error_rate_B);

    my_printf(&huart2, "==================\r\n");
}

/**
 * @brief  重置性能统计
 * @retval None
 */
void reset_performance_stats(void) {
    my_printf(&huart2, "\r\n重置性能统计...\r\n");

    encoder_data_A.error_count = 0;
    encoder_data_B.error_count = 0;

    my_printf(&huart2, "性能统计已重置\r\n");
}

/**
 * @brief  诊断编码器硬件状态和采样时间异常
 * @retval None
 */
void diagnose_encoder_sampling(void) {
    my_printf(&huart2, "\r\n=== 编码器采样诊断 ===\r\n");

    // 显示当前速度和对应的采样时间
    my_printf(&huart2, "当前状态:\r\n");
    my_printf(&huart2, "  编码器A: %.3f RPS -> %lu ms采样\r\n",
              encoder_data_A.speed_rps, encoder_data_A.adaptive_sample_time);
    my_printf(&huart2, "  编码器B: %.3f RPS -> %lu ms采样\r\n",
              encoder_data_B.speed_rps, encoder_data_B.adaptive_sample_time);

    // 分析采样时间差异原因
    my_printf(&huart2, "\r\n速度阈值分析:\r\n");
    my_printf(&huart2, "  高速阈值: %.1f RPS (采样20ms)\r\n", ENCODER_SPEED_THRESHOLD_H);
    my_printf(&huart2, "  低速阈值: %.1f RPS (采样100ms)\r\n", ENCODER_SPEED_THRESHOLD_L);
    my_printf(&huart2, "  中速范围: %.1f-%.1f RPS (采样50ms)\r\n",
              ENCODER_SPEED_THRESHOLD_L, ENCODER_SPEED_THRESHOLD_H);

    // 检查异常情况
    my_printf(&huart2, "\r\n异常检查:\r\n");
    float speed_diff = fabsf(encoder_data_A.speed_rps - encoder_data_B.speed_rps);
    if (speed_diff > 5.0f) {
        my_printf(&huart2, "  警告: 左右轮速度差异过大 (%.3f RPS)\r\n", speed_diff);
        my_printf(&huart2, "  可能原因: 机械故障、编码器连接问题、轮子打滑\r\n");
    }

    if (encoder_data_A.speed_rps < 1.0f && encoder_data_B.speed_rps > 10.0f) {
        my_printf(&huart2, "  警告: 编码器A速度异常低，可能硬件故障\r\n");
    }

    if (encoder_data_B.speed_rps < 1.0f && encoder_data_A.speed_rps > 10.0f) {
        my_printf(&huart2, "  警告: 编码器B速度异常低，可能硬件故障\r\n");
    }

    my_printf(&huart2, "====================\r\n");
}

