/**
 * @file encoder_app.h
 * @brief 编码器模块头文件 - 500PPR增量式编码器速度检测系统
 */
#ifndef ENCODER_APP_H
#define ENCODER_APP_H

#include "mydefine.h"

// 编码器配置参数已迁移到mydefine.h统一管理

// 支持多个编码器实例
typedef enum {
    ENCODER_A = 0,  // TIM2
    ENCODER_B = 1   // TIM3
} Encoder_ID_t;

// 编码器数据结构
typedef struct {
    // === 现有字段（保持完全兼容） ===
    Encoder_ID_t encoder_id;
    int32_t total_count;               // 总计数值
    int32_t last_count;                // 上次计数值
    int16_t speed_rpm;                 // 当前转速(RPM)
    int16_t speed_buffer[ENCODER_FILTER_SIZE]; // 速度滤波缓冲区
    uint8_t buffer_index;              // 缓冲区索引
    uint32_t last_update_time;         // 上次更新时间
    float speed_rps;                   // 转速(RPS)
    float speed_m_s;                   // 线速度(m/s)

    // === 新增字段（优化扩展） ===
    uint32_t adaptive_sample_time;     // 自适应采样时间(ms)
    uint32_t calc_time_us;            // 计算耗时统计(微秒)
    uint16_t error_count;             // 错误计数
    int32_t filter_sum;               // 滤波缓冲区累加和（增量滤波优化）
} Encoder_Data_t;

// 差速驱动数据结构
typedef struct {
    float linear_velocity;             // 线速度(m/s)
    float angular_velocity;            // 角速度(rad/s)
    float left_wheel_speed;            // 左轮速度(m/s)
    float right_wheel_speed;           // 右轮速度(m/s)
    uint32_t last_update_time;         // 上次更新时间
} Differential_Drive_t;

// 全局编码器数据
extern Encoder_Data_t encoder_data_A; // 编码器A数据(TIM2)
extern Encoder_Data_t encoder_data_B; // 编码器B数据(TIM3)
extern Differential_Drive_t diff_drive_data; // 差速驱动数据

/**
 * @brief 编码器初始化函数
 */
void Encoder_Init(void);

/**
 * @brief 编码器任务函数
 */
void encoder_task(void);

/**
 * @brief 编码器速度计算函数
 */
void calculate_speed_for_encoder(Encoder_Data_t* encoder_data, TIM_HandleTypeDef* htim);

/**
 * @brief 清除速度数据函数
 */
void clear_speed_data(void);

/**
 * @brief 获取自适应采样时间
 */
uint32_t get_adaptive_sample_time(float current_speed);

/**
 * @brief 计算差速驱动参数
 */
void calculate_differential_drive(void);

/**
 * @brief 应用滑动平均滤波（增量优化版本）
 */
void apply_moving_average_filter(Encoder_Data_t* encoder_data, float new_value);

// ==================== 数据访问接口 ====================
/**
 * @brief 获取左轮速度(RPS)
 */
float get_left_wheel_speed_rps(void);

/**
 * @brief 获取右轮速度(RPS)
 */
float get_right_wheel_speed_rps(void);

/**
 * @brief 获取左轮速度(m/s)
 */
float get_left_wheel_speed_ms(void);

/**
 * @brief 获取右轮速度(m/s)
 */
float get_right_wheel_speed_ms(void);

/**
 * @brief 获取差速驱动数据
 */
Differential_Drive_t* get_differential_drive_data(void);

/**
 * @brief 获取编码器A状态
 */
Encoder_Data_t* get_encoder_A_data(void);

/**
 * @brief 获取编码器B状态
 */
Encoder_Data_t* get_encoder_B_data(void);

/**
 * @brief 检查编码器系统是否正常工作
 */
uint8_t is_encoder_system_healthy(void);

// ==================== 调试和校准接口 ====================
/**
 * @brief 调试编码器计数器状态
 */
void debug_encoder_counter(void);

/**
 * @brief 调试编码器速度信息
 */
void debug_encoder_speed(void);

/**
 * @brief 编码器系统校准
 */
void encoder_calibration(void);

/**
 * @brief 显示性能统计信息
 */
void show_performance_stats(void);

/**
 * @brief 重置性能统计
 */
void reset_performance_stats(void);

/**
 * @brief 诊断编码器硬件状态和采样时间异常
 */
void diagnose_encoder_sampling(void);

#endif