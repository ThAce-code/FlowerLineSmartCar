#ifndef GARY_APP_H
#define GARY_APP_H

#include "mydefine.h"

// 循线状态枚举
typedef enum {
    LINE_LOST = 0,           // 丢线状态 (11111111 - 全白场)
    LINE_CENTER,             // 正中央 (11100111, 11000111 - 中间检测到黑线)
    LINE_SLIGHT_LEFT,        // 轻微左偏 (11001111, 10011111 - 黑线偏左)
    LINE_SLIGHT_RIGHT,       // 轻微右偏 (11110011, 11111001 - 黑线偏右)
    LINE_MODERATE_LEFT,      // 中度左偏 (10001111, 00011111 - 黑线明显偏左)
    LINE_MODERATE_RIGHT,     // 中度右偏 (11111000, 11110001 - 黑线明显偏右)
    LINE_SHARP_LEFT,         // 急剧左偏 (00111111, 00001111 - 黑线大幅偏左)
    LINE_SHARP_RIGHT,        // 急剧右偏 (11111100, 11110000 - 黑线大幅偏右)
    LINE_INTERSECTION,       // 交叉路口 (00000000 - 全黑线)
    LINE_T_LEFT,            // 左T型路口 (00000111 - 左侧全黑)
    LINE_T_RIGHT,           // 右T型路口 (11100000 - 右侧全黑)
    LINE_SEARCHING          // 寻线状态
} Gary_LineState_t;

// Gary传感器数据结构
typedef struct {
    uint8_t digital_data;            // 8通道数字数据 (位图)
    uint8_t analog_data[8];          // 8通道模拟数据 (0-255)
    uint8_t normalize_data[8];       // 8通道归一化数据
    Gary_LineState_t line_state;     // 当前循线状态
    float line_error;                // 位置偏差值 (-4.0到+4.0)
    uint8_t line_width;              // 检测到的线宽
    uint8_t data_ready;              // 数据就绪标志
    uint32_t last_update_time;       // 上次更新时间(ms)
    uint8_t comm_error_count;        // 通信错误计数
    uint8_t init_status;             // 初始化状态标志
} Gary_Data_t;

extern Gary_Data_t gary_data; // 全局Gary数据

/**
 * @brief Gary传感器初始化函数
 */
void gary_init(void);

/**
 * @brief Gary传感器任务函数
 */
void gary_task(void);

/**
 * @brief 检查Gary数据是否就绪
 */
uint8_t Gary_IsDataReady(void);

/**
 * @brief 检查Gary是否已初始化
 */
uint8_t Gary_IsInitialized(void);

/**
 * @brief 清除Gary错误状态
 */
void Gary_ClearError(void);

/**
 * @brief 获取数字模式数据
 */
uint8_t Gary_GetDigital(void);

/**
 * @brief 获取模拟模式数据
 */
void Gary_GetAnalog(uint8_t *data);

/**
 * @brief 获取归一化数据
 */
void Gary_GetNormalize(uint8_t *data);

/**
 * @brief 获取循线状态
 */
Gary_LineState_t Gary_GetLineState(void);

/**
 * @brief 获取位置偏差值
 */
float Gary_GetLineError(void);

/**
 * @brief 检测循线状态
 */
Gary_LineState_t Gary_DetectLineState(uint8_t digital_data);

/**
 * @brief 计算位置偏差 (平均值算法)
 */
float Gary_CalculateLineError(uint8_t digital_data);

/**
 * @brief 检测线宽信息
 */
uint8_t Gary_GetLineWidth(uint8_t digital_data);

/**
 * @brief 检测交叉路口
 */
uint8_t Gary_DetectIntersection(uint8_t digital_data);



#endif
