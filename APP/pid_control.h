#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "mydefine.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float out_max;
    float out_min;
}pid_params_t;

extern float basic_speed;
extern float line_error;
extern float pid_line_out;
extern PID_T PID_left_speed;
extern PID_T PID_right_speed;
extern PID_T PID_line;
extern PID_T PID_Angle;

extern pid_params_t left_speed;
extern pid_params_t right_speed;
extern pid_params_t line;
extern pid_params_t Angle;

/**
 * @brief 调度器初始化函数
 */
void PID_init(void);

void PID_Line_Control(void);

/**
 * @brief 调度器运行函数
 */
void pid_task(void);

/**
 * @brief PID重置函数
 */
void PID_reset_all(void);

/**
 * @brief 更新PID参数并重新初始化
 */
void PID_update_params(void);

#endif