#include "pid_control.h"

float basic_speed = 0.3f;
float line_error;
float pid_line_out;
float yaw;
float pid_yaw_out;

PID_T PID_left_speed;
PID_T PID_right_speed;
PID_T PID_line;
PID_T PID_Angle;

pid_params_t left_speed = {
    .Kp = 180.0f,
    .Ki = 16.0f,
    .Kd = 18.0f,
    .out_max = 999.0f,
    .out_min = -999.0f,
};

pid_params_t right_speed = {
    .Kp = 180.0f,
    .Ki = 16.0f,
    .Kd = 18.0f,
    .out_max = 999.0f,
    .out_min = -999.0f,
};


pid_params_t line = {
    .Kp = 1.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    // .out_max = 0.2f,
    // .out_min = -0.2f,
};

pid_params_t Angle = {
    .Kp = 0,
    .Ki = 0.0f,
    .Kd = 0,
    .out_max = 0.0f,
    .out_min = 0.0f
};

void PID_init(void) {
    pid_init(&PID_left_speed,left_speed.Kp,left_speed.Ki,left_speed.Kd,
        0.0f,left_speed.out_max);

    pid_init(&PID_right_speed,right_speed.Kp,right_speed.Ki,right_speed.Kd,
        0.0f,right_speed.out_max);

    pid_init(&PID_line,line.Kp,line.Ki,line.Kd,0.0f,line.out_max);

    pid_init(&PID_Angle,Angle.Kp,Angle.Ki,Angle.Kd,0.0f,Angle.out_max);

    pid_set_target(&PID_left_speed,basic_speed);
    pid_set_target(&PID_right_speed,basic_speed);
    pid_set_target(&PID_line,0.0f);
    pid_set_target(&PID_Angle,0.0f);
}

// 添加PID重置函数供外部调用
void PID_reset_all(void) {
    pid_reset(&PID_left_speed);
    pid_reset(&PID_right_speed);
    pid_reset(&PID_line);
    pid_reset(&PID_Angle);
}

// 更新PID参数并重新初始化
void PID_update_params(void) {
    // 更新左轮速度环PID参数
    pid_set_params(&PID_left_speed, left_speed.Kp, left_speed.Ki, left_speed.Kd);
    pid_set_limit(&PID_left_speed, left_speed.out_max);
    
    // 更新右轮速度环PID参数
    pid_set_params(&PID_right_speed, right_speed.Kp, right_speed.Ki, right_speed.Kd);
    pid_set_limit(&PID_right_speed, right_speed.out_max);
    
    // 更新循线环PID参数
    pid_set_params(&PID_line, line.Kp, line.Ki, line.Kd);
    pid_set_limit(&PID_line, line.out_max);
    
    // 更新角度环PID参数
    pid_set_params(&PID_Angle, Angle.Kp, Angle.Ki, Angle.Kd);
    pid_set_limit(&PID_Angle, Angle.out_max);
    
    // 重置所有PID控制器，清除历史数据
    PID_reset_all();
}

void PID_Line_Control(void) {
    line_error = Gary_GetLineError() / 4.0f;
    pid_line_out= pid_calculate_positional(&PID_line, line_error);
    pid_line_out = pid_constrain(pid_line_out,line.out_min,line.out_max);

    pid_set_target(&PID_left_speed,basic_speed - pid_line_out);
    pid_set_target(&PID_right_speed,basic_speed + pid_line_out);
}

void PID_Angle_Control(void) {
    yaw = imu_data.yaw;
    pid_yaw_out = pid_calculate_incremental(&PID_Angle,yaw);
}

void pid_task(void) {
    if (!enable) return; // 安全检查：电机未使能时直接返回

    // PID_Line_Control();

    float speed_current_left = get_left_wheel_speed_ms();
    float speed_current_right = get_right_wheel_speed_ms();
    float pid_left_out = pid_calculate_positional(&PID_left_speed,speed_current_left);
    float pid_right_out = pid_calculate_positional(&PID_right_speed,speed_current_right);
    pid_left_out = pid_constrain(pid_left_out,left_speed.out_min,left_speed.out_max);
    pid_right_out = pid_constrain(pid_right_out,right_speed.out_min,right_speed.out_max);
    Motor_SetSpeed(&motor1,(int32_t)pid_left_out,enable);
    Motor_SetSpeed(&motor2,(int32_t)pid_right_out,enable);
    my_printf(&huart2,"%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",encoder_data_A.speed_m_s,encoder_data_B.speed_m_s,basic_speed,pid_left_out,pid_right_out);

}



