/**
 * @file motor_app.c
 * @brief 电机控制模块实现 - TB6612FNG双路电机驱动控制
 */
#include "motor_app.h"
#include "pid_control.h"
#include "encoder_app.h"

int32_t PWM_left_value = 0;  // 左轮PWM值 (修复：设置为500测试)
int32_t PWM_right_value = 0; // 右轮PWM值 (修复：统一为500)
uint8_t enable = 0;            // 电机使能标志 (安全：默认禁用)

// 硬件配置参数
#define PWM_MAX_VALUE    1000  // PWM最大值
#define SPEED_MAX        1000  // 最大速度
#define SPEED_MIN        -1000 // 最小速度

Motor_t motor1,motor2; // 电机实例

/**
 * @brief  电机初始化函数
 * @param motor        电机实体
 * @param htim         PWM句柄
 * @param channel      PWM通道
 * @param in1_port     xN1_Port
 * @param in1_pin      xN1_Pin
 * @param in2_port     xN2_Port
 * @param in2_pin      xN2_Pin
 * @param stby_port    STBY_Port
 * @param stby_pin     STBY_Pin
 * @retval Motor_Result_t 初始化结果
 */
int8_t Motor_Create(Motor_t* motor,
                    TIM_HandleTypeDef* htim,
                    uint32_t channel,
                    GPIO_TypeDef* in1_port,
                    uint16_t in1_pin,
                    GPIO_TypeDef* in2_port,
                    uint16_t in2_pin,
                    GPIO_TypeDef* stby_port,
                    uint16_t stby_pin) {
    // 参数检查
    if (motor == NULL || htim == NULL) {
        return -1;
    }

    // 初始化硬件配置
    motor->hw.htim = htim;
    motor->hw.channel = channel;
    motor->hw.in1_port = in1_port;
    motor->hw.in1_pin = in1_pin;
    motor->hw.in2_port = in2_port;
    motor->hw.in2_pin = in2_pin;
    motor->hw.stby_port = stby_port;
    motor->hw.stby_pin = stby_pin;

    // 初始化电机状态
    motor->speed = 0;
    motor->state = MOTOR_STATE_STOP;
    motor->enable = 1;

    // 启动PWM
    HAL_TIM_PWM_Start(motor->hw.htim, motor->hw.channel);

    // 初始状态：停止
    HAL_GPIO_WritePin(motor->hw.stby_port, motor->hw.stby_pin, GPIO_PIN_SET);  // 使能
    HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, 0);

    return 0;
}
/**
 * @brief  设置电机速度和使能状态（TB6612驱动）
 * @param  motor: 电机实体指针
 * @param  speed: 速度值 (-1000 到 +1000)
 * @param  enable: 使能状态 (1:使能, 0:禁用)
 * @retval 0: 成功, -1: 参数错误
 */
int8_t Motor_SetSpeed(Motor_t* motor, int32_t speed, uint8_t enable) {
    // 参数检查
    if (motor == NULL) {
        return -1;
    }

    // 检查速度范围
    if (speed < -1000 || speed > 1000) {
        return -1;
    }

    // 设置使能状态
    motor->enable = enable;
    motor->speed = speed;

    if (!enable) {
        // 禁用单个电机：仅停止该电机，不影响STBY（避免影响另一个电机）
        motor->state = MOTOR_STATE_STOP;
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, 0);
        // 注意：不操作STBY引脚，避免影响另一个电机
        return 0;
    }

    // 使能电机：确保STBY开启（但不会因为单个电机禁用而关闭）
    HAL_GPIO_WritePin(motor->hw.stby_port, motor->hw.stby_pin, GPIO_PIN_SET);

    // 处理停止
    if (speed == 0) {
        // TB6612停止：IN1=0, IN2=0, PWM=任意值
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, 0);
        motor->state = MOTOR_STATE_STOP;
        return 0;
    }
    if (speed > 0) {
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_SET);
        motor->state = MOTOR_STATE_FORWARD;
    }
    if (speed < 0) {
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
        motor->state = MOTOR_STATE_BACKWARD;
    }

    uint16_t pwm_value = abs(speed);  // PWM值为速度绝对值

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, pwm_value);

    return 0;
}

void motor_task(void) {
    // 全局STBY控制：只有当全局enable为0时才关闭STBY
    if (!enable) {
        HAL_GPIO_WritePin(GPIOA, STBY_Pin, GPIO_PIN_RESET);  // 关闭整个TB6612
    } else {
        HAL_GPIO_WritePin(GPIOA, STBY_Pin, GPIO_PIN_SET);    // 开启整个TB6612
    }
}



/**
 * @brief  停止电机
 * @param  motor: 电机实体指针
 * @retval 0: 成功, -1: 参数错误
 */
int8_t Motor_Stop(Motor_t* motor) {
    // 参数检查
    if (motor == NULL) {
        return -1;
    }

    // 调用Motor_SetSpeed停止电机（保持使能状态）
    return Motor_SetSpeed(motor, 0, motor->enable);
}

/**
 * @brief  启动电机（通过串口命令调用）
 * @retval None
 */
void Motor_Start(void) {
    // 重置PID控制器状态，确保一致的启动条件
    PID_reset_all();

    // 清零编码器数据，确保一致的启动条件
    clear_speed_data();

    enable = 1;  // 使能电机
}

/**
 * @brief  停止电机（通过串口命令调用）
 * @retval None
 */
void Motor_StartStop(void) {
    enable = 0;  // 禁用电机
}

/**
 * @brief  独立控制单个电机（用于测试）
 * @param  motor: 电机实体指针
 * @param  speed: 速度值 (-1000 到 +1000)
 * @retval 0: 成功, -1: 参数错误
 */
int8_t Motor_SetSpeedIndependent(Motor_t* motor, int32_t speed) {
    // 参数检查
    if (motor == NULL) {
        return -1;
    }

    // 检查速度范围
    if (speed < -1000 || speed > 1000) {
        return -1;
    }

    // 确保TB6612芯片使能
    HAL_GPIO_WritePin(motor->hw.stby_port, motor->hw.stby_pin, GPIO_PIN_SET);

    // 更新电机状态
    motor->speed = speed;

    // 处理停止
    if (speed == 0) {
        // TB6612停止：IN1=0, IN2=0, PWM=0
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, 0);
        motor->state = MOTOR_STATE_STOP;
        return 0;
    }

    // 设置方向
    if (speed > 0) {
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_SET);
        motor->state = MOTOR_STATE_FORWARD;
    } else {
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);

        motor->state = MOTOR_STATE_BACKWARD;
    }

    // 设置PWM占空比
    uint16_t pwm_value = abs(speed);
    __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, pwm_value);

    return 0;
}


