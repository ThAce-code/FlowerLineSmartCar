/**
 * @file usart_app.c
 * @brief 串口通信模块实现 - UART1 DMA接收与命令解析处理
 */
#include "usart_app.h"

uint8_t uart_rx_dma_buffer[128]; // UART接收DMA缓冲区
uint8_t uart_dma_buffer[128];    // UART DMA缓冲区
uint16_t uart_rx_index;          // 接收索引
uint8_t uart_flag = 0;           // 串口标志
uint16_t uart_rx_size = 0;       // 接收数据大小

static cmd_state_t g_state = CMD_STATE_IDLE; // 命令状态机

// 添加字符串清理函数
void clean_string(char *str) {
    // 去除末尾的换行符和回车符
    int len = strlen(str);
    while (len > 0 && (str[len-1] == '\r' || str[len-1] == '\n' || str[len-1] == ' ')) {
        str[len-1] = '\0';
        len--;
    }
}

// 参数解析函数 - 解析空格分隔的命令参数
int parse_command_params(char *input, char **cmd, char **params, int max_params) {
    if (!input || !cmd || !params) return -1; // 参数检查

    // 复用现有的字符串清理函数
    clean_string(input);

    // 解析主命令
    *cmd = input;
    char *space = strchr(input, ' ');
    int param_count = 0;

    if (space) {
        *space = '\0'; // 分割主命令
        char *token = space + 1;

        // 解析参数
        while (token && param_count < max_params) {
            // 跳过多余空格
            while (*token == ' ') token++;
            if (*token == '\0') break;

            params[param_count++] = token;
            token = strchr(token, ' ');
            if (token) *token++ = '\0';
        }
    }

    return param_count; // 返回参数数量
}

int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    char buffer[512]; // 临时存储格式化后的字符串
    va_list arg;      // 处理可变参数
    int len;          // 最终字符串长度

    va_start(arg, format);
    // 安全地格式化字符串到 buffer
    len = vsnprintf(buffer, sizeof(buffer), format, arg);
    va_end(arg);

    // 通过 HAL 库发送 buffer 中的内容
    HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFFFF);
    return len;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. 确认是目标串口 (USART2)
    if (huart->Instance == USART2)
    {
        // 2. 紧急停止当前的 DMA 传输 (如果还在进行中)
        //    因为空闲中断意味着发送方已经停止，防止 DMA 继续等待或出错
        HAL_UART_DMAStop(huart);

        // 3. 将 DMA 缓冲区中有效的数据 (Size 个字节) 复制到待处理缓冲区
        memcpy(uart_dma_buffer, uart_rx_dma_buffer, Size);
        // 注意：这里使用了 Size，只复制实际接收到的数据
        uart_rx_size = Size;
        // 4. 举起"到货通知旗"，告诉主循环有数据待处理
        uart_flag = 1;

        // 5. 清空 DMA 接收缓冲区，为下次接收做准备
        //    虽然 memcpy 只复制了 Size 个，但清空整个缓冲区更保险
        memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));

        // 6. **关键：重新启动下一次 DMA 空闲接收**
        //    必须再次调用，否则只会接收这一次
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));

        // 7. 如果之前关闭了半满中断，可能需要在这里再次关闭 (根据需要)
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}



void handle_LEFT_PWM_set_command() {
    my_printf(&huart2,"请输入左轮pwm(-1000~1000):\r\n");
    g_state = CMD_STATE_WAIT_LEFT_PWM;
}
void handle_RIGHT_PWM_set_command() {
    my_printf(&huart2,"请输入右轮pwm(-1000~1000):\r\n");
    g_state = CMD_STATE_WAIT_RIGHT_PWM;
}
void handle_PWM_check_command() {
    my_printf(&huart2,"左轮PWM:%d\r\n",PWM_left_value);
    my_printf(&huart2,"右轮PWM:%d\r\n",PWM_right_value);
}

// PWM参数化指令处理函数 - 支持pwm [left|right] [value]格式
void handle_PWM_command_with_params(char** params, int param_count) {
    if (param_count == 0) {
        // 无参数时显示当前PWM状态
        my_printf(&huart2,"左轮PWM:%d\r\n",PWM_left_value);
        my_printf(&huart2,"右轮PWM:%d\r\n",PWM_right_value);
        return;
    }

    if (param_count == 1) {
        // 单参数时显示指定轮子的PWM状态
        if (strcmp(params[0], "left") == 0) {
            my_printf(&huart2,"左轮PWM:%d\r\n",PWM_left_value);
        } else if (strcmp(params[0], "right") == 0) {
            my_printf(&huart2,"右轮PWM:%d\r\n",PWM_right_value);
        } else {
            my_printf(&huart2,"错误：参数必须是 left 或 right\r\n");
        }
        return;
    }

    if (param_count == 2) {
        // 双参数时设置PWM值
        int32_t pwm_value = atoi(params[1]);

        // 参数范围验证
        if (pwm_value < -1000 || pwm_value > 1000) {
            my_printf(&huart2,"错误：PWM值范围为 -1000 到 +1000\r\n");
            return;
        }

        if (strcmp(params[0], "left") == 0) {
            PWM_left_value = pwm_value;
            my_printf(&huart2,"左轮PWM已设置为: %d\r\n", PWM_left_value);
            Motor_SetSpeed(&motor1, PWM_left_value, 1);
        } else if (strcmp(params[0], "right") == 0) {
            PWM_right_value = pwm_value;
            my_printf(&huart2,"右轮PWM已设置为: %d\r\n", PWM_right_value);
            Motor_SetSpeed(&motor2, PWM_right_value, 1);
        } else {
            my_printf(&huart2,"错误：参数必须是 left 或 right\r\n");
        }
        return;
    }

    // 参数过多
    my_printf(&huart2,"错误：参数过多，格式: pwm [left|right] [value]\r\n");
}

void handle_MOTOR_TEST_command() {
    my_printf(&huart2,"=== 独立电机测试模式 ===\r\n");
    my_printf(&huart2,"测试：右轮PWM=0，左轮PWM=999\r\n");

    // 使用独立控制函数，避免STBY引脚冲突
    Motor_SetSpeedIndependent(&motor2, 0);    // 右轮停止
    Motor_SetSpeedIndependent(&motor1, 999);  // 左轮全速

    my_printf(&huart2,"右轮已停止，左轮已启动\r\n");
    my_printf(&huart2,"请观察电机运行状态\r\n");
}
void handle_START_command(void) {
    Motor_Start();  // 使用新的Motor_Start函数
    my_printf(&huart2,"Motor started successfully\r\n");
}
void handle_STOP_command(void) {
    Motor_StartStop();  // 使用新的Motor_StartStop函数
    clear_speed_data(); // 清零速度数据
    my_printf(&huart2,"Motor stopped successfully\r\n");
}
void handle_SHOW_SPEED_command(void) {
    // 显示编码器A的数据
    my_printf(&huart2,"Encoder A - Speed RPS: %.2f rps\r\n", encoder_data_A.speed_rps);
    my_printf(&huart2,"Encoder A - Speed RPM: %d rpm\r\n", encoder_data_A.speed_rpm);
    my_printf(&huart2,"Encoder A - Speed m/s: %.3f m/s\r\n", encoder_data_A.speed_m_s);

    // 显示编码器B的数据
    my_printf(&huart2,"Encoder B - Speed RPS: %.2f rps\r\n", encoder_data_B.speed_rps);
    my_printf(&huart2,"Encoder B - Speed RPM: %d rpm\r\n", encoder_data_B.speed_rpm);
    my_printf(&huart2,"Encoder B - Speed m/s: %.3f m/s\r\n", encoder_data_B.speed_m_s);
}
void handle_SHOW_ADC_command(void) {
    my_printf(&huart2,"Voltage:%.2fV  \r\n",voltage);
    my_printf(&huart2,"ADC:%u\r\n",adc_val);
}

void handle_SHOW_IMU_command(void) {
    // 检查IMU是否已初始化
    if (!IMU_IsInitialized()) {
        my_printf(&huart2,"IMU未初始化\r\n");
        return;
    }

    // 检查IMU数据是否就绪
    if (!IMU_IsDataReady()) {
        my_printf(&huart2,"IMU数据未就绪，请稍候...\r\n");
        return;
    }

    // 显示IMU欧拉角数据
    my_printf(&huart2,"=== IMU姿态数据 ===\r\n");
    my_printf(&huart2,"Roll:  %.1f°\r\n", imu_data.roll);
    my_printf(&huart2,"Pitch: %.1f°\r\n", imu_data.pitch);
    my_printf(&huart2,"Yaw:   %.1f°\r\n", imu_data.yaw);
    my_printf(&huart2,"更新时间: %lu ms\r\n", imu_data.last_update_time);

    // 显示通信状态
    if (imu_data.comm_error_count > 0) {
        my_printf(&huart2,"通信错误次数: %d\r\n", imu_data.comm_error_count);
    } else {
        my_printf(&huart2,"通信状态: 正常\r\n");
    }
}

// 传感器数据合并显示函数 - 统一显示所有传感器数据
void handle_SENSOR_command(void) {
    my_printf(&huart2,"=== 传感器数据总览 ===\r\n");

    // 显示编码器速度数据
    my_printf(&huart2,"--- 编码器速度 ---\r\n");
    my_printf(&huart2,"Encoder A - Speed RPS: %.2f rps\r\n", encoder_data_A.speed_rps);
    my_printf(&huart2,"Encoder A - Speed RPM: %d rpm\r\n", encoder_data_A.speed_rpm);
    my_printf(&huart2,"Encoder A - Speed m/s: %.3f m/s\r\n", encoder_data_A.speed_m_s);
    my_printf(&huart2,"Encoder B - Speed RPS: %.2f rps\r\n", encoder_data_B.speed_rps);
    my_printf(&huart2,"Encoder B - Speed RPM: %d rpm\r\n", encoder_data_B.speed_rpm);
    my_printf(&huart2,"Encoder B - Speed m/s: %.3f m/s\r\n", encoder_data_B.speed_m_s);

    // 显示ADC数据
    my_printf(&huart2,"--- ADC电压 ---\r\n");
    my_printf(&huart2,"Voltage:%.2fV  \r\n",voltage);
    my_printf(&huart2,"ADC:%u\r\n",adc_val);

    // 显示IMU数据
    my_printf(&huart2,"--- IMU姿态 ---\r\n");
    if (!IMU_IsInitialized()) {
        my_printf(&huart2,"IMU未初始化\r\n");
    } else if (!IMU_IsDataReady()) {
        my_printf(&huart2,"IMU数据未就绪，请稍候...\r\n");
    } else {
        my_printf(&huart2,"Roll:  %.1f°\r\n", imu_data.roll);
        my_printf(&huart2,"Pitch: %.1f°\r\n", imu_data.pitch);
        my_printf(&huart2,"Yaw:   %.1f°\r\n", imu_data.yaw);
        my_printf(&huart2,"更新时间: %lu ms\r\n", imu_data.last_update_time);
        if (imu_data.comm_error_count > 0) {
            my_printf(&huart2,"通信错误次数: %d\r\n", imu_data.comm_error_count);
        } else {
            my_printf(&huart2,"通信状态: 正常\r\n");
        }
    }
    my_printf(&huart2,"==================\r\n");
}

// 编码器参数化指令处理函数 - 支持encoder [debug|cal]格式
void handle_ENCODER_command_with_params(char** params, int param_count) {
    if (param_count == 0) {
        // 无参数时显示使用帮助
        my_printf(&huart2,"编码器指令格式:\r\n");
        my_printf(&huart2,"  encoder debug - 显示编码器调试信息\r\n");
        my_printf(&huart2,"  encoder cal   - 执行编码器校准\r\n");
        return;
    }

    if (param_count == 1) {
        if (strcmp(params[0], "debug") == 0) {
            // 合并显示速度和计数器调试信息
            debug_encoder_speed();
            debug_encoder_counter();
        } else if (strcmp(params[0], "cal") == 0) {
            // 执行编码器校准
            encoder_calibration();
        } else {
            my_printf(&huart2,"错误：无效参数 '%s'\r\n", params[0]);
            my_printf(&huart2,"支持的参数: debug, cal\r\n");
        }
        return;
    }

    // 参数过多
    my_printf(&huart2,"错误：参数过多，格式: encoder [debug|cal]\r\n");
}

// 系统功能参数化指令处理函数 - 支持system [perf|reset|diag]格式
void handle_SYSTEM_command_with_params(char** params, int param_count) {
    if (param_count == 0) {
        // 无参数时显示使用帮助
        my_printf(&huart2,"系统指令格式:\r\n");
        my_printf(&huart2,"  system perf  - 显示性能统计信息\r\n");
        my_printf(&huart2,"  system reset - 重置统计数据\r\n");
        my_printf(&huart2,"  system diag  - 系统诊断信息\r\n");
        return;
    }

    if (param_count == 1) {
        if (strcmp(params[0], "perf") == 0) {
            // 显示性能统计
            show_performance_stats();
        } else if (strcmp(params[0], "reset") == 0) {
            // 重置统计数据
            reset_performance_stats();
        } else if (strcmp(params[0], "diag") == 0) {
            // 系统诊断
            diagnose_encoder_sampling();
        } else {
            my_printf(&huart2,"错误：无效参数 '%s'\r\n", params[0]);
            my_printf(&huart2,"支持的参数: perf, reset, diag\r\n");
        }
        return;
    }

    // 参数过多
    my_printf(&huart2,"错误：参数过多，格式: system [perf|reset|diag]\r\n");
}

// Gary传感器合并显示函数 - 统一显示所有Gary传感器信息
void handle_GARY_command(void) {
    // 检查Gary是否已初始化
    if (!Gary_IsInitialized()) {
        my_printf(&huart2,"Gary传感器未初始化\r\n");
        return;
    }

    // 检查数据是否就绪
    if (!Gary_IsDataReady()) {
        my_printf(&huart2,"Gary数据未就绪，请稍候...\r\n");
        return;
    }

    my_printf(&huart2,"=== Gary传感器完整信息 ===\r\n");

    // 显示传感器数据部分
    my_printf(&huart2,"--- 传感器数据 ---\r\n");
    my_printf(&huart2,"数字数据: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%u", (gary_data.digital_data >> i) & 0x01);
    }
    my_printf(&huart2," (0x%02X)\r\n", gary_data.digital_data);

    my_printf(&huart2,"模拟数据: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%3u ", gary_data.analog_data[i]);
    }
    my_printf(&huart2,"\r\n");

    my_printf(&huart2,"归一化值: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%3u ", gary_data.normalize_data[i]);
    }
    my_printf(&huart2,"\r\n");

    // 显示循线状态部分
    my_printf(&huart2,"--- 循线状态 ---\r\n");
    const char* state_names[] = {
        "丢线", "正中央", "轻微左偏", "轻微右偏",
        "中度左偏", "中度右偏", "急剧左偏", "急剧右偏",
        "交叉路口", "左T路口", "右T路口", "寻线中"
    };

    if(gary_data.line_state < 12) {
        my_printf(&huart2,"循线状态: %s\r\n", state_names[gary_data.line_state]);
    } else {
        my_printf(&huart2,"循线状态: 未知(%u)\r\n", gary_data.line_state);
    }

    my_printf(&huart2,"位置偏差: %.1f (范围: -4.0到+4.0)\r\n", gary_data.line_error);
    my_printf(&huart2,"线宽检测: %u个传感器\r\n", gary_data.line_width);
    my_printf(&huart2,"线检测: %s\r\n", (gary_data.line_state != LINE_LOST) ? "有线" : "无线");

    // 显示详细状态部分
    my_printf(&huart2,"--- 系统状态 ---\r\n");
    my_printf(&huart2,"初始化状态: %s\r\n", Gary_IsInitialized() ? "已初始化" : "未初始化");
    my_printf(&huart2,"数据就绪: %s\r\n", Gary_IsDataReady() ? "就绪" : "未就绪");
    my_printf(&huart2,"通信错误: %u次\r\n", gary_data.comm_error_count);
    my_printf(&huart2,"更新时间: %lu ms\r\n", gary_data.last_update_time);
    my_printf(&huart2,"上次更新: %lu ms前\r\n", HAL_GetTick() - gary_data.last_update_time);
    my_printf(&huart2,"========================\r\n");
}

// 页面参数化指令处理函数 - 支持page <motor|imu>格式
void handle_PAGE_command_with_params(char** params, int param_count) {
    if (param_count == 0) {
        // 无参数时显示使用帮助
        my_printf(&huart2,"页面切换指令格式:\r\n");
        my_printf(&huart2,"  page motor - 切换到电机页面\r\n");
        my_printf(&huart2,"  page imu   - 切换到IMU页面\r\n");
        return;
    }

    if (param_count == 1) {
        if (strcmp(params[0], "motor") == 0) {
            // 切换到电机页面
            OLED_SwitchPage(PAGE_MOTOR);
            my_printf(&huart2,"切换到电机页面\r\n");
        } else if (strcmp(params[0], "imu") == 0) {
            // 切换到IMU页面
            OLED_SwitchPage(PAGE_IMU);
            my_printf(&huart2,"切换到IMU页面\r\n");
        } else {
            my_printf(&huart2,"错误：无效页面 '%s'\r\n", params[0]);
            my_printf(&huart2,"支持的页面: motor, imu\r\n");
        }
        return;
    }

    // 参数过多
    my_printf(&huart2,"错误：参数过多，格式: page <motor|imu>\r\n");
}



void handle_interactive_input(char * buffer) {
    float value;

    // 清理字符串，去除换行符等
    clean_string(buffer);
    
    if (sscanf(buffer, "%f", &value) != 1) {
        // 尝试用atof兜底解析
        value = atof(buffer);
        if (value == 0 && buffer[0] != '0') {
            my_printf(&huart2, "invalid input format.\r\n");
            my_printf(&huart2,"DMA data: %s\n", buffer);
            g_state = CMD_STATE_IDLE;
            return;
        }
    }

    if (g_state == CMD_STATE_WAIT_LEFT_PWM) {
        PWM_left_value = (int32_t)value;  // 将输入值赋给左轮PWM值，支持负值
        my_printf(&huart2,"左轮PWM:%d\r\n",PWM_left_value);
        Motor_SetSpeed(&motor1, PWM_left_value, 1); // 应用新的PWM值
        g_state = CMD_STATE_IDLE;
    }
    else if (g_state == CMD_STATE_WAIT_RIGHT_PWM) {
        PWM_right_value = (int32_t)value;  // 将输入值赋给右轮PWM值，支持负值
        my_printf(&huart2,"右轮PWM:%d\r\n",PWM_right_value);
        Motor_SetSpeed(&motor2, PWM_right_value, 1); // 应用新的PWM值
        g_state = CMD_STATE_IDLE;
    }


}

void uart_command(uint8_t *buffer,uint16_t Size) {
    // 确保字符串以null结尾
    if (Size < sizeof(uart_dma_buffer)) {
        ((char *)buffer)[Size] = '\0';
    }

    // 创建可修改的缓冲区副本用于参数解析
    char input_buffer[256];
    strncpy(input_buffer, (char *)buffer, sizeof(input_buffer) - 1);
    input_buffer[sizeof(input_buffer) - 1] = '\0';

    // 参数解析
    char *cmd;
    char *params[5]; // 最多支持5个参数
    int param_count = parse_command_params(input_buffer, &cmd, params, 5);

    // 新的参数化指令处理
    if (strcmp(cmd, "pwm") == 0) {
        handle_PWM_command_with_params(params, param_count);
    }
    else if (strcmp(cmd, "start") == 0) {
        handle_START_command();
    }
    else if (strcmp(cmd, "stop") == 0) {
        handle_STOP_command();
    }
    else if (strcmp(cmd, "sensor") == 0) {
        handle_SENSOR_command();
    }
    else if (strcmp(cmd, "encoder") == 0) {
        handle_ENCODER_command_with_params(params, param_count);
    }
    else if (strcmp(cmd, "system") == 0) {
        handle_SYSTEM_command_with_params(params, param_count);
    }
    else if (strcmp(cmd, "gary") == 0) {
        if (param_count == 0) {
            handle_GARY_command(); // 无参数时显示完整信息
        } else if (param_count == 1) {
            if (strcmp(params[0], "ping") == 0) {
                handle_GARY_PING_command();
            } else if (strcmp(params[0], "reinit") == 0) {
                handle_GARY_REINIT_command();
            } else {
                my_printf(&huart2,"错误：无效Gary参数 '%s'\r\n", params[0]);
                my_printf(&huart2,"支持的参数: ping, reinit\r\n");
            }
        } else {
            my_printf(&huart2,"错误：Gary参数过多\r\n");
        }
    }
    else if (strcmp(cmd, "page") == 0) {
        handle_PAGE_command_with_params(params, param_count);
    }
    else if (strcmp(cmd, "speed") == 0) {
        handle_SPEED_command_with_params(params, param_count);
    }
    else if (strcmp(cmd, "pid") == 0) {
        handle_PID_command_with_params(params, param_count);
    }
    else if (strcmp(cmd, "help") == 0) {
        handle_HELP_command();
    }
    else if (g_state != CMD_STATE_IDLE) {
        handle_interactive_input((char *)buffer); // 处理交互式输入
    }
    else {
        my_printf(&huart2,"未知指令: %s\r\n", cmd);
        my_printf(&huart2,"输入 'help' 查看可用指令\r\n");
    }
}
/**
 * @brief  处理 DMA 接收到的 UART 数据
 * @param  None
 * @retval None
 */
void uart_task(void)
{
    // 1. 检查"到货通知旗"
    if(uart_flag == 0)
        return; // 旗子没举起来，说明没新货，直接返回

    // 2. 放下旗子，表示我们已经注意到新货了
    //    防止重复处理同一批数据
    uart_flag = 0;

    // 3. 处理 "待处理货架" (uart_dma_buffer) 中的数据
    if(uart_rx_size > 0) {
        uart_command(uart_dma_buffer,uart_rx_size);

    }

    //    (注意：如果数据不是字符串，需要用其他方式处理，比如按字节解析)

    // 4. 清空"待处理货架"，为下次接收做准备
    memset(uart_dma_buffer, 0, sizeof(uart_dma_buffer));
}





/**
 * @brief 帮助命令处理函数
 */
void handle_HELP_command(void) {
    my_printf(&huart2,"\r\n========== 智能小车命令帮助 (精简版) ==========\r\n");
    my_printf(&huart2,"=== 电机控制 (5个指令) ===\r\n");
    my_printf(&huart2,"pwm [left|right] [value] - PWM控制\r\n");
    my_printf(&huart2,"  示例: pwm          (查看状态)\r\n");
    my_printf(&huart2,"        pwm left 500 (设置左轮PWM)\r\n");
    my_printf(&huart2,"        pwm right -300 (设置右轮PWM)\r\n");
    my_printf(&huart2,"start                    - 启动电机\r\n");
    my_printf(&huart2,"stop                     - 停止电机\r\n");
    my_printf(&huart2,"speed <value>            - 设置基础速度(m/s)\r\n");
    my_printf(&huart2,"  示例: speed 0.5        (设置为0.5m/s)\r\n");
    my_printf(&huart2,"        speed            (查看当前速度)\r\n");
    my_printf(&huart2,"pid <controller> <kp> <ki> <kd> - 设置PID参数\r\n");
    my_printf(&huart2,"  示例: pid left 200 20 25 (设置左轮PID)\r\n");
    my_printf(&huart2,"        pid all 180 16 18  (设置所有速度环)\r\n");
    my_printf(&huart2,"        pid              (查看所有PID参数)\r\n");

    my_printf(&huart2,"\r\n=== 传感器数据 (2个指令) ===\r\n");
    my_printf(&huart2,"sensor                   - 显示所有传感器数据\r\n");
    my_printf(&huart2,"  (包含: 编码器速度+ADC电压+IMU姿态)\r\n");
    my_printf(&huart2,"encoder [debug|cal]      - 编码器功能\r\n");
    my_printf(&huart2,"  示例: encoder debug    (速度+计数器调试)\r\n");
    my_printf(&huart2,"        encoder cal      (编码器校准)\r\n");

    my_printf(&huart2,"\r\n=== Gary灰度传感器 (3个指令) ===\r\n");
    my_printf(&huart2,"gary                     - 显示完整传感器信息\r\n");
    my_printf(&huart2,"  (包含: 数据+循线状态+系统状态)\r\n");
    my_printf(&huart2,"gary ping                - 检测传感器连接\r\n");
    my_printf(&huart2,"gary reinit              - 重新初始化传感器\r\n");

    my_printf(&huart2,"\r\n=== 系统管理 (3个指令) ===\r\n");
    my_printf(&huart2,"system [perf|reset|diag] - 系统功能\r\n");
    my_printf(&huart2,"  示例: system perf      (性能统计)\r\n");
    my_printf(&huart2,"        system reset     (重置统计)\r\n");
    my_printf(&huart2,"        system diag      (系统诊断)\r\n");
    my_printf(&huart2,"page <motor|imu>         - 页面切换\r\n");
    my_printf(&huart2,"  示例: page motor       (切换到电机页面)\r\n");
    my_printf(&huart2,"        page imu         (切换到IMU页面)\r\n");
    my_printf(&huart2,"help                     - 显示此帮助\r\n");

    my_printf(&huart2,"\r\n=== 指令精简说明 ===\r\n");
    my_printf(&huart2,"原23个指令已精简为19个参数化指令\r\n");
    my_printf(&huart2,"主要变化: pls/prs/pc → pwm, sc/ac/imu → sensor\r\n");
    my_printf(&huart2,"删除指令: mtest (电机测试已移除)\r\n");
    my_printf(&huart2,"==========================================\r\n");
}

// ==================== Gary灰度传感器命令处理函数 ====================

/**
 * @brief Gary传感器连接检测命令
 */
void handle_GARY_PING_command(void) {
    my_printf(&huart2,"=== Gary传感器连接检测 ===\r\n");

    // 调用底层Ping函数检测连接
    if(Ping() == 0) {
        my_printf(&huart2,"Gary传感器连接正常\r\n");
        my_printf(&huart2,"I2C地址: 0x%02X\r\n", GARY_I2C_ADDR);
        my_printf(&huart2,"初始化状态: %s\r\n", Gary_IsInitialized() ? "已初始化" : "未初始化");
    } else {
        my_printf(&huart2,"Gary传感器连接失败\r\n");
        my_printf(&huart2,"请检查I2C3连接和传感器电源\r\n");
    }

    my_printf(&huart2,"通信错误计数: %u\r\n", gary_data.comm_error_count);
}

/**
 * @brief Gary传感器重新初始化命令
 */
void handle_GARY_REINIT_command(void) {
    my_printf(&huart2,"=== Gary传感器重新初始化 ===\r\n");

    // 重新初始化Gary传感器
    gary_init();

    // 检查初始化结果
    if (Gary_IsInitialized()) {
        my_printf(&huart2,"Gary传感器初始化成功\r\n");
        my_printf(&huart2,"I2C地址: 0x%02X\r\n", GARY_I2C_ADDR);
        my_printf(&huart2,"初始化状态: 已初始化\r\n");
    } else {
        my_printf(&huart2,"Gary传感器初始化失败\r\n");
        my_printf(&huart2,"请检查:\r\n");
        my_printf(&huart2,"1. I2C3硬件连接 (SCL:PC0, SDA:PC1)\r\n");
        my_printf(&huart2,"2. 传感器电源供应 (5V)\r\n");
        my_printf(&huart2,"3. I2C地址设置 (0x4C)\r\n");
    }

    my_printf(&huart2,"通信错误计数: %u\r\n", gary_data.comm_error_count);
}

/**
 * @brief Gary传感器数据显示命令
 */
void handle_GARY_DATA_command(void) {
    // 检查Gary是否已初始化
    if (!Gary_IsInitialized()) {
        my_printf(&huart2,"Gary传感器未初始化\r\n");
        return;
    }

    // 检查数据是否就绪
    if (!Gary_IsDataReady()) {
        my_printf(&huart2,"Gary数据未就绪，请稍候...\r\n");
        return;
    }

    my_printf(&huart2,"=== Gary传感器数据 ===\r\n");

    // 显示8通道数字数据
    my_printf(&huart2,"数字数据: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%u", (gary_data.digital_data >> i) & 0x01);
    }
    my_printf(&huart2," (0x%02X)\r\n", gary_data.digital_data);

    // 显示8通道模拟数据
    my_printf(&huart2,"模拟数据: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%3u ", gary_data.analog_data[i]);
    }
    my_printf(&huart2,"\r\n");

    // 显示8通道归一化数据
    my_printf(&huart2,"归一化值: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%3u ", gary_data.normalize_data[i]);
    }
    my_printf(&huart2,"\r\n");

    my_printf(&huart2,"更新时间: %lu ms\r\n", gary_data.last_update_time);
}

/**
 * @brief Gary循线状态显示命令
 */
void handle_GARY_LINE_command(void) {
    // 检查Gary是否已初始化
    if (!Gary_IsInitialized()) {
        my_printf(&huart2,"Gary传感器未初始化\r\n");
        return;
    }

    my_printf(&huart2,"=== Gary循线状态 ===\r\n");

    // 显示循线状态
    const char* state_names[] = {
        "丢线", "正中央", "轻微左偏", "轻微右偏",
        "中度左偏", "中度右偏", "急剧左偏", "急剧右偏",
        "交叉路口", "左T路口", "右T路口", "寻线中"
    };

    if(gary_data.line_state < 12) {
        my_printf(&huart2,"循线状态: %s\r\n", state_names[gary_data.line_state]);
    } else {
        my_printf(&huart2,"循线状态: 未知(%u)\r\n", gary_data.line_state);
    }

    my_printf(&huart2,"位置偏差: %.1f (范围: -4.0到+4.0)\r\n", gary_data.line_error);
    my_printf(&huart2,"线宽检测: %u个传感器\r\n", gary_data.line_width);
    my_printf(&huart2,"线检测: %s\r\n", (gary_data.line_state != LINE_LOST) ? "有线" : "无线");
}

/**
 * @brief Gary详细状态显示命令
 */
void handle_GARY_STATE_command(void) {
    // 检查Gary是否已初始化
    if (!Gary_IsInitialized()) {
        my_printf(&huart2,"Gary传感器未初始化\r\n");
        return;
    }

    my_printf(&huart2,"=== Gary详细状态 ===\r\n");

    // 显示传感器基本状态
    my_printf(&huart2,"初始化状态: %s\r\n", Gary_IsInitialized() ? "已初始化" : "未初始化");
    my_printf(&huart2,"数据就绪: %s\r\n", Gary_IsDataReady() ? "就绪" : "未就绪");
    my_printf(&huart2,"通信错误: %u次\r\n", gary_data.comm_error_count);

    // 显示原始数字数据的详细分析
    my_printf(&huart2,"\r\n--- 传感器通道分析 ---\r\n");
    my_printf(&huart2,"通道: 1 2 3 4 5 6 7 8\r\n");
    my_printf(&huart2,"数字: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%u ", (gary_data.digital_data >> i) & 0x01);
    }
    my_printf(&huart2,"\r\n");

    my_printf(&huart2,"模拟: ");
    for(uint8_t i = 0; i < 8; i++) {
        my_printf(&huart2,"%u ", gary_data.analog_data[i]);
    }
    my_printf(&huart2,"\r\n");

    // 显示循线算法分析
    my_printf(&huart2,"\r\n--- 循线算法分析 ---\r\n");
    uint8_t intersection = Gary_DetectIntersection(gary_data.digital_data);
    my_printf(&huart2,"交叉路口检测: %s\r\n", intersection ? "是" : "否");
    my_printf(&huart2,"线宽: %u个传感器\r\n", gary_data.line_width);
    my_printf(&huart2,"位置偏差: %.1f\r\n", gary_data.line_error);

    my_printf(&huart2,"\r\n--- 系统信息 ---\r\n");
    my_printf(&huart2,"采样周期: %u ms\r\n", GARY_SAMPLE_TIME);
    my_printf(&huart2,"上次更新: %lu ms前\r\n", HAL_GetTick() - gary_data.last_update_time);
}

// 基础速度设置命令处理函数 - 支持speed [value]格式
void handle_SPEED_command_with_params(char** params, int param_count) {
    if (param_count == 0) {
        // 无参数时显示当前基础速度
        my_printf(&huart2,"当前基础速度: %.3f m/s\r\n", basic_speed);
        my_printf(&huart2,"速度范围: 0.000 - 2.000 m/s\r\n");
        my_printf(&huart2,"使用格式: speed <value>\r\n");
        my_printf(&huart2,"示例: speed 0.5 (设置为0.5m/s)\r\n");
        return;
    }

    if (param_count == 1) {
        // 解析速度值
        float new_speed = atof(params[0]);
        
        // 参数范围验证
        if (new_speed < 0.0f || new_speed > 2.0f) {
            my_printf(&huart2,"错误：速度值范围为 0.000 到 2.000 m/s\r\n");
            my_printf(&huart2,"当前输入值: %.3f m/s\r\n", new_speed);
            return;
        }

        // 更新基础速度
        float old_speed = basic_speed;
        basic_speed = new_speed;
        
        // 重新设置PID目标值
        pid_set_target(&PID_left_speed, basic_speed);
        pid_set_target(&PID_right_speed, basic_speed);
        
        // 重置PID积分项，避免突变影响
        PID_reset_all();
        
        my_printf(&huart2,"基础速度已更新:\r\n");
        my_printf(&huart2,"  旧值: %.3f m/s\r\n", old_speed);
        my_printf(&huart2,"  新值: %.3f m/s\r\n", basic_speed);
        my_printf(&huart2,"PID目标值已同步更新\r\n");
        
        // 如果电机正在运行，提示用户
        if (enable) {
            my_printf(&huart2,"注意：电机正在运行，新速度将立即生效\r\n");
        } else {
            my_printf(&huart2,"提示：电机未启动，使用'start'命令启动电机\r\n");
        }
        
        return;
    }

    // 参数过多
    my_printf(&huart2,"错误：参数过多，格式: speed <value>\r\n");
    my_printf(&huart2,"示例: speed 0.8 (设置基础速度为0.8m/s)\r\n");
}

// PID参数设置命令处理函数 - 支持pid [controller] [kp] [ki] [kd]格式
void handle_PID_command_with_params(char** params, int param_count) {
    if (param_count == 0) {
        // 无参数时显示所有PID参数
        my_printf(&huart2,"=== 当前PID参数 ===\r\n");
        my_printf(&huart2,"左轮速度环: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                  left_speed.Kp, left_speed.Ki, left_speed.Kd);
        my_printf(&huart2,"右轮速度环: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                  right_speed.Kp, right_speed.Ki, right_speed.Kd);
        my_printf(&huart2,"循线环:     Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                  line.Kp, line.Ki, line.Kd);
        my_printf(&huart2,"角度环:     Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                  Angle.Kp, Angle.Ki, Angle.Kd);
        my_printf(&huart2,"\r\n使用格式:\r\n");
        my_printf(&huart2,"  pid <controller> <kp> <ki> <kd>\r\n");
        my_printf(&huart2,"控制器类型: left, right, line, angle, all\r\n");
        my_printf(&huart2,"示例: pid left 200 20 25\r\n");
        my_printf(&huart2,"      pid all 180 16 18 (设置所有速度环)\r\n");
        return;
    }

    if (param_count == 1) {
        // 单参数时显示指定控制器的PID参数
        if (strcmp(params[0], "left") == 0) {
            my_printf(&huart2,"左轮速度环: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                      left_speed.Kp, left_speed.Ki, left_speed.Kd);
        } else if (strcmp(params[0], "right") == 0) {
            my_printf(&huart2,"右轮速度环: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                      right_speed.Kp, right_speed.Ki, right_speed.Kd);
        } else if (strcmp(params[0], "line") == 0) {
            my_printf(&huart2,"循线环: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                      line.Kp, line.Ki, line.Kd);
        } else if (strcmp(params[0], "angle") == 0) {
            my_printf(&huart2,"角度环: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", 
                      Angle.Kp, Angle.Ki, Angle.Kd);
        } else {
            my_printf(&huart2,"错误：无效控制器 '%s'\r\n", params[0]);
            my_printf(&huart2,"支持的控制器: left, right, line, angle, all\r\n");
        }
        return;
    }

    if (param_count == 4) {
        // 四参数时设置PID值
        float kp = atof(params[1]);
        float ki = atof(params[2]);
        float kd = atof(params[3]);

        // 参数范围验证
        if (kp < 0 || ki < 0 || kd < 0) {
            my_printf(&huart2,"错误：PID参数不能为负值\r\n");
            return;
        }

        if (strcmp(params[0], "left") == 0) {
            // 更新左轮速度环参数
            float old_kp = left_speed.Kp, old_ki = left_speed.Ki, old_kd = left_speed.Kd;
            left_speed.Kp = kp;
            left_speed.Ki = ki;
            left_speed.Kd = kd;
            PID_update_params();
            my_printf(&huart2,"左轮速度环PID已更新:\r\n");
            my_printf(&huart2,"  旧值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", old_kp, old_ki, old_kd);
            my_printf(&huart2,"  新值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", kp, ki, kd);
            
        } else if (strcmp(params[0], "right") == 0) {
            // 更新右轮速度环参数
            float old_kp = right_speed.Kp, old_ki = right_speed.Ki, old_kd = right_speed.Kd;
            right_speed.Kp = kp;
            right_speed.Ki = ki;
            right_speed.Kd = kd;
            PID_update_params();
            my_printf(&huart2,"右轮速度环PID已更新:\r\n");
            my_printf(&huart2,"  旧值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", old_kp, old_ki, old_kd);
            my_printf(&huart2,"  新值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", kp, ki, kd);
            
        } else if (strcmp(params[0], "line") == 0) {
            // 更新循线环参数
            float old_kp = line.Kp, old_ki = line.Ki, old_kd = line.Kd;
            line.Kp = kp;
            line.Ki = ki;
            line.Kd = kd;
            PID_update_params();
            my_printf(&huart2,"循线环PID已更新:\r\n");
            my_printf(&huart2,"  旧值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", old_kp, old_ki, old_kd);
            my_printf(&huart2,"  新值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", kp, ki, kd);
            
        } else if (strcmp(params[0], "angle") == 0) {
            // 更新角度环参数
            float old_kp = Angle.Kp, old_ki = Angle.Ki, old_kd = Angle.Kd;
            Angle.Kp = kp;
            Angle.Ki = ki;
            Angle.Kd = kd;
            PID_update_params();
            my_printf(&huart2,"角度环PID已更新:\r\n");
            my_printf(&huart2,"  旧值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", old_kp, old_ki, old_kd);
            my_printf(&huart2,"  新值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", kp, ki, kd);
            
        } else if (strcmp(params[0], "all") == 0) {
            // 更新所有速度环参数（左轮和右轮）
            float old_left_kp = left_speed.Kp, old_left_ki = left_speed.Ki, old_left_kd = left_speed.Kd;
            float old_right_kp = right_speed.Kp, old_right_ki = right_speed.Ki, old_right_kd = right_speed.Kd;
            
            left_speed.Kp = kp;
            left_speed.Ki = ki;
            left_speed.Kd = kd;
            right_speed.Kp = kp;
            right_speed.Ki = ki;
            right_speed.Kd = kd;
            PID_update_params();
            
            my_printf(&huart2,"所有速度环PID已更新:\r\n");
            my_printf(&huart2,"左轮 - 旧值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", old_left_kp, old_left_ki, old_left_kd);
            my_printf(&huart2,"左轮 - 新值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", kp, ki, kd);
            my_printf(&huart2,"右轮 - 旧值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", old_right_kp, old_right_ki, old_right_kd);
            my_printf(&huart2,"右轮 - 新值: Kp=%.1f, Ki=%.1f, Kd=%.1f\r\n", kp, ki, kd);
            
        } else {
            my_printf(&huart2,"错误：无效控制器 '%s'\r\n", params[0]);
            my_printf(&huart2,"支持的控制器: left, right, line, angle, all\r\n");
            return;
        }

        // 如果电机正在运行，提示用户
        if (enable) {
            my_printf(&huart2,"注意：电机正在运行，新PID参数将立即生效\r\n");
        } else {
            my_printf(&huart2,"提示：电机未启动，使用'start'命令启动电机\r\n");
        }
        
        return;
    }

    // 参数数量错误
    my_printf(&huart2,"错误：参数数量错误，格式: pid <controller> <kp> <ki> <kd>\r\n");
    my_printf(&huart2,"控制器类型: left, right, line, angle, all\r\n");
    my_printf(&huart2,"示例: pid left 200 20 25\r\n");
}


