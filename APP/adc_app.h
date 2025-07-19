/**
 * @file adc_app.h
 * @brief ADC采集模块头文件 - 12位ADC DMA连续采样系统
 */
#ifndef ADC_APP_H
#define ADC_APP_H

#include "mydefine.h"

#define ADC_DMA_BUFFER_SIZE 32         // DMA缓冲区大小
extern uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE]; // DMA目标缓冲区
extern uint32_t adc_val;               // 平均ADC值
extern float voltage;                  // 电压值

/**
 * @brief ADC任务函数
 */
void adc_task(void);

#endif