/**
 * @file adc_app.c
 * @brief ADC采集模块实现 - 12位ADC DMA连续采样与平均值滤波
 */
#include "adc_app.h"

#define ADC_DMA_BUFFER_SIZE 32                        // DMA缓冲区大小
uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];         // DMA目标缓冲区
uint32_t adc_val;                                     // 平均ADC值
float voltage;                                        // 电压值

/**
 * @brief ADC任务函数
 * @retval 无
 */
void adc_task(void)
{
    uint32_t adc_sum = 0;

    for(uint16_t i = 0; i < ADC_DMA_BUFFER_SIZE; i++) // 计算DMA缓冲区中所有采样值的总和
    {
        adc_sum += adc_dma_buffer[i];
    }

    adc_val = adc_sum / ADC_DMA_BUFFER_SIZE;      // 计算平均ADC值
    voltage = ((float)adc_val * 3.3f) / 4096.0f; // 转换为实际电压值(12位分辨率, 3.3V参考电压)

    // 4. 使用计算出的平均值 (adc_val 或 voltage)
    // my_printf(&huart1, "Average ADC: %lu, Voltage: %.2fV\n", adc_val, voltage);
}