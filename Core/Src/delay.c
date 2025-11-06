// 使用 DWT 计数器实现微秒/毫秒延时，并覆盖 HAL_Delay()
#include "stm32f1xx_hal.h"

// DWT 初始化标记，避免重复初始化
static uint8_t dwt_initialized = 0;

static void dwt_init_if_needed(void)
{
    if (dwt_initialized) {
        return;
    }
    // 使能 DWT 计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT/ITM
    DWT->CYCCNT = 0;                                // 计数清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 使能 CYCCNT
    dwt_initialized = 1;
}

static inline uint32_t cycles_per_us(void)
{
    // SystemCoreClock 需在时钟配置后有效（Hz）
    return (SystemCoreClock / 1000000U);
}

void delay_us(uint32_t us)
{
    if (us == 0U) {
        return;
    }
    dwt_init_if_needed();
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * cycles_per_us();
    // 使用减法判断以适配 32 位计数器溢出
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

void delay_ms(uint32_t ms)
{
    // 避免 us 乘法溢出，分段调用
    while (ms--) {
        delay_us(1000U);
    }
}

// 覆盖 HAL 提供的弱符号，实现基于 DWT 的毫秒延时
void HAL_Delay(uint32_t Delay)
{
    delay_ms(Delay);
}

