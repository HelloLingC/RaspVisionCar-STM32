// To replace HAL_Delay()
#include "stm32f1xx_hal.h"

uint32_t get_tick(void) {
    return SysTick->VAL;
}

void simple_delay_ms(uint32_t ms) {
    uint16_t i;
    while (ms--) {
        i = 12000;
        while (i--) {
            //__NOP();
        }
    }
}

void systick_delay_ms(uint32_t ms) {
    uint32_t temp;
    SysTick->LOAD = 9000*ms;
    SysTick->VAL=0X00;//清空计数器
    SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
    do
    {
    temp=SysTick->CTRL;//读取当前倒计数值
    }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
        SysTick->CTRL=0x00; //关闭计数器
        SysTick->VAL =0X00; //清空计数器
}

void delay_ms(uint32_t ms) {
    systick_delay_ms(ms);
}

