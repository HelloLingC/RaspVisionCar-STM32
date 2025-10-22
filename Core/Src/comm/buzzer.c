#include "main.h"
#include "buzzer.h"

#define BUZZER_PIN GPIO_PIN_14
#define BUZZER_PORT GPIOC

static uint8_t buzzer_active = 0;
static uint16_t when_buzzer_should_end = 0;

void buzzer_on(uint16_t duration_ms) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
    buzzer_active = 1;
    when_buzzer_should_end = HAL_GetTick() + duration_ms;
}

void buzzer_off(void) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    buzzer_active = 0;
}

void buzzer_update(void) {
    if (buzzer_active) {
        if (HAL_GetTick() >= when_buzzer_should_end) {
            buzzer_off();
        }
    }
}