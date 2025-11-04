#include "buzzer.h"

#define BUZZER_PIN GPIO_PIN_14
#define BUZZER_PORT GPIOC

uint8_t buzzer_active_flag = 0;
uint32_t buzzer_timer = 0;

#define QUEUE_MAX_SIZE 12

typedef struct {
    uint16_t data[QUEUE_MAX_SIZE];
    int front;
    int rear;
} SegQueue_t;

static SegQueue_t buzzer_queue;

void buzzer_init(void) {
    buzzer_queue.front = 0;
    buzzer_queue.rear = 0;
}

void buzzer_enqueue(uint16_t duration_ms) {
    if(buzzer_queue.rear + 1 == QUEUE_MAX_SIZE) {
        buzzer_queue.rear = 0;
    } else {
        buzzer_queue.rear++;
    }
    if(buzzer_queue.rear == buzzer_queue.front) {
        return; // full
    }

    buzzer_queue.data[buzzer_queue.rear] = duration_ms;
}

uint16_t buzzer_dequeue(void) {
    if(buzzer_queue.front == buzzer_queue.rear) {
        return 0; // empty
    }
    if(buzzer_queue.front + 1 == QUEUE_MAX_SIZE) {
        buzzer_queue.front = 0;
    } else {
        buzzer_queue.front++;
    }
    return buzzer_queue.data[buzzer_queue.front];
}

// duration * 20ms = duration_ms
void buzzer_on(uint16_t duration) {
    // if buzzer already active, ignore the request
    if(!buzzer_active_flag) {
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
        buzzer_active_flag = 1;
        buzzer_timer = duration;
    }  
}

void buzzer_off(void) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    buzzer_active_flag = 0;
    buzzer_timer = 0;
}

void buzzer_update(void) {
    if (buzzer_active_flag) {
        buzzer_timer--;
        if (buzzer_timer <= 0) {
            buzzer_off();
        }
    }
}