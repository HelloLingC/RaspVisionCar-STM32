/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "motor_left.h"
#include "motor_right.h"
#include "rasp_comm.h"
#include "justfloat.h"
#include "ssd1306.h"
#include "feedforward_controller.h"
#include "delay.h"
#include "encoder.h"
#include "pid_controller.h"
#include "buzzer.h"

void PID_Calc(int16_t left_current, int16_t right_current, float* left_output, float* right_output);
void Motor_Left_Set_Raw_Speed(int16_t pwm_value);
void Motor_Right_Set_Raw_Speed(int16_t pwm_value);
// Add this in your main.h or similar header file

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 系统控制标志
volatile uint8_t system_stop_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// SystemCoreClock 72000000
#define SYSTICK_RELOAD_VAL (72000000 / 1000) - 1
uint32_t get_systick_value(void)
{
    return SysTick->VAL;  // 直接读取 STK_VAL 寄存器
}

uint32_t get_tick_ms(void)
{
    // Get CPU cycles and convert to milliseconds
    uint32_t cycles = DWT->CYCCNT;
    return cycles / (SystemCoreClock / 1000);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     利用Vofa发数据
  @param     data1，data2，data3，data4，data5，data6，要发的数据放进入参即可
  @return    null
  Sample     Vofa_data(Err,Steer_Angle,set_speed_right,right_wheel,set_speed_left,left_wheel);
  @note      如果不够用，继续往后加入参即可，继续加参数参考原则见下注释
-------------------------------------------------------------------------------------------------------------------*/
void Vofa_send_data(int data1, int data2, int data3, int data4, int data5, int data6) {
    float tempFloat[6];
    uint8_t tempData[28];

    tempFloat[0] = (float)data1;
    tempFloat[1] = (float)data2;
    tempFloat[2] = (float)data3;
    tempFloat[3] = (float)data4;
    tempFloat[4] = (float)data5;
    tempFloat[5] = (float)data6;

    memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));

    tempData[24] = 0x00;
    tempData[25] = 0x00;
    tempData[26] = 0x80;
    tempData[27] = 0x7f;

    HAL_UART_Transmit(&huart1, tempData, 28, HAL_MAX_DELAY);
}
volatile int16_t current_spd = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // 编码器初始化
  init_encoders();

  Motor_Init();

  //ff_init_default();

  // 初始化树莓派通信协议
  rasp_comm_init();

  // 初始化SSD1306显示屏
  if (SSD1306_Init() == 0) {
    // 显示欢迎信息
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Rasp Vision Car", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 20);
    SSD1306_Puts(__DATE__, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 30);
    SSD1306_Puts(__TIME__, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 40);
    SSD1306_Puts("System Ready", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
  } else {
    usart_log("SSD1306 Screen Initialization Failed");
  }

  HAL_Delay(1200);

  // Enable DWT (Data Watchpoint and Trace) unit
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  pid_init_default();
  int16_t target_rpm = 100;
  pid_set_target_rpm(target_rpm, target_rpm);

  HAL_TIM_Base_Start_IT(&htim1);
  // 启动比较中断
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

  buzzer_on(150);

  // usart_log("System Initialized");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // 检查停止标志
    if (system_stop_flag) {
      // 停止所有电机
      Motor_Left_Set_Raw_Speed(0);
      Motor_Right_Set_Raw_Speed(0);
      
      // 停止定时器
      HAL_TIM_Base_Stop_IT(&htim1);
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
      
      // 更新OLED显示停止状态
      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 0);
      SSD1306_Puts("Rasp Vision Car v1", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 15);
      SSD1306_Puts("Status: STOPPED", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 30);
      SSD1306_Puts("System Halted", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();
      
      // 进入停止状态循环
      while (system_stop_flag) {
        buzzer_update();
        HAL_Delay(100);
      }
      
      // 如果收到重新启动命令，重新初始化系统
      HAL_TIM_Base_Start_IT(&htim1);
      HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
      HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
    }
    
    // 处理树莓派通信buffer
    rasp_comm_process();
    buzzer_update();

    // Update OLED display
    uint32_t current_time = get_tick_ms();
    static uint32_t last_status_time = 0;
    if (current_time - last_status_time > 1000) {
      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 0);
      SSD1306_Puts("Rasp Vision Car v1", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 15);
      SSD1306_Puts("Status: Running", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 30);
      SSD1306_Puts("UPT:", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(35, 30);
      char time_str[12];
      snprintf(time_str, sizeof(time_str), "%us", (unsigned int)(get_tick_ms() / 1000));
      SSD1306_Puts(time_str, &Font_7x10, SSD1306_COLOR_WHITE);

      int16_t l_rpm = 0, r_rpm = 0;
      encoder_get_motor_speed(&l_rpm, &r_rpm);
      SSD1306_GotoXY(0, 45);
      char speed_str[18];
      snprintf(speed_str, sizeof(speed_str), "MTR: %d %d rpm", l_rpm, r_rpm);
      SSD1306_Puts(speed_str, &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();

      last_status_time = get_tick_ms();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM1) { // TIM1: 1000ms Interrupt
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1)
  {
    switch(htim->Channel)
    {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        // 10ms任务

        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        // 20ms任务
        encoder_update_10ms();
        int16_t l_rpm = 0, r_rpm = 0;
        encoder_get_motor_speed(&l_rpm, &r_rpm);

        float left_output = 0.0f, right_output = 0.0f;
        // PID_Calc(l_rpm, r_rpm, &left_output, &right_output);
        // Motor_Left_Set_Raw_Speed((int16_t)left_output);
        //Motor_Right_Set_Raw_Speed((int16_t)right_output);
        char message[100]; 
        snprintf(message, sizeof(message), "<main>:%d,%d,%d,%d,%d,%d\n", s_pid.target_left_rpm, s_pid.target_right_rpm, __HAL_TIM_GET_COUNTER(&htim2), __HAL_TIM_GET_COUNTER(&htim4), get_tick_ms(), 0);
        HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
        // Vofa_send_data(s_pid.target_left_rpm, s_pid.target_right_rpm, l_rpm, r_rpm, get_tick_ms(), 0);
        break;
        default:
          // 其他通道暂不处理
        break;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
