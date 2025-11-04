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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "rasp_comm.h"
#include "justfloat.h"
#include "ssd1306.h"
#include "feedforward_controller.h"
#include "encoder.h"
#include "pid_controller.h"
#include "buzzer.h"
#include "icm42688.h"
#include "ahrs_hal.h"

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
volatile uint8_t system_stop_flag = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t get_tick_ms(void)
{
    // Get CPU cycles and convert to milliseconds
    uint32_t cycles = DWT->CYCCNT;
    return cycles / (SystemCoreClock / 1000);
}

void imu_init();
void imu_update();
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Enable DWT (Data Watchpoint and Trace) unit
  // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // DWT->CYCCNT = 0;
  // DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  SSD1306_Init();
  rasp_comm_init();

  HAL_Delay(1000); // delay for icm42688 starting
  imu_init();
  HAL_Delay(1000);

  init_encoders();
  Motor_Init();
  ff_init_default();
  pid_init_default();

  motor_controller_set_turn_angle(0);
  motor_controller_set_target_rpm(0, 0);

  HAL_TIM_Base_Start_IT(&htim1);
  // 启动比较中断
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

  usart_info("========================================");
  usart_info("Rasp Vision Car v1");
  usart_info("========================================");
  buzzer_on(3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // Update OLED display
    uint32_t current_time = HAL_GetTick();
    static uint32_t last_status_time = 0;
    if (current_time - last_status_time > 1000) {
      SSD1306_Fill(SSD1306_COLOR_BLACK);
      SSD1306_GotoXY(0, 0);
      SSD1306_Puts("Rasp Vision Car v1", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(0, 15);
      SSD1306_Puts("UPT:", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(35, 15);
      char time_str[12];
      snprintf(time_str, sizeof(time_str), "%us", (unsigned int)(HAL_GetTick() / 1000));
      SSD1306_Puts(time_str, &Font_7x10, SSD1306_COLOR_WHITE);

      char targets_str[18];
      int16_t real_target_left_rpm = 0, real_target_right_rpm = 0;
      motor_controller_get_real_target_rpm(&real_target_left_rpm, &real_target_right_rpm);
      int16_t turn_angle = motor_controller_get_turn_angle();
      snprintf(targets_str, sizeof(targets_str), "V: %d, %d T: %d", real_target_left_rpm, real_target_right_rpm, turn_angle);
      SSD1306_GotoXY(0, 30);
      SSD1306_Puts(targets_str, &Font_7x10, SSD1306_COLOR_WHITE);

      int16_t l_rpm = 0, r_rpm = 0;
      encoder_get_motor_speed(&l_rpm, &r_rpm);
      char speed_str[18];
      snprintf(speed_str, sizeof(speed_str), "MTR: %d %d rpm", l_rpm, r_rpm);
      SSD1306_GotoXY(0, 45);
      SSD1306_Puts(speed_str, &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen();

      last_status_time = HAL_GetTick();
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
        buzzer_update();
        encoder_update_10ms();
        motor_controller_update();

       
        // char message[100];
        // snprintf(message, sizeof(message), "<main%u>:%d,%d,%d\n", HAL_GetTick(),
        //         round(current_attitude.pitch), round(current_attitude.roll), round(current_attitude.yaw));
        // HAL_UART_Transmit_IT(&huart1, message, strlen(message));

        // 重新设置下一个20ms触发点（200计数 = 20ms）
        uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
        uint32_t next_compare = (current_count + 200) % 10000;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, next_compare);
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
  usart_error("Error: %s", __FUNCTION__);
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
