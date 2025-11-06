#include "ahrs_hal.h"
#include "icm42688.h"
#include "rasp_comm.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

/**
 * @brief  ICM42688 I2C通信接口初始化
 */
static void icm42688_i2c_init(void) {
  // I2C已在MX_I2C2_Init()中初始化，这里无需额外操作
}

/**
 * @brief  延时函数包装器（用于适配icm42688_system_t接口）
 * @param  ms  延时毫秒数
 */
static void icm42688_delay_ms(uint16_t ms) { HAL_Delay((uint32_t)ms); }

#define ICM42688_CS_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define ICM42688_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

/**
 * @brief  读取ICM42688单个寄存器
 * @param  reg  寄存器地址
 * @return 寄存器值
 */
static uint8_t icm42688_read_reg(uint8_t reg) {
  uint8_t tx[2];
  uint8_t rx[2];
  tx[0] = reg | 0x80; // 最高位=1 表示读
  tx[1] = 0x00;

  ICM42688_CS_LOW();
  HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 1000);
  if (s != HAL_OK)
    usart_info("cannot transmit and receive for SPI2");
  ICM42688_CS_HIGH();

  return rx[1];
}

void icm42688_read_regs(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t reg_addr = reg | 0x80; // 最高位=1 表示读
  ICM42688_CS_LOW();
  HAL_StatusTypeDef s1 = HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 1000);
  HAL_StatusTypeDef s2 = HAL_SPI_Receive(&hspi2, data, len, 1000);
  if (s1 != HAL_OK || s2 != HAL_OK) {
    usart_error("SPI2 read regs failed: reg=0x%02X len=%u", reg, (unsigned)len);
  }
  ICM42688_CS_HIGH();
}

/**
 * @brief  写入ICM42688单个寄存器
 * @param  reg    寄存器地址
 * @param  value  写入的值
 */
static void icm42688_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx[2];
  tx[0] = reg & 0x7F; // 最高位=0 表示写
  tx[1] = value;
  ICM42688_CS_LOW();
  HAL_StatusTypeDef s = HAL_SPI_Transmit(&hspi2, tx, 2, 1000);
  if (s != HAL_OK) {
    usart_error("SPI2 write reg failed: reg=0x%02X val=0x%02X", reg, value);
  }
  ICM42688_CS_HIGH();
}

static uint32_t ahrs_get_time_us() { return HAL_GetTick(); }

void imu_init() {
  // IMU init
  // 使用静态变量确保生命周期持续整个程序运行期间
  static icm42688_comm_t icm42688_comm = {
      .init = icm42688_i2c_init,
      .read_reg = icm42688_read_reg,
      .write_reg = icm42688_write_reg,
      .read_regs = icm42688_read_regs,
  };
  static icm42688_system_t icm42688_system = {
      .delay_ms = icm42688_delay_ms,
  };
  static icm42688_config_t icm42688_config = {
      .interface_type = ICM42688_INTERFACE_SPI,
      .acc_sample = ICM42688_ACC_SAMPLE_SGN_2G,
      .gyro_sample = ICM42688_GYRO_SAMPLE_SGN_125DPS,
      .sample_rate = ICM42688_SAMPLE_RATE_2000,
  };
  icm42688_hal_init(&icm42688_comm, &icm42688_system, &icm42688_config);
  if (icm42688_init() != 0) {
    usart_error("ICM42688 init failed");
  }

  // 使用静态变量确保生命周期持续整个程序运行期间
  static ahrs_timer_t ahrs_timer = {.init = icm42688_i2c_init,
                                     .start = icm42688_i2c_init,
                                     .get_time_us = ahrs_get_time_us};
  static ahrs_system_t ahrs_system = {.delay_ms = icm42688_delay_ms};
  ahrs_hal_init(&ahrs_timer, &ahrs_system);
  ahrs_init();
  ahrs_init_attitude_offset();
}

void imu_update() {
    icm42688_get_acc();
    icm42688_get_gyro();
    float gx = icm42688_gyro_transition(icm42688_gyro.x) * AHRS_PI / 180.0f; // 转为弧度制
    float gy = icm42688_gyro_transition(icm42688_gyro.y) * AHRS_PI / 180.0f;
    float gz = icm42688_gyro_transition(icm42688_gyro.z) * AHRS_PI / 180.0f;
    
    float ax = icm42688_acc_transition(icm42688_acc.x);
    float ay = icm42688_acc_transition(icm42688_acc.y);
    float az = icm42688_acc_transition(icm42688_acc.z);

    // ahrs_update(gx, gy, gz, ax, ay, az, 0, 0, 0);
    // ahrs_euler_angle_t current_attitude;
    // ahrs_get_attitude(&current_attitude);

    char message[100];
    snprintf(message, sizeof(message), "<main%u>:%d,%d,%d\n", HAL_GetTick(),
            gx, gy, gz);
    HAL_UART_Transmit_IT(&huart1, message, strlen(message));
    // char message[100];
    // snprintf(message, sizeof(message), "<main%u>:%d,%d,%d\n", HAL_GetTick(),
    //         round(current_attitude.pitch), round(current_attitude.roll), round(current_attitude.yaw));
    // HAL_UART_Transmit_IT(&huart1, message, strlen(message));
}