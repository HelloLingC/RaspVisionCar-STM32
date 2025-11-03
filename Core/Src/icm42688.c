#include "main.h"
#include "icm42688.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c2;

#define ICM42688_ADDR (0x69 << 1)

uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c2, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

void icm42688_write_reg(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c2, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

bool icm42688_init(void){
    uint8_t who_am_i = icm42688_read_reg(0x75);
    if (who_am_i != 0x47)
    {
        return false;
    }
    return true;
}