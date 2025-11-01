//
// Created by lingc on 11/1/2025.
//

#include <ssd1306.h>
#include <stdio.h>
#include <encoder.h>

void OLED_task_update(void) {
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("Rasp Vision Car v1", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(0, 15);
  SSD1306_Puts("Status: Running", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(0, 30);


  SSD1306_Puts("UPT:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(35, 30);
  char time_str[12];
  snprintf(time_str, sizeof(time_str), "%us", (unsigned int)(HAL_GetTick() / 1000));
  SSD1306_Puts(time_str, &Font_7x10, SSD1306_COLOR_WHITE);

  int16_t l_rpm = 0, r_rpm = 0;
  // encoder_get_motor_speed(&l_rpm, &r_rpm);
  SSD1306_GotoXY(0, 45);
  char speed_str[18];
  snprintf(speed_str, sizeof(speed_str), "MTR: %d %d rpm", l_rpm, r_rpm);
  SSD1306_Puts(speed_str, &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
}