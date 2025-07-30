/*
 * STM32F407 OLED I2C Driver Configuration:
 *   Hardware Connection:
 *     OLED_SCL  -> PB6  (I2C1_SCL)
 *     OLED_SDA  -> PB7  (I2C1_SDA)
 *     VCC       -> 3.3V
 *     GND       -> GND
 *
 *   Alternative Connection (I2C2):
 *     OLED_SCL  -> PB10 (I2C2_SCL)
 *     OLED_SDA  -> PB11 (I2C2_SDA)
 *
 *   CubeMX Configuration:
 *     I2C1:
 *       - Mode: I2C
 *       - Speed: Standard Mode (100 KHz)
 *       - Addressing Mode: 7-bit
 *       - Clock Stretching: Enabled
 *       - General Call: Disabled
 *       - No Stretch: Disabled
 *
 *   OLED Device Address: 0x78 (7-bit address: 0x3C)
 */

#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

/* GPIO时钟使能 */
#define   OLED_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()

/* GPIO引脚定义 */
#define   GPIOx_OLED_PORT               GPIOB
#define   OLED_SCK_PIN                  GPIO_PIN_6    // I2C1对应GPIO_PIN_6，I2C2对应GPIO_PIN_10
#define   OLED_SCK_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_SET)
#define   OLED_SCK_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_RESET)
#define   OLED_SCK_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SCK_PIN)
#define   OLED_SDA_PIN                  GPIO_PIN_7    // I2C1对应GPIO_PIN_7，I2C2对应GPIO_PIN_11
#define   OLED_SDA_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define   OLED_SDA_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define   OLED_SDA_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SDA_PIN)

/* OLED控制函数声明 */
void WriteCmd(void);
void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_On(void);
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t no);

#endif

