/*
 * STM32F407 OLED SPI Driver Configuration:
 *   Hardware Connection:
 *     OLED_SCK  -> PA5  (SPI1_SCK)
 *     OLED_MOSI -> PA7  (SPI1_MOSI)
 *     OLED_RES  -> PB0  (GPIO Output)
 *     OLED_DC   -> PB1  (GPIO Output)
 *     OLED_CS   -> PA4  (SPI1_NSS) - Optional, can be controlled by GPIO
 *
 *   CubeMX Configuration:
 *     SPI1:
 *       - Mode: Full-Duplex Master/Transmit Only Master
 *       - Data Size: 8 Bits
 *       - Clock Polarity: Low
 *       - Clock Phase: 1 Edge
 *       - Baud Rate: up to 21 MBits/s
 *     GPIO:
 *       - PB0: GPIO_Output (OLED_RES)
 *       - PB1: GPIO_Output (OLED_DC)
 */

#ifndef __OLED_HARDWARE_SPI_H
#define __OLED_HARDWARE_SPI_H

#include "oled_config.h"
#include "spi.h"

#define OLED_CMD 0  // 写命令
#define OLED_DATA 1 // 写数据

// 使用配置文件中的定义
// GPIO控制宏已在oled_config.h中定义

// OLED控制用函数
void delay_ms(uint32_t ms);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);
uint32_t oled_pow(uint8_t m, uint8_t n);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey);
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[]);
void OLED_Init(void);

#endif /* #ifndef __OLED_HARDWARE_SPI_H */
