/*
 * OLED I2C Driver Implementation
 * Hardware Connection:
 *   I2C1->PB6--SCL      PB7--SDA
 *   I2C2->PB10--SCL     PB11--SDA
 */

#include "oled.h"
#include "i2c.h"
#include "oledfont.h" // 字体文件

/* OLED初始化命令序列 */
uint8_t CMD_Data[] = {
    0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
    0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
    0xD8, 0x30, 0x8D, 0x14, 0xAF}; // 初始化命令

/**
 * @brief  写入初始化命令序列
 * @param  None
 * @retval None
 */
void WriteCmd(void)
{
    uint8_t i = 0;
    for (i = 0; i < 27; i++)
    {
        HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Data + i, 1, 0x100);
    }
}

/**
 * @brief  向OLED写入一个命令
 * @param  cmd: 命令字节
 * @retval None
 */
void OLED_WR_CMD(uint8_t cmd)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}

/**
 * @brief  向OLED写入一个数据
 * @param  data: 数据字节
 * @retval None
 */
void OLED_WR_DATA(uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}

/**
 * @brief  初始化OLED屏幕
 * @param  None
 * @retval None
 */
void OLED_Init(void)
{
    HAL_Delay(200);
    WriteCmd();
}

/**
 * @brief  清屏
 * @param  None
 * @retval None
 */
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_CMD(0xb0 + i);
        OLED_WR_CMD(0x00);
        OLED_WR_CMD(0x10);
        for (n = 0; n < 128; n++)
            OLED_WR_DATA(0);
    }
}

/**
 * @brief  开启OLED显示
 * @param  None
 * @retval None
 */
void OLED_Display_On(void)
{
    OLED_WR_CMD(0X8D); // SET DCDC命令
    OLED_WR_CMD(0X14); // DCDC ON
    OLED_WR_CMD(0XAF); // DISPLAY ON
}

/**
 * @brief  关闭OLED显示
 * @param  None
 * @retval None
 */
void OLED_Display_Off(void)
{
    OLED_WR_CMD(0X8D); // SET DCDC命令
    OLED_WR_CMD(0X10); // DCDC OFF
    OLED_WR_CMD(0XAE); // DISPLAY OFF
}

/**
 * @brief  设置光标位置
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @retval None
 */
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_CMD(0xb0 + y);
    OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10);
    OLED_WR_CMD(x & 0x0f);
}

/**
 * @brief  全屏点亮
 * @param  None
 * @retval None
 */
void OLED_On(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_CMD(0xb0 + i); // 设置页地址（0~7）
        OLED_WR_CMD(0x00);     // 设置显示位置－列低地址
        OLED_WR_CMD(0x10);     // 设置显示位置－列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_DATA(1);
    } // 更新显示
}

/**
 * @brief  计算幂函数
 * @param  m: 底数
 * @param  n: 指数
 * @retval 计算结果
 */
unsigned int oled_pow(uint8_t m, uint8_t n)
{
    unsigned int result = 1;
    while (n--)
        result *= m;
    return result;
}

/**
 * @brief  显示数字
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  num: 数值(0~4294967295)
 * @param  len: 数字的位数
 * @param  size2: 字体大小
 * @retval None
 */
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
    }
}

/**
 * @brief  在指定位置显示一个字符
 * @param  x: 横坐标 (0~127)
 * @param  y: 纵坐标 (0~63)
 * @param  chr: 要显示的字符
 * @param  Char_Size: 字符大小 (8/16)
 * @retval None
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; // 得到偏移后的值
    if (x > 128 - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WR_DATA(F8X16[c * 16 + i]);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WR_DATA(F8X16[c * 16 + i + 8]);
    }
    else
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WR_DATA(F6x8[c][i]);
    }
}

/**
 * @brief  显示一个字符串
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  chr: 字符串指针
 * @param  Char_Size: 字符大小
 * @retval None
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

/**
 * @brief  显示中文
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  no: 中文字符编号
 * @retval None
 * @note   需要在oledfont.h中定义Hzk数组
 */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t, adder = 0;
    OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_DATA(Hzk[2 * no][t]);
        adder += 1;
    }
    OLED_Set_Pos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_DATA(Hzk[2 * no + 1][t]);
        adder += 1;
    }
}

/**
 * @brief  显示图片
 * @param  x0: 起始横坐标
 * @param  y0: 起始纵坐标
 * @param  x1: 结束横坐标
 * @param  y1: 结束纵坐标
 * @param  no: 图片编号
 * @retval None
 * @note   需要在oledfont.h中定义BMP数组
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t no)
{
    uint32_t j = 0;
    uint8_t x, y;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WR_DATA(BMP[no][j++]);
        }
    }
}
