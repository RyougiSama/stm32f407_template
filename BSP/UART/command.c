#include "command.h"

// 指令的最小长度，修改该值以适配不同协议格式的长度
#define COMMAND_MIN_LENGTH 4
// 循环缓冲区大小，增大该值以降低缓冲区溢出的概率
#define BUFFER_SIZE 220
// 循环缓冲区
static uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
static uint8_t read_index = 0;
// 循环缓冲区写索引
static uint8_t write_index = 0;

/**
 * @brief 增加读索引
 * @param length 要增加的长度
 */
static void Command_AddReadIndex(uint8_t length)
{
    read_index += length;
    read_index %= BUFFER_SIZE;
}

/**
 * @brief 读取第i位数据 超过缓存区长度自动循环
 * @param i 要读取的数据索引
 */
static uint8_t Command_Read(uint8_t i)
{
    uint8_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/**
 * @brief 计算未处理的数据长度
 * @return 未处理的数据长度
 * @retval 0 缓冲区为空
 * @retval 1~BUFFER_SIZE-1 未处理的数据长度
 * @retval BUFFER_SIZE 缓冲区已满
 */
static uint8_t Command_GetLength()
{
    // 读索引等于写索引时，缓冲区为空
    if (read_index == write_index) {
        return 0;
    }
    // 如果缓冲区已满,返回BUFFER_SIZE
    if ((write_index + 1) % BUFFER_SIZE == read_index) {
        return BUFFER_SIZE - 1;  // 实际可用空间比BUFFER_SIZE少1，避免满空无法区分
    }
    // 如果缓冲区未满,返回未处理的数据长度
    if (read_index <= write_index) {
        return write_index - read_index;
    } else {
        return BUFFER_SIZE - read_index + write_index;
    }
}
// uint8_t Command_GetLength() {
//   // 读索引等于写索引时，缓冲区为空
//   if (read_index == write_index) {
//     return 0;
//   }
//   // 如果缓冲区已满,返回BUFFER_SIZE
//   if (write_index + 1 == read_index || (write_index == BUFFER_SIZE - 1 && read_index == 0)) {
//     return BUFFER_SIZE;
//   }
//   // 如果缓冲区未满,返回未处理的数据长度
//   if (read_index <= write_index) {
//     return write_index - read_index;
//   } else {
//     return BUFFER_SIZE - read_index + write_index;
//   }
// }

/**
 * @brief 计算缓冲区剩余空间
 * @return 剩余空间
 * @retval 0 缓冲区已满
 * @retval 1~BUFFER_SIZE-1 剩余空间
 * @retval BUFFER_SIZE 缓冲区为空
 */
static uint8_t Command_GetRemain()
{
    return BUFFER_SIZE - Command_GetLength();
}

/**
 * @brief 向缓冲区写入数据
 * @param data 要写入的数据指针
 * @param length 要写入的数据长度
 * @return 写入的数据长度
 */
uint8_t Command_Write(uint8_t *data, uint8_t length)
{
    // 如果缓冲区不足 则不写入数据 返回0
    if (Command_GetRemain() < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (write_index + length < BUFFER_SIZE) {
        memcpy(buffer + write_index, data, length);
        write_index += length;
    } else {
        uint8_t first_length = BUFFER_SIZE - write_index;
        memcpy(buffer + write_index, data, first_length);
        memcpy(buffer, data + first_length, length - first_length);
        write_index = length - first_length;
    }
    return length;
}

/**
 * @brief 尝试获取一条指令，重写该函数以适配指定协议格式
 * @param command 指令存放指针
 * @return 获取的指令长度
 * @retval 0 没有获取到指令
 */
uint8_t Command_GetCommand(uint8_t *command)
{
    // 寻找完整指令
    while (1) {
        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (Command_GetLength() < COMMAND_MIN_LENGTH) {
            return 0;
        }
        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(read_index) != 0xAA) {
            Command_AddReadIndex(1);
            continue;
        }
        // 如果缓冲区长度小于指令长度 则不可能有完整的指令
        uint8_t length = Command_Read(read_index + 1);
        if (Command_GetLength() < length) {
            return 0;
        }
        // 如果校验和不正确 则跳过 重新开始寻找
        uint8_t sum = 0;
        for (uint8_t i = 0; i < length - 1; i++) {
            sum += Command_Read(read_index + i);
        }
        if (sum != Command_Read(read_index + length - 1)) {
            Command_AddReadIndex(1);
            continue;
        }
        // 如果找到完整指令 则将指令写入command 返回指令长度
        for (uint8_t i = 0; i < length; i++) {
            command[i] = Command_Read(read_index + i);
        }
        Command_AddReadIndex(length);
        return length;
    }
}
