/**
 * @file uart_user.h
 * @author Shiki
 * @brief UART user buffer, to start, use HAL_UARTEx_ReceiveToIdle_DMA in user_init.c
 *        to receive data into this buffer.
 *        Remember to call Uart_DataProcess() in the main loop to process the received data.
 * @version 0.1
 * @date 2025-07-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __UART_USER_H
#define __UART_USER_H

#include "main.h"
#include "stdio.h"

#define UART_USER_BUFFER_SIZE 50

extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint8_t g_uart_command_buffer[UART_USER_BUFFER_SIZE]; // UART command buffer

// Call this function in the task scheduler to process received UART data
void Uart_DataProcess(void);

#endif
