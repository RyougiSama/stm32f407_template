/**
 * @file command.h
 * @author Shiki
 * @brief UART Command, protocol format is 0xAA + length + data + checksum
 * @version 0.1
 * @date 2025-07-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __COMMAND_H
#define __COMMAND_H

#include "main.h"
#include <string.h>

uint8_t Command_Write(uint8_t *data, uint8_t length);
uint8_t Command_GetCommand(uint8_t *command);

#endif