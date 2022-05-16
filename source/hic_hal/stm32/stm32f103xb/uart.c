/**
 * @file    uart.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "string.h"

#include "stm32f1xx.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"

// For usart
#define CDC_UART                     USART2
#define CDC_UART_ENABLE()            __HAL_RCC_USART2_CLK_ENABLE()
#define CDC_UART_DISABLE()           __HAL_RCC_USART2_CLK_DISABLE()
#define CDC_UART_IRQn                USART2_IRQn
#define CDC_UART_IRQn_Handler        USART2_IRQHandler

#define UART_PINS_PORT_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_PINS_PORT_DISABLE()     __HAL_RCC_GPIOA_CLK_DISABLE()

// For CSK update
#define CDC_UART2                    USART3
#define CDC_UART2_ENABLE()           __HAL_RCC_USART3_CLK_ENABLE()
#define CDC_UART2_DISABLE()          __HAL_RCC_USART3_CLK_DISABLE()
#define CDC_UART2_IRQn               USART3_IRQn
#define CDC_UART2_IRQn_Handler       USART3_IRQHandler

#define UART2_PINS_PORT_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define UART2_PINS_PORT_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()

#define UART_TX_PORT                 GPIOA
#define UART_TX_PIN                  GPIO_PIN_2

#define UART_RX_PORT                 GPIOA
#define UART_RX_PIN                  GPIO_PIN_3

#define UART_CTS_PORT                GPIOA
#define UART_CTS_PIN                 GPIO_PIN_0

#define UART_RTS_PORT                GPIOA
#define UART_RTS_PIN                 GPIO_PIN_1

// For CSK update
#define CSK_BOOT_PORT                GPIOA
#define CSK_BOOT_PIN                 GPIO_PIN_6

#define CSK_RESET_PORT               GPIOB
#define CSK_RESET_PIN                GPIO_PIN_0

#define UART2_TX_PORT                GPIOB
#define UART2_TX_PIN                 GPIO_PIN_10

#define UART2_RX_PORT                GPIOB
#define UART2_RX_PIN                 GPIO_PIN_11


#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (512)

#define CTRL_DTR_BIT        (1 << 0)
#define CTRL_RTS_BIT        (1 << 1)

uint8_t update_mode = 0;

circ_buf_t write_buffer;
uint8_t write_buffer_data[BUFFER_SIZE];
circ_buf_t read_buffer;
uint8_t read_buffer_data[BUFFER_SIZE];

static UART_Configuration configuration = {
    .Baudrate = 9600,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};

extern uint32_t SystemCoreClock;



static void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

void uart2_set_configuration(uint32_t baud_rate)
{
    UART_HandleTypeDef uart_handle;

    memset(&uart_handle, 0, sizeof(uart_handle));
    uart_handle.Instance = CDC_UART2;
    uart_handle.Init.Parity = HAL_UART_PARITY_NONE;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    uart_handle.Init.BaudRate = baud_rate;
    uart_handle.Init.Mode = UART_MODE_TX_RX;

    // Disable uart and tx/rx interrupt
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);

    HAL_UART_DeInit(&uart_handle);
    HAL_UART_Init(&uart_handle);

    CDC_UART2->CR1 |= USART_IT_RXNE;
}

void uart2_enable(void)
{
    CDC_UART->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    NVIC_DisableIRQ(CDC_UART_IRQn);

    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = UART2_TX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART2_TX_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = UART2_RX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART2_RX_PORT, &GPIO_InitStructure);

    NVIC_EnableIRQ(CDC_UART2_IRQn);

    uart2_set_configuration(115200);
}

void uart2_disable(void)
{
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    NVIC_DisableIRQ(CDC_UART2_IRQn);

    UART_HandleTypeDef uart_handle;
    memset(&uart_handle, 0, sizeof(uart_handle));
    uart_handle.Instance = CDC_UART2;
    HAL_UART_DeInit(&uart_handle);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = UART2_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART2_TX_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(UART2_TX_PORT, UART2_TX_PIN, GPIO_PIN_RESET);
    GPIO_InitStructure.Pin = UART2_RX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART2_RX_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(UART2_RX_PORT, UART2_RX_PIN, GPIO_PIN_RESET);

    CDC_UART->CR1 &= ~USART_IT_TXE;
    NVIC_EnableIRQ(CDC_UART_IRQn);
}

int32_t uart_initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    CDC_UART->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    clear_buffers();

    CDC_UART_ENABLE();
    UART_PINS_PORT_ENABLE();
    CDC_UART2_ENABLE();
    UART2_PINS_PORT_ENABLE();

    //TX pin
    GPIO_InitStructure.Pin = UART_TX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStructure);
    //RX pin
    GPIO_InitStructure.Pin = UART_RX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_RX_PORT, &GPIO_InitStructure);
    //CTS pin, input
    GPIO_InitStructure.Pin = UART_CTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_CTS_PORT, &GPIO_InitStructure);
    //RTS pin, output low
    HAL_GPIO_WritePin(UART_RTS_PORT, UART_RTS_PIN, GPIO_PIN_RESET);
    GPIO_InitStructure.Pin = UART_RTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(UART_RTS_PORT, &GPIO_InitStructure);

    // CSK BOOT pin, open-drain low
    HAL_GPIO_WritePin(CSK_BOOT_PORT, CSK_BOOT_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = CSK_BOOT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CSK_BOOT_PORT, &GPIO_InitStructure);
    // CSK RESET pin, open-drain low
    HAL_GPIO_WritePin(CSK_RESET_PORT, CSK_RESET_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = CSK_RESET_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CSK_RESET_PORT, &GPIO_InitStructure);

    NVIC_EnableIRQ(CDC_UART_IRQn);

    return 1;
}

int32_t uart_uninitialize(void)
{
    CDC_UART->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    clear_buffers();
    return 1;
}

int32_t uart_reset(void)
{
    const uint32_t cr1 = CDC_UART->CR1;
    const uint32_t cr21 = CDC_UART2->CR1;
    CDC_UART->CR1 = cr1 & ~(USART_IT_TXE | USART_IT_RXNE);
    CDC_UART2->CR1 = cr21 & ~(USART_IT_TXE | USART_IT_RXNE);
    clear_buffers();
    CDC_UART->CR1 = cr1 & ~USART_IT_TXE;
    CDC_UART2->CR1 = cr21 & ~USART_IT_TXE;
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    UART_HandleTypeDef uart_handle;
    HAL_StatusTypeDef status;

    if (update_mode) {
        uart2_set_configuration(config->Baudrate);
        return 1;
    }

    memset(&uart_handle, 0, sizeof(uart_handle));
    uart_handle.Instance = CDC_UART;

    // parity
    configuration.Parity = config->Parity;
    if(config->Parity == UART_PARITY_ODD) {
        uart_handle.Init.Parity = HAL_UART_PARITY_ODD;
    } else if(config->Parity == UART_PARITY_EVEN) {
        uart_handle.Init.Parity = HAL_UART_PARITY_EVEN;
    } else if(config->Parity == UART_PARITY_NONE) {
        uart_handle.Init.Parity = HAL_UART_PARITY_NONE;
    } else {   //Other not support
        uart_handle.Init.Parity = HAL_UART_PARITY_NONE;
        configuration.Parity = UART_PARITY_NONE;
    }

    // stop bits
    configuration.StopBits = config->StopBits;
    if(config->StopBits == UART_STOP_BITS_2) {
        uart_handle.Init.StopBits = UART_STOPBITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1_5) {
        uart_handle.Init.StopBits = UART_STOPBITS_2;
        configuration.StopBits = UART_STOP_BITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1) {
        uart_handle.Init.StopBits = UART_STOPBITS_1;
    } else {
        uart_handle.Init.StopBits = UART_STOPBITS_1;
        configuration.StopBits = UART_STOP_BITS_1;
    }

    //Only 8 bit support
    configuration.DataBits = UART_DATA_BITS_8;
    if (uart_handle.Init.Parity == HAL_UART_PARITY_ODD || uart_handle.Init.Parity == HAL_UART_PARITY_EVEN) {
        uart_handle.Init.WordLength = UART_WORDLENGTH_9B;
    } else {
        uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    }

    // No flow control
    configuration.FlowControl = UART_FLOW_CONTROL_NONE;
    uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    
    // Specified baudrate
    configuration.Baudrate = config->Baudrate;
    uart_handle.Init.BaudRate = config->Baudrate;

    // TX and RX
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    
    // Disable uart and tx/rx interrupt
    CDC_UART->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);

    clear_buffers();

    status = HAL_UART_DeInit(&uart_handle);
    util_assert(HAL_OK == status);
    status = HAL_UART_Init(&uart_handle);
    util_assert(HAL_OK == status);
    (void)status;

    CDC_UART->CR1 |= USART_IT_RXNE;

    return 1;
}

int32_t uart_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    config->FlowControl = UART_FLOW_CONTROL_NONE;

    return 1;
}

void uart_set_control_line_state(uint16_t ctrl_bmp)
{
    // Some host may pull both RTS and DTR down by default. Thus only trust the
    // level when RTS and DTR are different.
    uint8_t reset = (ctrl_bmp & (CTRL_DTR_BIT | CTRL_RTS_BIT)) == CTRL_RTS_BIT;
    uint8_t boot = (ctrl_bmp & (CTRL_DTR_BIT | CTRL_RTS_BIT)) == CTRL_DTR_BIT;
    HAL_GPIO_WritePin(CSK_RESET_PORT, CSK_RESET_PIN, reset ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(CSK_BOOT_PORT, CSK_BOOT_PIN, boot ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (update_mode && !boot) {
        uart2_disable();
    } else if (!update_mode && boot) {
        uart2_enable();
    }
    update_mode = boot;
}

int32_t uart_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt = circ_buf_write(&write_buffer, data, size);
    if (update_mode) {
        CDC_UART2->CR1 |= USART_IT_TXE;
    } else {
        CDC_UART->CR1 |= USART_IT_TXE;
    }

    return cnt;
}

int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&read_buffer, data, size);
}

void CDC_UART_IRQn_Handler(void)
{
    const uint32_t sr = CDC_UART->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t dat = CDC_UART->DR;
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            // Drop character
        }
    }

    if (sr & USART_SR_TXE) {
        if (circ_buf_count_used(&write_buffer) > 0) {
            CDC_UART->DR = circ_buf_pop(&write_buffer);
        } else {
            CDC_UART->CR1 &= ~USART_IT_TXE;
        }
    }
}

void CDC_UART2_IRQn_Handler(void)
{
    const uint32_t sr = CDC_UART2->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t dat = CDC_UART2->DR;
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            // Drop character
        }
    }

    if (sr & USART_SR_TXE) {
        if (circ_buf_count_used(&write_buffer) > 0) {
            CDC_UART2->DR = circ_buf_pop(&write_buffer);
        } else {
            CDC_UART2->CR1 &= ~USART_IT_TXE;
        }
    }
}
