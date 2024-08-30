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
#ifdef LISTENAI_NANOKIT
#include "cmsis_os2.h"
#include "daplink_debug.h"
#endif /* LISTENAI_NANOKIT */

// For usart
#ifdef LISTENAI_NANOKIT
#define CDC_UART1                    USART2
#define CDC_UART1_ENABLE()           __HAL_RCC_USART2_CLK_ENABLE()
#define CDC_UART1_DISABLE()          __HAL_RCC_USART2_CLK_DISABLE()
#define CDC_UART1_IRQn               USART2_IRQn

#define CDC_UART2                    USART3
#define CDC_UART2_ENABLE()           __HAL_RCC_USART3_CLK_ENABLE()
#define CDC_UART2_DISABLE()          __HAL_RCC_USART3_CLK_DISABLE()
#define CDC_UART2_IRQn               USART3_IRQn

#define UART1_PINS_PORT_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART1_PINS_PORT_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define UART2_PINS_PORT_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define UART2_PINS_PORT_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
#else /* LISTENAI_NANOKIT */
#define CDC_UART                     USART2
#define CDC_UART_ENABLE()            __HAL_RCC_USART2_CLK_ENABLE()
#define CDC_UART_DISABLE()           __HAL_RCC_USART2_CLK_DISABLE()
#define CDC_UART_IRQn                USART2_IRQn
#define CDC_UART_IRQn_Handler        USART2_IRQHandler

#define UART_PINS_PORT_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_PINS_PORT_DISABLE()     __HAL_RCC_GPIOA_CLK_DISABLE()
#endif /* LISTENAI_NANOKIT */

#define UART_TX_PORT                 GPIOA
#define UART_TX_PIN                  GPIO_PIN_2

#define UART_RX_PORT                 GPIOA
#define UART_RX_PIN                  GPIO_PIN_3

#define UART_CTS_PORT                GPIOA
#define UART_CTS_PIN                 GPIO_PIN_0

#ifdef LISTENAI_NANOKIT
#define UART_RTS_PORT                GPIOB
#define UART_RTS_PIN                 GPIO_PIN_3

#define UART_DTR_PORT                GPIOB
#define UART_DTR_PIN                 GPIO_PIN_4

#define CSK_BOOT_PORT                GPIOA
#define CSK_BOOT_PIN                 GPIO_PIN_6

#define CSK_RESET_PORT               GPIOB
#define CSK_RESET_PIN                GPIO_PIN_0

#define UART2_TX_PORT                GPIOB
#define UART2_TX_PIN                 GPIO_PIN_10

#define UART2_RX_PORT                GPIOB
#define UART2_RX_PIN                 GPIO_PIN_11
#else /* LISTENAI_NANOKIT */
#define UART_RTS_PORT                GPIOA
#define UART_RTS_PIN                 GPIO_PIN_1
#endif /* LISTENAI_NANOKIT */


#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (512)

#ifdef LISTENAI_NANOKIT
#define CTRL_DTR_BIT        (1 << 0)
#define CTRL_RTS_BIT        (1 << 1)

uint8_t update_mode = 0;
uint8_t last_dtr = 0;
uint32_t last_dtr_time = 0;

// Maximum time between DTR assertion and deassertion being considered a reset.
// The reason for this is to determine an auto reset performed by tools, aside
// from assertions from Terminal programs.
#define DTR_MAX_ASSERT_TIME_MS (750)

#define CDC_UART                     (update_mode ? CDC_UART2 : CDC_UART1)

void CDC_UART_IRQn_Handler(void);
void USART2_IRQHandler(void) { CDC_UART_IRQn_Handler(); }
void USART3_IRQHandler(void) { CDC_UART_IRQn_Handler(); }
#endif /* LISTENAI_NANOKIT */

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

#ifdef LISTENAI_NANOKIT
void uart2_enable(void)
{
    CDC_UART1->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    NVIC_DisableIRQ(CDC_UART1_IRQn);

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

    // update_mode should be 1 when this function is called
    UART_Configuration uart2_config;
    uart2_config.Baudrate = 115200;
    uart2_config.DataBits = UART_DATA_BITS_8;
    uart2_config.Parity = UART_PARITY_NONE;
    uart2_config.StopBits = UART_STOP_BITS_1;
    uart2_config.FlowControl = UART_FLOW_CONTROL_NONE;
    uart_set_configuration(&uart2_config);
}

void uart2_disable(void)
{
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    NVIC_DisableIRQ(CDC_UART2_IRQn);

    UART_HandleTypeDef uart_handle;
    memset(&uart_handle, 0, sizeof(uart_handle));
    uart_handle.Instance = CDC_UART2;
    HAL_UART_DeInit(&uart_handle);

    // Set PA15/PA18 to high-Z to avoid fucking up LCD modules
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = UART2_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART2_TX_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = UART2_RX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART2_RX_PORT, &GPIO_InitStructure);

    CDC_UART1->CR1 &= ~USART_IT_TXE;
    NVIC_EnableIRQ(CDC_UART1_IRQn);
}
#endif /* LISTENAI_NANOKIT */

int32_t uart_initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef LISTENAI_NANOKIT
    CDC_UART1->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
#else /* LISTENAI_NANOKIT */
    CDC_UART->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
#endif /* LISTENAI_NANOKIT */
    clear_buffers();

#ifdef LISTENAI_NANOKIT
    CDC_UART1_ENABLE();
    UART1_PINS_PORT_ENABLE();
    CDC_UART2_ENABLE();
    UART2_PINS_PORT_ENABLE();
#else /* LISTENAI_NANOKIT */
    CDC_UART_ENABLE();
    UART_PINS_PORT_ENABLE();
#endif /* LISTENAI_NANOKIT */

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
#ifdef LISTENAI_NANOKIT
    //RTS pin, output high
    HAL_GPIO_WritePin(UART_RTS_PORT, UART_RTS_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = UART_RTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(UART_RTS_PORT, &GPIO_InitStructure);
    //DTR pin, output high
    HAL_GPIO_WritePin(UART_DTR_PORT, UART_DTR_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = UART_DTR_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(UART_DTR_PORT, &GPIO_InitStructure);
    //CSK BOOT pin, open-drain high
    HAL_GPIO_WritePin(CSK_BOOT_PORT, CSK_BOOT_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = CSK_BOOT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CSK_BOOT_PORT, &GPIO_InitStructure);
    //CSK RESET pin, open-drain high
    HAL_GPIO_WritePin(CSK_RESET_PORT, CSK_RESET_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = CSK_RESET_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CSK_RESET_PORT, &GPIO_InitStructure);
#else /* LISTENAI_NANOKIT */
    //RTS pin, output low
    HAL_GPIO_WritePin(UART_RTS_PORT, UART_RTS_PIN, GPIO_PIN_RESET);
    GPIO_InitStructure.Pin = UART_RTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(UART_RTS_PORT, &GPIO_InitStructure);
#endif /* LISTENAI_NANOKIT */

#ifdef LISTENAI_NANOKIT
    NVIC_EnableIRQ(CDC_UART1_IRQn);
#else /* LISTENAI_NANOKIT */
    NVIC_EnableIRQ(CDC_UART_IRQn);
#endif /* LISTENAI_NANOKIT */

    return 1;
}

int32_t uart_uninitialize(void)
{
#ifdef LISTENAI_NANOKIT
    CDC_UART1->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
    CDC_UART2->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
#else /* LISTENAI_NANOKIT */
    CDC_UART->CR1 &= ~(USART_IT_TXE | USART_IT_RXNE);
#endif /* LISTENAI_NANOKIT */
    clear_buffers();
    return 1;
}

int32_t uart_reset(void)
{
#ifdef LISTENAI_NANOKIT
    const uint32_t cr1_1 = CDC_UART1->CR1;
    const uint32_t cr1_2 = CDC_UART2->CR1;
    CDC_UART1->CR1 = cr1_1 & ~(USART_IT_TXE | USART_IT_RXNE);
    CDC_UART2->CR1 = cr1_2 & ~(USART_IT_TXE | USART_IT_RXNE);
#else
    const uint32_t cr1 = CDC_UART->CR1;
    CDC_UART->CR1 = cr1 & ~(USART_IT_TXE | USART_IT_RXNE);
#endif /* LISTENAI_NANOKIT */
    clear_buffers();
#ifdef LISTENAI_NANOKIT
    CDC_UART1->CR1 = cr1_1 & ~USART_IT_TXE;
    CDC_UART2->CR1 = cr1_2 & ~USART_IT_TXE;
#else /* LISTENAI_NANOKIT */
    CDC_UART->CR1 = cr1 & ~USART_IT_TXE;
#endif /* LISTENAI_NANOKIT */
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    UART_HandleTypeDef uart_handle;
    HAL_StatusTypeDef status;

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
#ifdef LISTENAI_NANOKIT
    uint8_t rts = ctrl_bmp & CTRL_RTS_BIT;
    uint8_t dtr = ctrl_bmp & CTRL_DTR_BIT;

    debug_msg("control: 0x%04x, rts: %d, dtr: %d\n", ctrl_bmp, !!rts, !!dtr);

    // Set RTS and DTR (both active low)
    HAL_GPIO_WritePin(UART_RTS_PORT, UART_RTS_PIN, rts ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(UART_DTR_PORT, UART_DTR_PIN, dtr ? GPIO_PIN_RESET : GPIO_PIN_SET);

    uint8_t do_reset = 0;
    if (!last_dtr && dtr) {  // DTR is being asserted
        last_dtr_time = osKernelGetSysTimerCount();
    } else if (last_dtr && !dtr) {  // DTR is being deasserted
        const uint32_t now = osKernelGetSysTimerCount();
        const uint32_t ticks_per_ms = osKernelGetSysTimerFreq() / 1000;
        const int32_t diff = now - last_dtr_time;
        if (diff > 0 && diff < DTR_MAX_ASSERT_TIME_MS * ticks_per_ms) {
            do_reset = 1;
        }
        debug_msg("DTR pulse width: %d (%d ms), reset: %d, boot: %d\n",
            diff, diff / ticks_per_ms, do_reset, !!rts);
    }
    last_dtr = dtr;

    if (do_reset) {
        // Set boot pin according to current state of RTS
        HAL_GPIO_WritePin(CSK_BOOT_PORT, CSK_BOOT_PIN, rts ? GPIO_PIN_RESET : GPIO_PIN_SET);

        // Switch to correct UART
        uint8_t need_reset = 0;
        if (rts && !update_mode) {
            need_reset = 1;
            update_mode = 1;
            uart2_enable();
        } else if (!rts && update_mode) {
            need_reset = 1;
            update_mode = 0;
            uart2_disable();
        }

        if (need_reset) {
            // Reset target
            HAL_GPIO_WritePin(CSK_RESET_PORT, CSK_RESET_PIN, GPIO_PIN_RESET);
            osDelay(1);
            HAL_GPIO_WritePin(CSK_RESET_PORT, CSK_RESET_PIN, GPIO_PIN_SET);
        }
    } else if (update_mode && !rts && !dtr) {
        // Exit update mode without reset when CDC is closed
        debug_msg("Exiting update mode\n");
        HAL_GPIO_WritePin(CSK_BOOT_PORT, CSK_BOOT_PIN, GPIO_PIN_SET);
        update_mode = 0;
        uart2_disable();
    }
#endif /* LISTENAI_NANOKIT */
}

int32_t uart_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt = circ_buf_write(&write_buffer, data, size);
    CDC_UART->CR1 |= USART_IT_TXE;

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
