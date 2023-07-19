/*!
 * @file      smtc_hal_uart.c
 *
 * @brief     Board specific package UART API implementation.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h> // bool type
#include <stdint.h>  // C99 types

#include "nrf_drv_uart.h"
#include "smtc_hal_gpio_pin_names.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_uart.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define assert_param(expr) ((void)0U) // TODO implement assert

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief UART structure
 */
typedef struct hal_uart_s {
    nrf_drv_uart_t instance;
    nrf_drv_uart_config_t config;
} hal_uart_t;

static hal_uart_t hal_uart[] = {
    [0] =
        {
            .instance = NRF_DRV_UART_INSTANCE(0),
            .config = {0},
        },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

uint8_t uart_rx_done = false;
uint8_t uart_tx_done = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief UART user event handler.
 */
void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_uart_init(const uint32_t id, const hal_gpio_pin_names_t uart_tx, const hal_gpio_pin_names_t uart_rx) {
    assert_param((id > 0) && ((id - 1) < sizeof(hal_uart)));
    uint32_t local_id = id - 1;

    hal_uart[local_id].config.hwfc = NRF_UART_HWFC_DISABLED;
    hal_uart[local_id].config.baudrate = NRF_UART_BAUDRATE_115200;
    hal_uart[local_id].config.pseltxd = uart_tx;
    hal_uart[local_id].config.pselrxd = uart_rx;

    if (nrf_drv_uart_init(&hal_uart[local_id].instance, &hal_uart[local_id].config, NULL) != NRF_SUCCESS) {
        mcu_panic();
    }
}

void hal_uart_deinit(const uint32_t id) {
    assert_param((id > 0) && ((id - 1) < sizeof(hal_uart)));
    uint32_t local_id = id - 1;

    nrf_drv_uart_uninit(&hal_uart[local_id].instance);
}

void hal_uart_tx(const uint32_t id, uint8_t *buff, uint16_t len) {

    assert_param((id > 0) && ((id - 1) < sizeof(hal_uart)));
    uint32_t local_id = id - 1;

    nrf_drv_uart_tx(&hal_uart[local_id].instance, (uint8_t *)buff, (uint8_t)len);
    NRF_UART0->EVENTS_CTS=0;
    NRF_UART0->EVENTS_TXDRDY=0;
}

void hal_uart_rx(const uint32_t id, uint8_t *rx_buffer, uint16_t len) {

    assert_param((id > 0) && ((id - 1) < sizeof(hal_uart)));
    uint32_t local_id = id - 1;

    nrf_drv_uart_rx(&hal_uart[local_id].instance, rx_buffer, (uint8_t)len);

    while (uart_rx_done != true)
        ;

    uart_rx_done = false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context) {
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE) {
        uart_rx_done = true;
    } else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE) {
        uart_tx_done = true;
    }
}

/* --- EOF ------------------------------------------------------------------ */