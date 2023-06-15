/*!
 * \file      smtc_hal_spi.c
 *
 * \brief     SPI Hardware Abstraction Layer implementation
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

#include "modem_pinout.h"
#include "nrf_drv_spi.h"
#include "smtc_hal.h"

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
 *  @brief SPI structure
 */
typedef struct hal_spi_s {
    nrf_drv_spi_t instance;
    nrf_drv_spi_config_t config;
} hal_spi_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
// TODO do we need more than 1 instance?
static hal_spi_t hal_spi[] = {
    [0] =
        {
            .instance = NRF_DRV_SPI_INSTANCE(2),
            .config = {0},
        },
};
static volatile bool spi_xfer_done; /**< Flag used to indicate that SPI instance completed the transfer. */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief SPI user event handler.
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_spi_init(const uint32_t id, const hal_gpio_pin_names_t mosi, const hal_gpio_pin_names_t miso,
    const hal_gpio_pin_names_t sclk) {

    assert_param((id > 0) && ((id - 1) < sizeof(hal_spi)));
    uint32_t local_id = id - 1;

    hal_spi[local_id].config.frequency = NRF_DRV_SPI_FREQ_8M;
    hal_spi[local_id].config.mode = NRF_DRV_SPI_MODE_0;
    hal_spi[local_id].config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    hal_spi[local_id].config.orc = 0xFF;
    hal_spi[local_id].config.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    hal_spi[local_id].config.sck_pin = sclk;
    hal_spi[local_id].config.mosi_pin = mosi;
    hal_spi[local_id].config.miso_pin = miso;
    hal_spi[local_id].config.ss_pin = NRFX_SPI_PIN_NOT_USED;

    if (nrf_drv_spi_init(&hal_spi[local_id].instance, &hal_spi[local_id].config, NULL, NULL) != NRF_SUCCESS) {
        mcu_panic();
    }
}

void hal_spi_deinit(const uint32_t id) {
    assert_param((id > 0) && ((id - 1) < sizeof(hal_spi)));
    uint32_t local_id = id - 1;

    nrf_drv_spi_uninit(&hal_spi[local_id].instance);
}

uint16_t hal_spi_in_out(const uint32_t id, const uint16_t out_data) {
    assert_param((id > 0) && ((id - 1) < sizeof(hal_spi)));
    uint32_t local_id = id - 1;
    uint8_t in_data;

    if (nrf_drv_spi_transfer(&hal_spi[local_id].instance, (uint8_t *)&out_data, 1, (uint8_t *)&in_data, 1)) {
        mcu_panic();
    }

    if (NRF_SPIM2->EVENTS_ENDRX)
        NRF_SPIM2->EVENTS_ENDRX = 0;
    if (NRF_SPIM2->EVENTS_ENDTX)
        NRF_SPIM2->EVENTS_ENDTX = 0;
    if (NRF_SPIM2->EVENTS_END)
        NRF_SPIM2->EVENTS_END = 0;

    return (uint16_t)in_data;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context) {
    spi_xfer_done = true;
}

/* --- EOF ------------------------------------------------------------------ */