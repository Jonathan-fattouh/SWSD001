/*!
 * @file      sx126x_mb1cas_board.c
 *
 * @brief     Target board SX126X MB1CAS shield board board driver implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#include <stdbool.h>
#include "smtc_hal.h"
#include "smtc_board.h"
#include "ralf_sx126x.h"
#include "sx126x_hal_context.h"
#include "modem_pinout.h"
#include "smtc_shield_isp4520_common_if.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define BOARD_TX_POWER_OFFSET 0
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static sx126x_hal_context_t radio_context = {
    .nss    = SMTC_RADIO_NSS,
    .busy   = SMTC_RADIO_BUSY,
    .reset  = SMTC_RADIO_NRST,
    .spi_id = HAL_RADIO_SPI_ID,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_board_init_periph( void )
{
    smtc_shield_isp4520_init( );
}

void smtc_board_reinit_periph( void )
{
    smtc_shield_isp4520_init( );
}

void smtc_board_deinit_periph( void )
{
    smtc_shield_isp4520_deinit( );
}

ralf_t* smtc_board_initialise_and_get_ralf( void )
{
    static ralf_t local_ralf = { 0 };
    local_ralf               = ( ralf_t ) RALF_SX126X_INSTANTIATE( &radio_context );
    return &local_ralf;
}

int smtc_board_get_tx_power_offset( void )
{
    return BOARD_TX_POWER_OFFSET;
}

uint32_t smtc_board_get_tcxo_startup_time_in_ms( void )
{
    bool                        tcxo_is_radio_controlled;
    sx126x_tcxo_ctrl_voltages_t supply_voltage;
    uint32_t                    startup_time_in_tick;

    smtc_shield_isp4520_get_xosc_cfg( &tcxo_is_radio_controlled, &supply_voltage, &startup_time_in_tick );

    if( tcxo_is_radio_controlled == true )
    {
        return startup_time_in_tick * 1000 / 64000;
    }

    return 0;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
