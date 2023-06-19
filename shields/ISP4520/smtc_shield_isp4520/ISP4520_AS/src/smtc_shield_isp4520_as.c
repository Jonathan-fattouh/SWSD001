/*!
 * @file      smtc_shield_sx1261mb1cas.c
 *
 * @brief     Implementation specific to SX1261MB1CAS shield
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

#include <stddef.h>
#include "smtc_shield_isp4520_as.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define ISP4520_AS_SUBGHZ_FREQ_MIN 150000000
#define ISP4520_AS_SUBGHZ_FREQ_MAX 960000000

#define ISP4520_AS_MIN_PWR -17
#define ISP4520_AS_MAX_PWR 15

// PA config table
const smtc_shield_sx126x_pa_pwr_cfg_t pa_cfg_table[ISP4520_AS_MAX_PWR - ISP4520_AS_MIN_PWR + 1] = {
    { // Expected output power = -17dBm
        .power = -15,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -16dBm
        .power = -14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -15dBm
        .power = -15,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -14dBm
        .power = -11,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -13dBm
        .power = -13,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -12dBm
        .power = -9,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -11dBm
        .power = -8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -10dBm
        .power = -7,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -9dBm
        .power = -8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -8dBm
        .power = -7,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -7dBm
        .power = -6,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -6dBm
        .power = -4,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -5dBm
        .power = -3,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -4dBm
        .power = -2,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -3dBm
        .power = -1,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -2dBm
        .power = 1,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -1dBm
        .power = 1,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 0dBm
        .power = 2,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 1dBm
        .power = 5,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 2dBm
        .power = 6,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 3dBm
        .power = 7,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 4dBm
        .power = 8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 5dBm
        .power = 8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 6dBm
        .power = 9,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 7dBm
        .power = 12,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 8dBm
        .power = 13,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 9dBm
        .power = 13,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 10dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 11dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 12dBm
        .power = 12,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x05,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 13dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 14dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x04,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 15dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x05,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

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

const smtc_shield_sx126x_pa_pwr_cfg_t* smtc_shield_sx126x_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                                          int8_t         expected_output_pwr_in_dbm )
{
    if( ( ISP4520_AS_SUBGHZ_FREQ_MIN <= rf_freq_in_hz ) && ( rf_freq_in_hz <= ISP4520_AS_SUBGHZ_FREQ_MAX ) )
    {
        if( ( ISP4520_AS_MIN_PWR <= expected_output_pwr_in_dbm ) &&
            ( expected_output_pwr_in_dbm <= ISP4520_AS_MAX_PWR ) )
        {
            return &( pa_cfg_table[expected_output_pwr_in_dbm - ISP4520_AS_MIN_PWR] );
        }
    }

    return NULL;
}

bool smtc_shield_sx126x_is_dio2_set_as_rf_switch( void )
{
    return true;
}

sx126x_reg_mod_t smtc_shield_sx126x_get_reg_mode( void )
{
    return SX126X_REG_MODE_DCDC;
}

void smtc_shield_sx126x_get_xosc_cfg( bool* tcxo_is_radio_controlled, sx126x_tcxo_ctrl_voltages_t* supply_voltage,
                                      uint32_t* startup_time_in_tick )
{
    *tcxo_is_radio_controlled = true;
    *supply_voltage           = SX126X_TCXO_CTRL_1_8V;
    *startup_time_in_tick     = 128;//300;
}

void smtc_shield_sx126x_get_ocp_value( uint8_t* ocp_in_step_of_2_5_ma )
{
    *ocp_in_step_of_2_5_ma = 0x38;
}

void smtc_shield_sx126x_init( void )
{
    smtc_shield_sx126x_init_led_tx( );
    smtc_shield_sx126x_init_led_rx( );

    smtc_shield_sx126x_reset_led_rx( );
    smtc_shield_sx126x_reset_led_tx( );
}

void smtc_shield_sx126x_deinit( void )
{
    smtc_shield_sx126x_deinit_led_tx( );
    smtc_shield_sx126x_deinit_led_rx( );
}

void smtc_shield_sx126x_handle_pre_rx( void )
{
    smtc_shield_sx126x_set_led_rx( );
}

void smtc_shield_sx126x_handle_post_rx( void )
{
    smtc_shield_sx126x_reset_led_rx( );
}

void smtc_shield_sx126x_handle_pre_tx( void )
{
    smtc_shield_sx126x_set_led_tx( );
}

void smtc_shield_sx126x_handle_post_tx( void )
{
    smtc_shield_sx126x_reset_led_tx( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
