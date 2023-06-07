/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
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

#include "nrf_drv_timer.h"
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

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
const nrf_drv_timer_t TIMER_INSTANCE = NRF_DRV_TIMER_INSTANCE(0);
static hal_lp_timer_irq_t lptim_tmr_irq = {.context = NULL, .callback = NULL};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void *p_context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init(void) {
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    if (nrf_drv_timer_init(&TIMER_INSTANCE, &timer_cfg, timer_event_handler)) {
        mcu_panic();
    }

    lptim_tmr_irq = (hal_lp_timer_irq_t){.context = NULL, .callback = NULL};
}

void hal_lp_timer_start(const uint32_t milliseconds, const hal_lp_timer_irq_t *tmr_irq) {
    uint32_t time_ticks;

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_INSTANCE, milliseconds);

    nrf_drv_timer_compare(&TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, time_ticks, true);

    nrf_drv_timer_enable(&TIMER_INSTANCE);

    lptim_tmr_irq = *tmr_irq;
}

void hal_lp_timer_stop(void) {
    nrf_drv_timer_disable(&TIMER_INSTANCE);
}

void hal_lp_timer_irq_enable(void) {
    NVIC_EnableIRQ(TIMER0_IRQn);
}

void hal_lp_timer_irq_disable(void) {
    NVIC_DisableIRQ(TIMER0_IRQn);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void timer_event_handler(nrf_timer_event_t event_type, void *p_context) {
    static uint32_t i;

    switch (event_type) {
    case NRF_TIMER_EVENT_COMPARE0:
        if (lptim_tmr_irq.callback != NULL) {
            lptim_tmr_irq.callback(lptim_tmr_irq.context);
        }
        break;

    default:
        // Do nothing.
        break;
    }
}

/* --- EOF ------------------------------------------------------------------ */