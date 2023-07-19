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

#include "app_timer.h"
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
APP_TIMER_DEF(lp_timer);
static hal_lp_timer_irq_t lptim_tmr_irq = {.context = NULL, .callback = NULL};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**@brief Function for handling the lp_timer timeout.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void lp_timer_handler(void *p_context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init(void) {
    // Initialize timer module.
    if ((app_timer_init() != NRF_SUCCESS)) {
        mcu_panic();
    }
    // Create timers.
    if (app_timer_create(&lp_timer, APP_TIMER_MODE_SINGLE_SHOT, lp_timer_handler) != NRF_SUCCESS) {
        mcu_panic();
    }
    lptim_tmr_irq = (hal_lp_timer_irq_t){.context = NULL, .callback = NULL};
}

void hal_lp_timer_start(const uint32_t milliseconds, const hal_lp_timer_irq_t *tmr_irq) {
    app_timer_stop(lp_timer);
    app_timer_start(lp_timer, APP_TIMER_TICKS(milliseconds), NULL);

    lptim_tmr_irq = *tmr_irq;
}

void hal_lp_timer_stop(void) {
     app_timer_stop(lp_timer);
}

void hal_lp_timer_irq_enable(void) {
    NVIC_EnableIRQ(RTC0_IRQn);
}

void hal_lp_timer_irq_disable(void) {
    NVIC_DisableIRQ(RTC0_IRQn);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lp_timer_handler(void *p_context) {
    if (lptim_tmr_irq.callback != NULL) {
        lptim_tmr_irq.callback(lptim_tmr_irq.context);
    }
}

/* --- EOF ------------------------------------------------------------------ */