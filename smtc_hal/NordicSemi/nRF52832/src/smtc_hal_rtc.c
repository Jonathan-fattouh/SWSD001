/*!
 * @file      smtc_hal_rtc.c
 *
 * @brief     RTC Hardware Abstraction Layer implementation
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

#include <math.h>
#include <time.h>

#include "app_timer.h"
#include "nrf_delay.h"
#include "smtc_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * Calculates ceiling( X / N )
 */
#define DIVC(X, N) (((X) + (N)-1) / (N))

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* MCU Wake Up Time */
#define MIN_ALARM_DELAY_IN_TICKS 3U // in ticks


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief RTC timer context
 */
typedef struct
{
    uint32_t time_ref_in_ticks; // Reference time
} rtc_context_t;

/*!
 * @brief RTC structure
 */
typedef struct hal_rtc_s {
    /*!
     * Keep the value of the RTC timer when the RTC alarm is set
     * Set with the \ref hal_rtc_set_context function
     * Value is kept as a Reference to calculate alarm
     */
    rtc_context_t context;
} hal_rtc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

hal_rtc_t hal_rtc = {
    .context = {0},
};

static volatile bool wut_timer_irq_happened = false;
APP_TIMER_DEF(alarm_timer);
APP_TIMER_DEF(wakeup_timer);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Get current full resolution RTC timestamp in ticks
 *
 * @returns timestamp_in_ticks Current timestamp in ticks
 */
static uint64_t rtc_get_timestamp_in_ticks(void);

/**@brief Function for handling the alarm timer timeout.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void alarm_timeout_handler(void *p_context);

/**@brief Function for handling the wakeup timer timeout.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void wakeup_timeout_handler(void *p_context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init(void) {
    // Initialize timer module.
    if ((app_timer_init() != NRF_SUCCESS)) {
        mcu_panic();
    }
    // Create timers.
    if (app_timer_create(&alarm_timer, APP_TIMER_MODE_SINGLE_SHOT, alarm_timeout_handler) != NRF_SUCCESS) {
        mcu_panic();
    }
    if (app_timer_create(&wakeup_timer, APP_TIMER_MODE_SINGLE_SHOT, wakeup_timeout_handler) != NRF_SUCCESS) {
        mcu_panic();
    }

    hal_rtc_set_time_ref_in_ticks();
}

uint32_t hal_rtc_get_time_s(void) {
    uint32_t seconds;
    uint32_t ticks;

    ticks = hal_rtc_get_timer_value();

    uint64_t tmp=ticks*(APP_TIMER_CONFIG_RTC_FREQUENCY + 1);
    seconds = tmp/(uint64_t)APP_TIMER_CLOCK_FREQ;

    return seconds;
}

uint32_t hal_rtc_get_time_100us(void) {
    uint32_t ticks;
    uint32_t val_100us;

    ticks = hal_rtc_get_timer_value();

    uint64_t tmp=10000*ticks*(APP_TIMER_CONFIG_RTC_FREQUENCY + 1);
    val_100us = tmp/(uint64_t)APP_TIMER_CLOCK_FREQ;

    return val_100us;
}

uint32_t hal_rtc_get_time_ms(void) {
    uint32_t milliseconds;
    uint32_t ticks;

    ticks = hal_rtc_get_timer_value();

    uint64_t tmp=1000*ticks*(APP_TIMER_CONFIG_RTC_FREQUENCY + 1);
    milliseconds = tmp/(uint64_t)APP_TIMER_CLOCK_FREQ;

    return milliseconds;
}

void hal_rtc_stop_alarm(void) {
    app_timer_stop(alarm_timer);
}

/*!
 * @brief Sets the alarm
 *
 * @remark The alarm is set at now (read in this function) + timeout
 *
 * @param [in] timeout Duration of the Timer ticks
 */
void hal_rtc_start_alarm(uint32_t timeout) {
    app_timer_stop(alarm_timer);
    app_timer_start(alarm_timer, timeout, NULL);
}

uint32_t hal_rtc_get_timer_value(void) {
    return (uint32_t)rtc_get_timestamp_in_ticks();
}

uint32_t hal_rtc_get_timer_elapsed_value(void) {
    uint32_t timestamp_value = hal_rtc_get_timer_value();
    return ((uint32_t)(timestamp_value - hal_rtc.context.time_ref_in_ticks));
}

void hal_rtc_delay_in_ms(const uint32_t milliseconds) {
    nrf_delay_ms(milliseconds);
    /*
        uint64_t delay_in_ticks = 0;
        uint64_t ref_delay_in_ticks = hal_rtc_get_timer_value();
        delay_in_ticks = hal_rtc_ms_2_tick(milliseconds);
        // Wait delay ms
        while (((hal_rtc_get_timer_value() - ref_delay_in_ticks)) < delay_in_ticks) {
            __NOP();
        }*/
}

void hal_rtc_wakeup_timer_set_s(const int32_t seconds) {
    int32_t milliseconds = seconds * 1000;
   /* reset irq status */
    wut_timer_irq_happened = false;
    app_timer_stop(wakeup_timer);
    app_timer_start(wakeup_timer, APP_TIMER_TICKS(milliseconds), NULL);
}

void hal_rtc_wakeup_timer_set_ms(const int32_t milliseconds) {
    /* reset irq status */
    wut_timer_irq_happened = false;
    app_timer_stop(wakeup_timer);
    app_timer_start(wakeup_timer, APP_TIMER_TICKS(milliseconds), NULL);
}

void hal_rtc_wakeup_timer_stop(void) {
    app_timer_stop(wakeup_timer);
}

bool hal_rtc_has_wut_irq_happened(void) {
    return wut_timer_irq_happened;
}

uint32_t hal_rtc_set_time_ref_in_ticks(void) {
    hal_rtc.context.time_ref_in_ticks = (uint32_t)hal_rtc_get_timer_value();
    return hal_rtc.context.time_ref_in_ticks;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void alarm_timeout_handler(void *p_context) {
    timer_irq_handler();
}

static void wakeup_timeout_handler(void *p_context) {
    wut_timer_irq_happened = true;
}

uint32_t hal_rtc_get_time_ref_in_ticks(void) {
    return hal_rtc.context.time_ref_in_ticks;
}

uint32_t hal_rtc_ms_2_tick(const uint32_t milliseconds) {
    return APP_TIMER_TICKS(milliseconds);
}

uint32_t hal_rtc_tick_2_100_us(const uint32_t tick) {
   // return APP_TIMER_100US(tick);
    uint64_t tmp=tick*(APP_TIMER_CONFIG_RTC_FREQUENCY + 1);
    tmp = tmp/(uint64_t)APP_TIMER_CLOCK_FREQ;
    tmp*=10000;
    return (uint32_t)tmp;
}

uint32_t hal_rtc_tick_2_ms(const uint32_t tick) {
    //return APP_TIMER_MS(tick);
    uint64_t tmp=tick*(APP_TIMER_CONFIG_RTC_FREQUENCY + 1);
    tmp = tmp/(uint64_t)APP_TIMER_CLOCK_FREQ;
    tmp*=1000;
    return (uint32_t)tmp;
}

static uint64_t rtc_get_timestamp_in_ticks(void) {
    return app_timer_timestamp_get();
}

uint32_t hal_rtc_get_minimum_timeout(void) { return (MIN_ALARM_DELAY_IN_TICKS); }

uint32_t hal_rtc_temp_compensation(uint32_t period, float temperature) {
    float k = RTC_TEMP_COEFFICIENT;
    float k_dev = RTC_TEMP_DEV_COEFFICIENT;
    float t = RTC_TEMP_TURNOVER;
    float t_dev = RTC_TEMP_DEV_TURNOVER;
    float interim = 0.0;
    float ppm = 0.0;

    if (k < (float)0.0) {
        ppm = (k - k_dev);
    } else {
        ppm = (k + k_dev);
    }
    interim = (temperature - (t - t_dev));
    ppm *= interim * interim;

    /* Calculate the drift in time */
    interim = ((float)period * ppm) / ((float)1e6);

    /* Calculate the resulting time period */
    interim += period;
    interim = floor(interim);

    if (interim < (float)0.0) {
        interim = (float)period;
    }

    /* Calculate the resulting period */
    return (uint32_t)interim;
}