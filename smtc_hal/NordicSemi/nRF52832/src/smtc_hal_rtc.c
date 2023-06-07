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

#include "nrf_drv_rtc.h"
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

#define RTC2_CLOCK_FREQ 32768 /**< Clock frequency of the RTC timer. */
#define RTC2_PRESCALER 2      // resolution ~100us
// #define RTC2_PRESCALER 31 // resolution ~1ms

/* RTC Time base in us */
#define USEC_NUMBER 1000000U
#define MSEC_NUMBER (USEC_NUMBER / 1000)

/*!
 * @brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR ((uint32_t)366U)
#define DAYS_IN_YEAR ((uint32_t)365U)
#define SECONDS_IN_1DAY ((uint32_t)86400U)
#define SECONDS_IN_1HOUR ((uint32_t)3600U)
#define SECONDS_IN_1MINUTE ((uint32_t)60U)
#define MINUTES_IN_1HOUR ((uint32_t)60U)
#define HOURS_IN_1DAY ((uint32_t)24U)

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
    nrf_drv_rtc_t instance;
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
    .instance = NRF_DRV_RTC_INSTANCE(2),
    .context = {0},
};

static volatile bool wut_timer_irq_happened = false;

/*!
 * \brief Indicates the number of overflows
 */
static uint8_t m_ovrflw_cnt = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type);

#if 0
/*!
 * @brief Get current full resolution RTC timestamp in ticks
 *
 * @returns timestamp_in_ticks Current timestamp in ticks
 */
static uint64_t rtc_get_timestamp_in_ticks( RTC_DateTypeDef* date, RTC_TimeTypeDef* time );
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init(void) {
    // Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = RTC2_PRESCALER;
    if (nrf_drv_rtc_init(&hal_rtc.instance, &config, rtc_handler) != NRF_SUCCESS) {
        mcu_panic();
    }

    // Power on RTC instance
  //  nrf_drv_rtc_tick_enable(&hal_rtc.instance, true);
    nrf_drv_rtc_overflow_enable(&hal_rtc.instance, true);
    nrf_drv_rtc_enable(&hal_rtc.instance);

    hal_rtc_set_time_ref_in_ticks();
}

uint32_t hal_rtc_get_time_s(void) {
    uint32_t milliseconds;
    uint32_t ticks;

    ticks = hal_rtc_get_timer_value();
    milliseconds = hal_rtc_tick_2_ms(ticks);

    uint32_t seconds = (uint32_t)(milliseconds / 1000);

    return seconds;
}

uint32_t hal_rtc_get_time_100us(void) {
    uint32_t ticks;
    uint32_t val_100us;

    ticks = hal_rtc_get_timer_value();

    // TODO

    return val_100us;
}

uint32_t hal_rtc_get_time_ms(void) {
    uint32_t milliseconds;
    uint32_t ticks;

    ticks = hal_rtc_get_timer_value();
    milliseconds = hal_rtc_tick_2_ms(ticks);

    return milliseconds;
}

void hal_rtc_stop_alarm(void) {
    nrfx_rtc_cc_disable(&hal_rtc.instance, 0);
}

/*!
 * @brief Sets the alarm
 *
 * @remark The alarm is set at now (read in this function) + timeout
 *
 * @param [in] timeout Duration of the Timer ticks
 */
void hal_rtc_start_alarm(uint32_t timeout) {
    uint32_t now = hal_rtc_get_timer_value();

    nrf_drv_rtc_cc_set(&hal_rtc.instance, 0, now + timeout, true);

    HAL_DBG_TRACE_DEBUG("start_alarm: now=%d, timeout=%d, res= %d\n", now, timeout, now + timeout);
}

uint32_t hal_rtc_get_timer_value(void) {
    uint32_t value;

    value = (((uint32_t)m_ovrflw_cnt) << 24) + NRF_RTC2->COUNTER;

    return value;
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
    uint32_t now, ticks;

    ticks = hal_rtc_ms_2_tick(1000 * seconds);
    now = hal_rtc_get_timer_value();

    nrfx_rtc_cc_disable(&hal_rtc.instance, 1);
    /* reset irq status */
    wut_timer_irq_happened = false;
    nrf_drv_rtc_cc_set(&hal_rtc.instance, 1, now + ticks, true);
}

void hal_rtc_wakeup_timer_set_ms(const int32_t _milliseconds) {
    uint32_t now, ticks;

    ticks = hal_rtc_ms_2_tick(_milliseconds);
    now = hal_rtc_get_timer_value();

    nrfx_rtc_cc_disable(&hal_rtc.instance, 1);
    /* reset irq status */
    wut_timer_irq_happened = false;
    nrf_drv_rtc_cc_set(&hal_rtc.instance, 1, now + ticks, true);
  //   HAL_DBG_TRACE_INFO("wakeup_timer ms: now=%d, timeout=%d, res= %d\n", now, ticks, now + ticks);
}

void hal_rtc_wakeup_timer_stop(void) {
    nrfx_rtc_cc_disable(&hal_rtc.instance, 1);
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

static void rtc_handler(nrf_drv_rtc_int_type_t int_type) {
    if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
        timer_irq_handler();
    }
    if (int_type == NRF_DRV_RTC_INT_COMPARE1) {
        wut_timer_irq_happened = true;
    }
    if (int_type == NRF_DRV_RTC_INT_OVERFLOW) {
        m_ovrflw_cnt++;
    }
}

uint32_t hal_rtc_get_time_ref_in_ticks(void) {
    return hal_rtc.context.time_ref_in_ticks;
}

uint32_t hal_rtc_ms_2_tick(const uint32_t milliseconds) {
    return ((uint32_t)ROUNDED_DIV((milliseconds) * ((uint64_t)RTC2_CLOCK_FREQ), 1000 * (RTC2_PRESCALER + 1)));
}

uint32_t hal_rtc_tick_2_100_us(const uint32_t tick) {
    return ((uint32_t)ROUNDED_DIV((tick) * ((uint64_t)(10000 * (RTC2_PRESCALER + 1))), (uint64_t)RTC2_CLOCK_FREQ));
}

uint32_t hal_rtc_tick_2_ms(const uint32_t tick) {
    return ((uint32_t)ROUNDED_DIV((tick) * ((uint64_t)(1000 * (RTC2_PRESCALER + 1))), (uint64_t)RTC2_CLOCK_FREQ));
}

#if 0
static uint64_t rtc_get_timestamp_in_ticks( RTC_DateTypeDef* date, RTC_TimeTypeDef* time )
{
    uint64_t timestamp_in_ticks = 0;
    uint32_t correction;
    uint32_t seconds;

    /* Make sure it is correct due to asynchronous nature of RTC */
    volatile uint32_t ssr;

    do
    {
        ssr = RTC->SSR;
        HAL_RTC_GetDate( &hal_rtc.handle, date, RTC_FORMAT_BIN );
        HAL_RTC_GetTime( &hal_rtc.handle, time, RTC_FORMAT_BIN );
    } while( ssr != RTC->SSR );

    /* Calculate amount of elapsed days since 01/01/2000 */
    seconds = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * date->Year, 4 );

    correction = ( ( date->Year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;

    seconds +=
        ( DIVC( ( date->Month - 1 ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( date->Month - 1 ) * 2 ) ) & 0x03 ) ) );

    seconds += ( date->Date - 1 );

    /* Convert from days to seconds */
    seconds *= SECONDS_IN_1DAY;

    seconds += ( ( uint32_t ) time->Seconds + ( ( uint32_t ) time->Minutes * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t ) time->Hours * SECONDS_IN_1HOUR ) );

    timestamp_in_ticks = ( ( ( uint64_t ) seconds ) << N_PREDIV_S ) + ( PREDIV_S - time->SubSeconds );

    return timestamp_in_ticks;
}

#endif
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