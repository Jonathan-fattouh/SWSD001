/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     MCU Hardware Abstraction Layer implementation
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
#include "smtc_hal_mcu.h"

#include "smtc_board.h"

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "smtc_hal.h"
#include "nrf_pwr_mgmt.h"

#if (HAL_DBG_TRACE == HAL_FEATURE_ON)
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Watchdog counter reload value during sleep
 *
 * \remark The period must be lower than MCU watchdog period
 */
#define WATCHDOG_RELOAD_PERIOD_SECONDS 20
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Low Power options
 */
typedef enum low_power_mode_e {
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool exit_wait = false;
static volatile low_power_mode_t lp_current_mode = LOW_POWER_ENABLE;
static bool partial_sleep_enable = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief init the MCU clock tree
 */
static void hal_mcu_system_clock_config(void);

/*!
 * @brief init the GPIO
 */
static void hal_mcu_gpio_init(void);

/*!
 * @brief init the power voltage detector
 */
static void hal_mcu_pvd_config(void);

#if (HAL_LOW_POWER_MODE == HAL_FEATURE_ON)
/*!
 * @brief Deinit the MCU
 */
static void hal_mcu_lpm_mcu_deinit(void);

/*!
 * @brief Initializes MCU after a stop mode
 */
static void hal_mcu_lpm_mcu_reinit(void);
#endif

/*!
 * @brief MCU enter in low power sleep mode
 */
static void hal_mcu_lpm_enter_sleep_mode(void);

/*!
 * @brief MCU exit low power sleep mode
 */
static void hal_mcu_lpm_exit_sleep_mode(void);

/*!
 * @brief Function runing the low power mode handler
 */
static void hal_mcu_sleep_handler(void);

#if (HAL_DBG_TRACE == HAL_FEATURE_ON)
/*!
 * @brief printf
 */
static void vprint(const char *fmt, va_list argp);
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin(uint32_t *mask) {
    *mask = __get_PRIMASK();
    __disable_irq();
}

void hal_mcu_critical_section_end(uint32_t *mask) {
    __set_PRIMASK(*mask);
}

void hal_mcu_disable_irq(void) {
    __disable_irq();
}

void hal_mcu_enable_irq(void) {
    __enable_irq();
}

void hal_mcu_delay_ms(uint32_t delay_ms) {
    nrf_delay_ms(delay_ms);
}

uint32_t hal_mcu_get_tick(void) {
#if 0
	return HAL_GetTick( );
#endif
}

void hal_mcu_init(void) {
    nrf_pwr_mgmt_init();

    /* Initialize clocks */
    hal_mcu_system_clock_config();

    /* Initialize GPIOs */
    hal_mcu_gpio_init();

    /* Initialize UART */
#if (HAL_USE_PRINTF_UART == HAL_FEATURE_ON)
    hal_uart_init(HAL_PRINTF_UART_ID, UART_TX, UART_RX);
#endif
    /* Initialize low power timer */
    hal_lp_timer_init( );

    /* Initialize the user flash */
    hal_flash_init( );

    /* Initialize SPI */
    hal_spi_init(HAL_RADIO_SPI_ID, SMTC_RADIO_SPI_MOSI, SMTC_RADIO_SPI_MISO, SMTC_RADIO_SPI_SCLK);

    /* Initialize RTC */
    hal_rtc_init( );

    /* Initialize ADC */
   // hal_adc_init( );

    // Initialize watchdog
#if (HAL_USE_WATCHDOG == HAL_FEATURE_ON)
    hal_watchdog_init( );
#endif // HAL_USE_WATCHDOG == HAL_FEATURE_ON
}

void hal_mcu_reset(void) {
    __disable_irq();
    NVIC_SystemReset(); // Restart system
}

void hal_mcu_wait_us(const int32_t microseconds) {
    nrf_delay_us(microseconds);
}

void hal_mcu_set_sleep_for_ms(const int32_t milliseconds) {
    bool last_sleep_loop = false;

    if (milliseconds <= 0) {
        return;
    }

    // Critical section commented - We want to keep RTC interrupt otherwise we will never wake up
   // CRITICAL_SECTION_BEGIN();

    int32_t time_counter = milliseconds;

#if (HAL_USE_WATCHDOG == HAL_FEATURE_ON)
    hal_watchdog_reload();
#endif // HAL_USE_WATCHDOG == HAL_FEATURE_ON

    do {
        if ((time_counter > (WATCHDOG_RELOAD_PERIOD_SECONDS * 1000))) {
            time_counter -= WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
            hal_rtc_wakeup_timer_set_ms(WATCHDOG_RELOAD_PERIOD_SECONDS * 1000);
        } else {
            hal_rtc_wakeup_timer_set_ms(time_counter);
            // if the sleep time is less than the wdog reload period, this is the last sleep loop
            last_sleep_loop = true;
        }
        hal_mcu_sleep_handler();

#if (HAL_USE_WATCHDOG == HAL_FEATURE_ON)
        hal_watchdog_reload();
#endif // HAL_USE_WATCHDOG == HAL_FEATURE_ON
    } while ((hal_rtc_has_wut_irq_happened() == true) && (last_sleep_loop == false));
    if (last_sleep_loop == false) {
        // in case sleep mode is interrupted by an other irq than the wake up timer, stop it and exit
        hal_rtc_wakeup_timer_stop(); //TODO cause infinite loop, why?
    }

     // Critical section commented - We want to keep RTC interrupt otherwise we will never wake up
  //  CRITICAL_SECTION_END();
}

uint16_t hal_mcu_get_vref_level(void) { return hal_adc_get_vref_int(); }

int16_t hal_mcu_get_temperature(void) { return hal_adc_get_temp(); }

void hal_mcu_disable_low_power_wait(void) {
    exit_wait = true;
    lp_current_mode = LOW_POWER_DISABLE;
}

void hal_mcu_enable_low_power_wait(void) {
    exit_wait = false;
    lp_current_mode = LOW_POWER_ENABLE;
}

void hal_mcu_trace_print(const char *fmt, ...) {
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    SMTC_HAL_TRACE_PRINTF("Wrong parameters value: file %s on line %lu\r\n", (const char *)file, line);
    // Infinite loop
    while (1) {
    }
}
#endif

void hal_mcu_partial_sleep_enable(bool enable) { partial_sleep_enable = enable; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void hal_mcu_system_clock_config(void) {
   /* if (nrf_drv_clock_init())
    {
        mcu_panic( );  // Initialization Error
    }
    nrf_drv_clock_lfclk_request(NULL);*/
    NRF_CLOCK->LFCLKSRC = 1;
NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
NRF_CLOCK->TASKS_LFCLKSTART = 1;
while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
NRF_CLOCK->EVENTS_LFCLKSTARTED=0;
}

static void hal_mcu_pvd_config(void) {
#if 0
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_1;
    sConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING;
    if( HAL_PWR_ConfigPVD( &sConfigPVD ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    /* Enable PVD */
    HAL_PWR_EnablePVD( );

    /* Enable and set PVD Interrupt priority */
    HAL_NVIC_SetPriority( PVD_PVM_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( PVD_PVM_IRQn );
#endif
}

static void hal_mcu_gpio_init(void) {
    hal_gpio_init_out(SMTC_RADIO_NSS, 1);
    hal_gpio_init_in(SMTC_RADIO_BUSY, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_OFF, NULL);
    // Here init only the pin as an exti rising and the callback will be attached later
    hal_gpio_init_in(SMTC_RADIO_DIOX, HAL_GPIO_PULL_MODE_DOWN, HAL_GPIO_IRQ_MODE_RISING, NULL);
    hal_gpio_init_out(SMTC_RADIO_NRST, 1);
}

#if (HAL_LOW_POWER_MODE == HAL_FEATURE_ON)
/**
 * @brief De-init periph begore going in sleep mode
 *
 */
static void hal_mcu_lpm_mcu_deinit(void) {
    hal_spi_deinit(HAL_RADIO_SPI_ID);

    /* Disable I2C */
    //  hal_i2c_deinit( HAL_I2C_ID );
    /* Disable UART */
#if (HAL_USE_PRINTF_UART == HAL_FEATURE_ON)
    hal_uart_deinit(HAL_PRINTF_UART_ID);
#endif
}

/**
 * @brief Re-init MCU clock after a wait in stop mode 2
 *
 */
static void hal_mcu_lpm_mcu_reinit(void) {
    /* Initialize UART */
#if (HAL_USE_PRINTF_UART == HAL_FEATURE_ON)
    hal_uart_init(HAL_PRINTF_UART_ID, UART_TX, UART_RX);
#endif
    /* Initialize I2C */
    // hal_i2c_init( HAL_I2C_ID, SMTC_I2C_SDA, SMTC_I2C_SCL );

    /* Initialize SPI */
    hal_spi_init(HAL_RADIO_SPI_ID, SMTC_RADIO_SPI_MOSI, SMTC_RADIO_SPI_MISO, SMTC_RADIO_SPI_SCLK);
}
#endif

/**
 * @brief Enters Sleep Mode
 *
 * @note ARM exits the function when waking up
 *
 */
static void hal_mcu_lpm_enter_sleep_mode(void) {
    // Deinit periph & enter Stop Mode
    if( partial_sleep_enable != true )
    {
        smtc_board_deinit_periph( );
    }

#if (HAL_LOW_POWER_MODE == HAL_FEATURE_ON)
    if( lp_current_mode == LOW_POWER_ENABLE )
    {
      //  hal_mcu_lpm_mcu_deinit( ); //TODO commment for test
        nrf_pwr_mgmt_run();
    }
    else
#endif
    {
        __WFI( );
    }
}

/**
 * @brief Exits Sleep Mode
 *
 */
static void hal_mcu_lpm_exit_sleep_mode(void) {
#if (HAL_LOW_POWER_MODE == HAL_FEATURE_ON)
    if (lp_current_mode == LOW_POWER_ENABLE) {
        /* Reinitializes the MCU */
      //  hal_mcu_lpm_mcu_reinit(); //TODO comment for test
    }
#endif

    if (partial_sleep_enable == false) {
        // Reinitializes the peripherals
        smtc_board_reinit_periph();
    }
}

/**
 * @brief Low power handler
 *
 */
static void hal_mcu_sleep_handler(void) {
    hal_mcu_lpm_enter_sleep_mode( );
    hal_mcu_lpm_exit_sleep_mode( );
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    HAL_DBG_TRACE_ERROR("\x1B[0;31m"); // red color
    HAL_DBG_TRACE_ERROR("HARDFAULT_Handler\n");
    HAL_DBG_TRACE_ERROR("\x1B[0m"); // default color
    while (1) {
    }
}

#if (HAL_DBG_TRACE == HAL_FEATURE_ON)
static void vprint(const char *fmt, va_list argp) {
    char string[HAL_PRINT_BUFFER_SIZE];
    if (0 < vsprintf(string, fmt, argp)) // build string
    {
        hal_uart_tx(HAL_PRINTF_UART_ID, (uint8_t *)string, strlen(string));
    }
}
#endif

/* --- EOF ------------------------------------------------------------------ */