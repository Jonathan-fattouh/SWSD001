/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
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

#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "smtc_hal.h"

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

/*!
 * GPIO setup data structure
 */
typedef struct bsp_gpio_s {
    hal_gpio_pin_names_t pin;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
    uint32_t alternate;
} gpio_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * Array holding attached IRQ gpio data context
 */
static hal_gpio_irq_t const *gpio_irq[32];//[16];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/** @brief GPIOTE event handler.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t unused) {
    uint8_t callback_index = pin;
   // uint8_t callback_index = 0;

  /*  if (pin > 0) {
        while (pin != 0x01) {
            pin = pin >> 1;
            callback_index++;
        }
    }*/

    if ((gpio_irq[callback_index] != NULL) && (gpio_irq[callback_index]->callback != NULL)) {
        gpio_irq[callback_index]->callback(gpio_irq[callback_index]->context);
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

//
// MCU input pin Handling
//

void hal_gpio_init_in(const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
    const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t *irq) {
    const uint32_t modes[] = {0xFF, NRF_GPIOTE_POLARITY_LOTOHI, NRF_GPIOTE_POLARITY_HITOLO, NRF_GPIOTE_POLARITY_TOGGLE};
    const uint32_t pulls[] = {NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_PULLDOWN};

    // No Interrupt needed
    if (irq_mode == HAL_GPIO_IRQ_MODE_OFF) {
        nrf_gpio_cfg_input(pin, pulls[pull_mode]);
    }
    // Interrupt needed
    else {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.hi_accuracy = false;

        // Check if gpiote lib is already initialized, initiliaze it if not
        if (!nrf_drv_gpiote_is_init()) {
            nrf_drv_gpiote_init();
        }

        hal_gpio_irq_attach(irq);

        gpiote_in_config.sense = modes[irq_mode];
        gpiote_in_config.pull = pulls[pull_mode];

        nrf_drv_gpiote_in_init(pin, &gpiote_in_config, gpiote_evt_handler);
        nrf_drv_gpiote_in_event_enable(pin, true);
    }
}

void hal_gpio_init_out(const hal_gpio_pin_names_t pin, const hal_gpio_state_t value) {
    nrf_gpio_cfg_output(pin);
    nrf_gpio_pin_write(pin, value);
}

void hal_gpio_irq_attach(const hal_gpio_irq_t *irq) {
    if ((irq != NULL) && (irq->callback != NULL)) {
        gpio_irq[(irq->pin)/* & 0x0F*/] = irq;
    }
}

void hal_gpio_irq_deatach(const hal_gpio_irq_t *irq) {
    if (irq != NULL) {
        gpio_irq[(irq->pin) /*& 0x0F*/] = NULL;
    }
}

void hal_gpio_irq_enable(void) {
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

void hal_gpio_irq_disable(void) {
    NVIC_DisableIRQ(GPIOTE_IRQn);
}

//
// MCU pin state control
//

void hal_gpio_set_value(const hal_gpio_pin_names_t pin, const hal_gpio_state_t value) {
    nrf_gpio_pin_write(pin, value);
}

uint32_t hal_gpio_get_value(const hal_gpio_pin_names_t pin) {
    return nrf_gpio_pin_read(pin);
}

void hal_gpio_clear_pending_irq(const hal_gpio_pin_names_t pin) 
{ 
   NVIC_ClearPendingIRQ( GPIOTE_IRQn );                                                    
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */