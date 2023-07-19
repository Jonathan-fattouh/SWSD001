/*!
 * \file      smtc_hal_flash.c
 *
 * \brief     FLASH Hardware Abstraction Layer implementation
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h> // TODO: check if needed

#include "smtc_hal.h"
#ifdef SOFTDEVICE_PRESENT
#include "nrf_fstorage_sd.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#else
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <string.h>
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define FLASH_OPERATION_MAX_RETRY 4
#define FLASH_PAGE_DIVIDER 10
#define NB_FLASH_BYTES_TO_TEST ADDR_FLASH_PAGE_SIZE / FLASH_PAGE_DIVIDER

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

/**
 * @brief  Gets the page of a given address
 * @param  address: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t hal_flash_get_page(uint32_t address);

static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt);

static uint32_t nrf5_flash_end_addr_get();

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
    {
        /* Set a handler for fstorage events. */
        .evt_handler = fstorage_evt_handler,

        /* These below are the boundaries of the flash space assigned to this instance of fstorage.
         * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
         * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
         * last page of flash available to write data. */
        .start_addr = 0x7C000,
        .end_addr = 0x80000,
}; // NOTE flash addr set manualy to end of flash for debug purpose, they are changed at runtime

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_status_t hal_flash_init(void) {
    uint8_t status = SMTC_HAL_SUCCESS;
    nrf_fstorage_api_t *p_fs_api;

#ifdef SOFTDEVICE_PRESENT
    NRF_LOG_INFO("SoftDevice is present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_sd implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
#else
    NRF_LOG_INFO("SoftDevice not present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_nvmc implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
     * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the
     * SoftDevice is disabled or not present.
     *
     * Using this implementation when the SoftDevice is enabled results in a hardfault. */
    p_fs_api = &nrf_fstorage_nvmc;
#endif

    if (nrf_fstorage_init(&fstorage, p_fs_api, NULL)) {
        mcu_panic();
    }

    // locate start & end of user flash space
    fstorage.start_addr = (CODE_END & ~(NRF_FICR->CODEPAGESIZE-1));
    fstorage.end_addr = nrf5_flash_end_addr_get();

    return status;
}

smtc_hal_status_t hal_flash_erase_page(uint32_t addr, uint8_t nb_page) {
    uint8_t status = SMTC_HAL_SUCCESS;
    uint32_t first_user_page = 0;
    uint32_t nb_of_pages_max = 0;

    /* Get the 1st page to erase */
    first_user_page = hal_flash_get_page(addr);

    if ((fstorage.start_addr > addr) || fstorage.end_addr < (addr + nb_page*NRF_FICR->CODEPAGESIZE)) { 
        status = SMTC_HAL_FAILURE;
        return status;
    }

    if (nrf_fstorage_erase(&fstorage, addr, nb_page, NULL)) {
        /*
       Error occurred while  erase.
       User can add here some code to deal with this error.
     */
        /* Infinite loop */
        while (1) {
        }
    }

    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(&fstorage)) {
    }

    return status;
}

uint8_t hal_flash_force_erase_page(uint32_t addr, uint8_t nb_page) {
    uint8_t status = SMTC_HAL_SUCCESS;
    uint32_t first_user_page = 0;
    uint32_t nb_of_pages_max = 0;

    /* Get the 1st page to erase */
    first_user_page = hal_flash_get_page(addr);

    /* Get the number of pages to erase from 1st page */
   // nb_of_pages_max = hal_flash_get_page(FLASH_USER_END_ADDR) - hal_flash_get_page(flash_user_start_addr) + 1;


    if (nrf_fstorage_erase(&fstorage, addr, nb_page, NULL)) {
        /*
       Error occurred while  erase.
       User can add here some code to deal with this error.
     */
        /* Infinite loop */
        while (1) {
        }
    }

    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(&fstorage)) {
    }

    return status;
}

smtc_hal_status_t hal_flash_write_buffer(uint32_t addr, const uint8_t *buffer, uint32_t size) {
    uint8_t status = SMTC_HAL_SUCCESS;
    uint32_t real_size = 0;
    uint32_t addr_end = 0;
    uint32_t nb_of_pages_max = 0;

    /* Complete size for FLASH_TYPEPROGRAM_DOUBLEWORD operation*/
    if ((size % 8) != 0) {
        real_size = size + (8 - (size % 8));
    } else {
        real_size = size;
    }

    addr_end = addr + real_size;

    /* Get the number of pages available */
 //   nb_of_pages_max = hal_flash_get_page(FLASH_USER_END_ADDR) - hal_flash_get_page(flash_user_start_addr) + 1;

    if ((fstorage.start_addr > addr) || fstorage.end_addr < addr_end) { 
        status = SMTC_HAL_FAILURE;
        return status;
    }

    /* Program the user Flash area */
    if (nrf_fstorage_write(&fstorage, addr, buffer, size, NULL)) {
        /* Error occurred while writing data in Flash memory.
        User can add here some code to deal with this error */
        /* Infinite loop */
        while (1) {
        }
    }

    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(&fstorage)) {
    }

    return real_size;
}

void hal_flash_read_buffer(uint32_t addr, uint8_t *buffer, uint32_t size) {

    nrf_fstorage_read(&fstorage, addr, buffer, size);
}

uint32_t hal_flash_get_user_start_addr(void) { return  fstorage.start_addr; }

void hal_flash_set_user_start_addr(uint32_t addr) {  fstorage.start_addr = addr; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get() {
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ? bootloader_addr : (code_sz * page_sz));
}

static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt) {
    if (p_evt->result != NRF_SUCCESS) {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id) {
    case NRF_FSTORAGE_EVT_WRITE_RESULT: {
        NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
            p_evt->len, p_evt->addr);
    } break;

    case NRF_FSTORAGE_EVT_ERASE_RESULT: {
        NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
            p_evt->len, p_evt->addr);
    } break;

    default:
        break;
    }
}

static uint32_t hal_flash_get_page(uint32_t address) {
#if 0
    return ( address - FLASH_BASE ) / FLASH_PAGE_SIZE;
#else
    return 0;
#endif
}

/* --- EOF ------------------------------------------------------------------ */