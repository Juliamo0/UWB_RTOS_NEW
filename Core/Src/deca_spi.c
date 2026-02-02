/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions for STM32F446RE - MULTI CS VERSION
 *          Support for 4 DW1000 modules on different CS pins
 *
 * @attention
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for STM32F446RE using HAL - Multi-CS support added
 */

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* External SPI handle - defined in main.c */
extern SPI_HandleTypeDef hspi1;

/* ========== MULTI-CS CONFIGURATION ========== */
/* CS Pin definitions for 4 DW1000 modules */
#define DW_NSS_PORT         GPIOB

#define DW_NSS_PIN_CS1      GPIO_PIN_1   // CS1 → PB1
#define DW_NSS_PIN_CS2      GPIO_PIN_15  // CS2 → PB15
#define DW_NSS_PIN_CS3      GPIO_PIN_14  // CS3 → PB14
#define DW_NSS_PIN_CS4      GPIO_PIN_13  // CS4 → PB13

/* Current active CS pin */
static uint16_t current_cs_pin = DW_NSS_PIN_CS1;  // Default: CS1

/* Array of all CS pins for easy access */
static const uint16_t cs_pins[4] = {
    DW_NSS_PIN_CS1,
    DW_NSS_PIN_CS2,
    DW_NSS_PIN_CS3,
    DW_NSS_PIN_CS4
};

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_select_chip()
 *
 * Select which DW1000 chip to communicate with
 *
 * @param cs_num - Chip select number (1, 2, 3, or 4)
 *
 * @return 0 for success, -1 for invalid CS number
 */
int dw1000_select_chip(uint8_t cs_num)
{
    if (cs_num < 1 || cs_num > 4) {
        return -1;  // Invalid CS number
    }

    current_cs_pin = cs_pins[cs_num - 1];
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_get_current_chip()
 *
 * Get currently selected chip number
 *
 * @return Current chip number (1-4)
 */
uint8_t dw1000_get_current_chip(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (current_cs_pin == cs_pins[i]) {
            return i + 1;
        }
    }
    return 1;  // Default
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_init_all_cs()
 *
 * Initialize all CS pins (set all to HIGH/inactive)
 * Call this once during initialization
 */
void dw1000_init_all_cs(void)
{
    /* Set all CS pins HIGH (inactive) */
    for (uint8_t i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(DW_NSS_PORT, cs_pins[i], GPIO_PIN_SET);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void)
{
    /* SPI already initialized by CubeMX in main.c */
    /* Initialize all CS pins */
    dw1000_init_all_cs();
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    /* Nothing to do - SPI remains initialized */
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16_t headerLength,
               const uint8_t *headerBuffer,
               uint32_t bodyLength,
               const uint8_t *bodyBuffer)
{
    uint8_t tx_buf[300];
    uint8_t rx_buf[300];

    /* Check buffer size */
    if ((headerLength + bodyLength) > sizeof(tx_buf)) {
        return -1;
    }

    /* Combine header and body into single buffer */
    memcpy(tx_buf, headerBuffer, headerLength);
    if (bodyLength > 0 && bodyBuffer != NULL) {
        memcpy(tx_buf + headerLength, bodyBuffer, bodyLength);
    }

    /* Wait for SPI to be ready */
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        /* Wait */
    }

    /* Pull CS low to start transaction */
    HAL_GPIO_WritePin(DW_NSS_PORT, current_cs_pin, GPIO_PIN_RESET);

    /* Small delay for CS setup time */
    for (volatile int i = 0; i < 10; i++);

    /* Transmit data */
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf,
                           headerLength + bodyLength, 100);

    /* Small delay before releasing CS */
    for (volatile int i = 0; i < 10; i++);

    /* Pull CS high to end transaction */
    HAL_GPIO_WritePin(DW_NSS_PORT, current_cs_pin, GPIO_PIN_SET);

    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns 0 for success, or -1 for error
 */
int readfromspi(uint16_t headerLength,
                const uint8_t *headerBuffer,
                uint32_t readLength,
                uint8_t *readBuffer)
{
    uint8_t tx_buf[300];
    uint8_t rx_buf[300];

    /* Check buffer size */
    if ((headerLength + readLength) > sizeof(tx_buf)) {
        return -1;
    }

    /* Prepare TX buffer - header + dummy bytes for read */
    memcpy(tx_buf, headerBuffer, headerLength);
    memset(tx_buf + headerLength, 0xFF, readLength); // Dummy bytes

    /* Wait for SPI to be ready */
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        /* Wait */
    }

    /* Pull CS low to start transaction */
    HAL_GPIO_WritePin(DW_NSS_PORT, current_cs_pin, GPIO_PIN_RESET);

    /* Small delay for CS setup time */
    for (volatile int i = 0; i < 10; i++);

    /* Transmit header and receive data */
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf,
                           headerLength + readLength, 100);

    /* Small delay before releasing CS */
    for (volatile int i = 0; i < 10; i++);

    /* Pull CS high to end transaction */
    HAL_GPIO_WritePin(DW_NSS_PORT, current_cs_pin, GPIO_PIN_SET);

    /* Copy received data (skip header bytes) */
    memcpy(readBuffer, rx_buf + headerLength, readLength);

    return 0;
}

/******************************************************************************
 * End of file
 ******************************************************************************/
