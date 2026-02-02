/*! ----------------------------------------------------------------------------
 * @file	deca_spi.h
 * @brief	SPI access functions - MULTI CS VERSION
 *          Support for 4 DW1000 modules on different CS pins
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for Multi-CS support
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "deca_types.h"

#define DECA_MAX_SPI_HEADER_LENGTH      (3)                     // max number of bytes in header (for formating & sizing)

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * MULTI-CS SUPPORT FUNCTIONS
 */

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_select_chip()
 *
 * Select which DW1000 chip to communicate with
 *
 * @param cs_num - Chip select number (1, 2, 3, or 4)
 *                 1 = CS1 (PB1)
 *                 2 = CS2 (PB15)
 *                 3 = CS3 (PB14)
 *                 4 = CS4 (PB13)
 *
 * @return 0 for success, -1 for invalid CS number
 */
int dw1000_select_chip(uint8_t cs_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_get_current_chip()
 *
 * Get currently selected chip number
 *
 * @return Current chip number (1-4)
 */
uint8_t dw1000_get_current_chip(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_init_all_cs()
 *
 * Initialize all CS pins (set all to HIGH/inactive)
 * Call this once during initialization
 */
void dw1000_init_all_cs(void);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
