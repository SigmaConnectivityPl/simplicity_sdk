/***************************************************************************//**
 * @file
 * @brief Configuration header for bootloader Spi Peripheral Eusart Driver
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.  This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#ifndef BTL_SPI_PERIPHERAL_EUSART_DRIVER_CONFIG_H
#define BTL_SPI_PERIPHERAL_EUSART_DRIVER_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

// <h>SPI Peripheral Eusart Driver

// <o SL_SPI_PERIPHERAL_EUSART_RX_BUFFER_SIZE> Receive buffer size:[0-2048] <0-2048>
// <i> Default: 300
#define SL_SPI_PERIPHERAL_EUSART_RX_BUFFER_SIZE    300

// <o SL_SPI_PERIPHERAL_EUSART_TX_BUFFER_SIZE> Transmit buffer size:[0-2048] <0-2048>
// <i> Default: 50
#define SL_SPI_PERIPHERAL_EUSART_TX_BUFFER_SIZE    50

// <o SL_SPI_PERIPHERAL_EUSART_LDMA_RX_CHANNEL> LDMA channel for SPI RX:[0-1] <0-1>
// <i> Default: 0
#define SL_SPI_PERIPHERAL_EUSART_LDMA_RX_CHANNEL    0

// <o SL_SPI_PERIPHERAL_EUSART_LDMA_TX_CHANNEL> LDMA channel for SPI TX:[0-1] <0-1>
// <i> Default: 1
#define SL_SPI_PERIPHERAL_EUSART_LDMA_TX_CHANNEL    1

// </h>

// <<< end of configuration section >>>

// <<< sl:start pin_tool >>>
// <eusart signal=TX,RX,CS,SCLK> SL_EUSART_SPINCP

// $[EUSART_SL_EUSART_SPINCP]
#define SL_EUSART_SPINCP_PERIPHERAL              EUSART1
#define SL_EUSART_SPINCP_PERIPHERAL_NO           1

// EUSART1 TX on PC10
#define SL_EUSART_SPINCP_TX_PORT                 gpioPortC
#define SL_EUSART_SPINCP_TX_PIN                  10

// EUSART1 RX on PC11
#define SL_EUSART_SPINCP_RX_PORT                 gpioPortC
#define SL_EUSART_SPINCP_RX_PIN                  11

// EUSART1 CS on PB06
#define SL_EUSART_SPINCP_CS_PORT                 gpioPortB
#define SL_EUSART_SPINCP_CS_PIN                  6

// EUSART1 SCLK on PB05
#define SL_EUSART_SPINCP_SCLK_PORT               gpioPortB
#define SL_EUSART_SPINCP_SCLK_PIN                5

// [EUSART_SL_EUSART_SPINCP]$

// <<< sl:end pin_tool >>>

#endif // BTL_SPI_PERIPHERAL_EUSART_DRIVER_CONFIG_H
