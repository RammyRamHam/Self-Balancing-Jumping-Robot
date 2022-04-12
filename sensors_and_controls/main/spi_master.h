#ifndef _SPI_MASTER_H_
#define _SPI_MASTER_H_

#include "driver/spi_master.h"

//static void IRAM_ATTR gpio_handshake_isr_handler(void* arg);
void configSpiMaster(spi_device_handle_t* handle);
uint8_t sendReceiveByteSpi(spi_device_handle_t* handle, uint8_t data);
void sendReceiveSpi(spi_device_handle_t* handle, void* send, void* receive, size_t length);

#endif