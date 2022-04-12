#ifndef _SPI_SLAVE_H_
#define _SPI_SLAVE_H_

#include "driver/spi_slave.h"

void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans);
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans);
void configSpiSlave();
uint8_t sendReceiveByteSpi(uint8_t send);
void sendReceiveSpi(void* send, void* receive, size_t length);
#endif