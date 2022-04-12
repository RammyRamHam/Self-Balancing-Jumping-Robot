#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "spi_slave.h"
#include "config.h"

#define RCV_HOST HSPI_HOST

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<SPI_HANDSHAKE_GPIO));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<SPI_HANDSHAKE_GPIO));
}

void configSpiSlave() {
     //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=SPI_MOSI_GPIO,
        .miso_io_num=SPI_MISO_GPIO,
        .sclk_io_num=SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=SPI_CS_GPIO,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<SPI_HANDSHAKE_GPIO)
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(SPI_MOSI_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_SCLK_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_CS_GPIO, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}

uint8_t sendReceiveByteSpi(uint8_t send) {
    uint8_t receive;
    spi_slave_transaction_t trans = {
        .length = 8,
        .tx_buffer = &send,
        .rx_buffer = &receive,
    };

    spi_slave_transmit(RCV_HOST, &trans, portMAX_DELAY);

    return receive;
}

void sendReceiveSpi(void* send, void* receive, size_t length) {
    spi_slave_transaction_t trans = {
        .length = 8*length,
        .tx_buffer = send,
        .rx_buffer = receive,
    };

    spi_slave_transmit(RCV_HOST, &trans, portMAX_DELAY);
}