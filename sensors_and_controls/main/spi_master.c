#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

#include "spi_master.h"
#include "config.h"

#define SENDER_HOST HSPI_HOST

static xQueueHandle rdySem;


static void IRAM_ATTR gpio_handshake_isr_handler(void* arg) {
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime;
    uint32_t currtime=esp_cpu_get_ccount();
    uint32_t diff=currtime-lasthandshaketime;
    if (diff<240000) return; //ignore everything <1ms after an earlier irq
    lasthandshaketime=currtime;

    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

void configSpiMaster(spi_device_handle_t* handle) {
    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=SPI_MOSI_GPIO,
        .miso_io_num=SPI_MISO_GPIO,
        .sclk_io_num=SPI_SCLK_GPIO,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=5000000,
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=SPI_CS_GPIO,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    //GPIO config for the handshake line.
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<SPI_HANDSHAKE_GPIO)
    };

    //Create the semaphore.
    rdySem=xSemaphoreCreateBinary();
    
    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(SPI_HANDSHAKE_GPIO, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(SPI_HANDSHAKE_GPIO, gpio_handshake_isr_handler, NULL);

    //Initialize the SPI bus and add the device we want to send stuff to.
    spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SENDER_HOST, &devcfg, handle);

    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect
    //positive edge on the handshake line.
    xSemaphoreGive(rdySem);
}

uint8_t sendReceiveByteSpi(spi_device_handle_t* handle, uint8_t send) {
    uint8_t receive;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &send,
        .rx_buffer = &receive,
    };

    //Wait for slave to be ready for next byte before sending
    xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
    spi_device_transmit(*handle, &trans);

    return receive;
}

void sendReceiveSpi(spi_device_handle_t* handle, void* send, void* receive, size_t length) {
    spi_transaction_t trans = {
        .length = 8*length,
        .tx_buffer = send,
        .rx_buffer = receive,
    };

    xSemaphoreTake(rdySem, portMAX_DELAY);
    spi_device_transmit(*handle, &trans);
}