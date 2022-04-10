#include "driver/i2c.h"
#include "i2c.h"
#include "config.h"

void i2cInit(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    i2c_set_timeout(i2c_master_port , I2C_TIMEOUT);

    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void i2cWriteByte(uint8_t devAdd, uint8_t regAdd, uint8_t data) {
    uint8_t writeBuf[2] = {regAdd, data};
    i2c_master_write_to_device(I2C_MASTER_NUM, devAdd, writeBuf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

uint8_t i2cReadByte(uint8_t devAdd, uint8_t regAdd) {
    uint8_t data;
    i2c_master_write_read_device(I2C_MASTER_NUM, devAdd, &regAdd, 1, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    return data;
}

void i2cReadBytes(uint8_t devAdd, uint8_t regAdd, uint8_t* data, uint8_t length) {
    i2c_master_write_read_device(I2C_MASTER_NUM, devAdd, &regAdd, 1, data, length, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}
