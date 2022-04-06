#include <stdio.h>
#include "esp_log.h"
#include "i2c.h"

static const char *TAG = "sensors_and_controls";

#define BNO055_SENSOR_ADDR                 0x68
#define BNO055_WHO_AM_I_REG_ADDR           0x75

#define BNO055_PWR_MGMT_1_REG_ADDR         0x6B
#define BNO055_RESET_BIT                   7


static void BNO055_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_master_write_read_device(I2C_MASTER_NUM, BNO055_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}


static void BNO055_register_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    
    i2c_master_write_to_device(I2C_MASTER_NUM, BNO055_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}


void app_main(void) {
    uint8_t data[2];
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

    BNO055_register_read(BNO055_WHO_AM_I_REG_ADDR, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by reseting the BNO055 */
    BNO055_register_write_byte(BNO055_PWR_MGMT_1_REG_ADDR, 1 << BNO055_RESET_BIT);

    i2c_driver_delete(I2C_MASTER_NUM);
    ESP_LOGI(TAG, "I2C unitialized successfully");
}
