#include "speaker.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "audio.h"
#include "esp_adc_cal.h"
#include "esp_rom_sys.h"
#include "config.h"


/**
 * @brief I2S ADC/DAC mode init.
 */
void speakerInit(void) {
    i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
    .sample_rate =  44100,
    .bits_per_sample = I2S_SAMPLE_BITS,
    .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
    .channel_format = I2S_FORMAT,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 1024,
    .use_apll = false,
    };
    //install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    //init DAC pad
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    i2s_zero_dma_buffer(I2S_NUM);
}


/**
 * @brief Reset i2s clock and mode
 */
void speakerReset(void) {
    i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_SAMPLE_BITS, I2S_CHANNEL_NUM);
}

/**
 * @brief Set i2s clock for example audio file
 */
void speakerSetFilePlayMode(void) {
    i2s_set_clk(I2S_NUM, 16000, I2S_SAMPLE_BITS, 1);
}

/**
 * @brief Scale data to 16bit/32bit for I2S DMA output.
 *        DAC can only output 8bit data value.
 *        I2S DMA will still send 16 bit or 32bit data, the highest 8bit contains DAC data.
 */
int speakerDataScale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
    uint32_t j = 0;
#if (I2S_SAMPLE_BITS == 16)
    for (int i = 0; i < len; i++) {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i]*3/10;
    }
    return (len * 2);
#else
    for (int i = 0; i < len; i++) {
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0.3*s_buff[i];
    }
    return (len * 4);
#endif
}


void speakerPlay(void*arg) {
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_FAT, AUDIO_PARTITION_NAME);
    int i2s_read_len = I2S_READ_LEN;
    size_t bytes_written;

    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    while (1) {

        printf("Playing file: \n");
        int offset = 0;
        int tot_size = sizeof(audio_table);
        speakerSetFilePlayMode();
        while (offset < tot_size) {
            int play_len = ((tot_size - offset) > (1024)) ? (1024) : (tot_size - offset);
            int i2s_wr_len = speakerDataScale(i2s_write_buff, (uint8_t*)(audio_table + offset), play_len);
            i2s_write(I2S_NUM, i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
            offset += play_len;
            //example_disp_buf((uint8_t*) i2s_write_buff, 32);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        speakerReset();
    }
    
    free(i2s_write_buff);
    vTaskDelete(NULL);
}
