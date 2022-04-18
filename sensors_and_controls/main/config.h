#ifndef _CONFIG_H_
#define _CONFIG_H_

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_TIMEOUT                 (1<<19)

#define SPI_HANDSHAKE_GPIO          17
#define SPI_MOSI_GPIO               23
#define SPI_MISO_GPIO               19
#define SPI_SCLK_GPIO               18
#define SPI_CS_GPIO                 5

#define PID_KP                      -0.5
#define PID_KI                      0.0
#define PID_KD                      0.0
#define PID_MAX_I                   0.0

#define ANGLE_SETPOINT              52.0

#define I2S_SAMPLE_RATE             16000
#define I2S_NUM                     0
#define I2S_SAMPLE_BITS             16
#define AUDIO_PARTITION_NAME        "storage"
#define I2S_READ_LEN                16 * 1024
#define I2S_FORMAT                  (I2S_CHANNEL_FMT_RIGHT_LEFT)
#define I2S_CHANNEL_NUM             1//((I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))


#endif