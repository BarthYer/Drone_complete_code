#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <string.h>
#include <stdbool.h>
#include "drone_receiver.h"

const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#define CE_PIN   48
#define CSN_PIN  10


#define NRF_CMD_W_REGISTER   0x20
#define NRF_CMD_R_REGISTER   0x00
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_R_RX_PAYLOAD 0x61
#define NRF_CMD_FLUSH_TX     0xE1
#define NRF_CMD_FLUSH_RX     0xE2
#define NRF_CMD_NOP          0xFF

#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11


uint8_t addr[5] = {'N','O','D','E','1'};


static const struct device *spi_dev;
static struct spi_config spi_cfg;

struct DataPackage data;


static inline void csn_low()
{
    gpio_pin_set(gpio_dev, CSN_PIN, 0);
}

static inline void csn_high()
{
    gpio_pin_set(gpio_dev, CSN_PIN, 1);
}

uint8_t spi_transfer(uint8_t data)
{
    uint8_t tx = data;
    uint8_t rx;

    struct spi_buf tx_buf = {.buf = &tx, .len = 1};
    struct spi_buf rx_buf = {.buf = &rx, .len = 1};

    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    csn_low();
    spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    csn_high();

    return rx;
}

void nrf_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {NRF_CMD_W_REGISTER | reg, value};
    uint8_t rx[2];

    struct spi_buf tx_buf = {.buf = tx, .len = 2};
    struct spi_buf rx_buf = {.buf = rx, .len = 2};

    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    csn_low();
    spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    csn_high();
}

uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {NRF_CMD_R_REGISTER | reg, NRF_CMD_NOP};
    uint8_t rx[2];

    struct spi_buf tx_buf = {.buf = tx, .len = 2};
    struct spi_buf rx_buf = {.buf = rx, .len = 2};

    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    csn_low();
    spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    csn_high();

    return rx[1];
}

void nrf_write_buf(uint8_t cmd, uint8_t *data, int len)
{
    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = cmd;
    memcpy(&tx[1], data, len);

    struct spi_buf tx_buf = {.buf = tx, .len = len + 1};
    struct spi_buf rx_buf = {.buf = rx, .len = len + 1};

    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    csn_low();
    spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    csn_high();
}


void nrf_init_rx()
{
    gpio_pin_set(gpio_dev, CE_PIN, 0);

    nrf_write_reg(REG_CONFIG, 0x0F);   // RX mode, CRC enabled, power up
    nrf_write_reg(REG_EN_AA, 0x00);    // disable auto-ack
    nrf_write_reg(REG_EN_RXADDR, 0x01); // enable pipe 0
    nrf_write_reg(REG_SETUP_AW, 0x03); // 5-byte address
    nrf_write_reg(REG_RF_CH, 76);       // channel 76
    nrf_write_reg(REG_RF_SETUP, 0x06); // 1 Mbps, PA MAX (0 dBm)

    nrf_write_buf(NRF_CMD_W_REGISTER | REG_RX_ADDR_P0, addr, 5);

    /* Payload width must match sizeof(DataPackage) on both TX and RX sides */
    nrf_write_reg(REG_RX_PW_P0, sizeof(struct DataPackage));

    gpio_pin_set(gpio_dev, CE_PIN, 1); // start listening
}


void nrf_driver_init(void)
{
    spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));

    spi_cfg.frequency = 1000000;
    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;

    gpio_pin_configure(gpio_dev, CE_PIN,  GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(gpio_dev, CSN_PIN, GPIO_OUTPUT_ACTIVE);

    nrf_init_rx();
}


struct DataPackage read_data()
{
    struct DataPackage data = {0};  /* initialise so we return zeros when no packet */
    uint8_t status = nrf_read_reg(REG_STATUS);

    if (status & 0x40) /* RX_DR bit set: data is ready */
    {
        uint8_t tx[sizeof(struct DataPackage) + 1];
        uint8_t rx[sizeof(struct DataPackage) + 1];

        /* Build SPI transaction: command byte + dummy bytes */
        tx[0] = NRF_CMD_R_RX_PAYLOAD;
        memset(&tx[1], 0xFF, sizeof(struct DataPackage));

        struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
        struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};

        struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
        struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

        csn_low();
        spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
        csn_high();

        /* rx[0] = status byte, rx[1..N] = payload */
        memcpy(&data, &rx[1], sizeof(struct DataPackage));

        /* Clear RX_DR interrupt flag */
        nrf_write_reg(REG_STATUS, 0x40);

        /* Flush RX FIFO */
        spi_transfer(NRF_CMD_FLUSH_RX);

        /*printk("x_right: %d | y_right: %d | x_left: %d | y_left: %d | active: %d\n",
               data.x_right,
               data.y_right,
               data.x_left,
               data.y_left,
               (int)data.drone_active);*/
    }
    return data;
}



