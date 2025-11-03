#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_slave_demo, LOG_LEVEL_INF);

/* SPI slave node from device tree */
#define SPI_SLAVE_NODE DT_NODELABEL(spi3)

static const struct device *spis_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);

/* TX and RX buffers */
static uint8_t tx_buf[4] = {0x11, 0x22, 0x33, 0x44};
static uint8_t rx_buf[4] = {0};

/* SPI buffer structures */
static struct spi_buf tx_spi_buf = {
    .buf = tx_buf,
    .len = sizeof(tx_buf)
};

static struct spi_buf rx_spi_buf = {
    .buf = rx_buf,
    .len = sizeof(rx_buf)
};

static const struct spi_buf_set tx_set = {
    .buffers = &tx_spi_buf,
    .count = 1
};

static const struct spi_buf_set rx_set = {
    .buffers = &rx_spi_buf,
    .count = 1
};

/* SPI configuration for NCS v3.1.1 */
/* Mode 0: CPOL=0, CPHA=0, MSB first, slave mode */
static const struct spi_config spi_cfg = {
    .operation = SPI_OP_MODE_SLAVE | SPI_TRANSFER_MSB | SPI_WORD_SET(8), // Mode 0
    .frequency = 1000000,  // ignored in slave, required field
    .slave = 0,
    .cs = NULL,            // Chip select handled externally
};

void main(void)
{
    if (!device_is_ready(spis_dev)) {
        LOG_ERR("SPI device not ready");
        return;
    }  

    LOG_INF("nRF9161 SPI Slave Ready");

    while (1) {
        //k_msleep(5);
        int ret = spi_transceive(spis_dev, NULL, &tx_set, &rx_set);
        LOG_INF("Received: %02X %02X %02X %02X",
                rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

        for (int i = 0; i < 4; i++) {
                tx_buf[i] = rx_buf[i] + 1;
            }

        if (ret == 0) {
            LOG_INF("Received: %02X %02X %02X %02X",
                rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

            /* Update TX buffer for next transfer */
            for (int i = 0; i < 4; i++) {
                tx_buf[i] = rx_buf[i] + 1;
            }
        } else if (ret == -EIO) {
            /* No master activity yet; optional to ignore */
            // LOG_DBG("No master activity yet");
        } else {
            continue;
            //LOG_ERR("SPI transceive failed: %d", ret);
        }

        k_msleep(10000); /* reduce CPU load */
    }
}
