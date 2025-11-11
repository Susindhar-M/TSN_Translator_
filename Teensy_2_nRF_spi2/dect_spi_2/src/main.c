#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>

#define RPIN 12
LOG_MODULE_REGISTER(dect_spi_gw, LOG_LEVEL_INF);

/* --- SPI Configuration --- */
#define SPI_NODE DT_NODELABEL(spi2)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static const struct spi_config spi_cfg = {
    .operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8),
    .slave = 0,
};

const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

// Define a simple framing protocol
#define SOP_CHAR 0x7E
#define PAYLOAD_LEN 32
#define PACKET_LEN (1 + 1 + PAYLOAD_LEN + 1)

struct spi_packet {
    uint8_t sop;
    uint8_t len;
    uint8_t payload[PAYLOAD_LEN];
    uint8_t checksum;
} __attribute__((packed));

struct data_packet {
    uint8_t buf[PAYLOAD_LEN];
    size_t len;
} __attribute__((packed));

/* Dect PHY config parameters. */
static struct nrf_modem_dect_phy_config_params dect_phy_config_params = {
    .band_group_index = ((CONFIG_CARRIER >= 525 && CONFIG_CARRIER <= 551)) ? 1 : 0,
    .harq_rx_process_count = 4,
    .harq_rx_expiry_time_us = 5000000,
};

// SPI TX and RX buffers
static struct spi_packet spi_tx_packet;
static struct spi_packet spi_rx_packet;

static struct spi_buf spi_tx_buf = {.buf = &spi_tx_packet,.len = sizeof(spi_tx_packet)};
static struct spi_buf spi_rx_buf = {.buf = &spi_rx_packet,.len = sizeof(spi_rx_packet)};
static const struct spi_buf_set spi_tx_set = {.buffers = &spi_tx_buf,.count = 1};
static const struct spi_buf_set spi_rx_set = {.buffers = &spi_rx_buf,.count = 1};

/* --- Message Queue Configuration --- */
K_MSGQ_DEFINE(spi_to_dect_msgq, sizeof(struct data_packet), 10, 4);
K_MSGQ_DEFINE(dect_to_spi_msgq, sizeof(struct data_packet), 10, 4);

/* --- DECT NR+ Configuration --- */
#define DATA_LEN_MAX 32
static bool dect_exit;
static uint16_t device_id;
K_SEM_DEFINE(dect_op_sem, 0, 1);

// DECT PHY header structure
struct phy_ctrl_field_common {
    uint32_t packet_length : 4;
    uint32_t packet_length_type : 1;
    uint32_t header_format : 3;
    uint32_t short_network_id : 8;
    uint32_t transmitter_id_hi : 8;
    uint32_t transmitter_id_lo : 8;
    uint32_t df_mcs : 3;
    uint32_t reserved : 1;
    uint32_t transmit_power : 4;
    uint32_t pad : 24;
};

/* --- Helper Functions --- */
static uint8_t calculate_checksum(const struct spi_packet *packet) {
    uint8_t chk = packet->sop ^ packet->len;
    for (int i = 0; i < packet->len; i++) {
        chk ^= packet->payload[i];
    }
    return chk;
}

/* --- DECT NR+ Callbacks and Functions --- */

// Callback after init operation
static void on_init(const struct nrf_modem_dect_phy_init_event *evt) {
    if (evt->err) {
        LOG_ERR("DECT Init failed, err %d", evt->err);
        dect_exit = true;
    } else {
        LOG_INF("DECT Init successful");
    }
    k_sem_give(&dect_op_sem);
}

// Callback after configure operation
static void on_configure(const struct nrf_modem_dect_phy_configure_event *evt) {
    if (evt->err) {
        LOG_ERR("DECT Configure failed, err %d", evt->err);
        dect_exit = true;
    } else {
        LOG_INF("DECT Configure successful");
    }
    k_sem_give(&dect_op_sem);
}

// Callback after activate operation
static void on_activate(const struct nrf_modem_dect_phy_activate_event *evt) {
    if (evt->err) {
        LOG_ERR("DECT Activate failed, err %d", evt->err);
        dect_exit = true;
    } else {
        LOG_INF("DECT Activate successful");
    }
    k_sem_give(&dect_op_sem);
}

static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt) {
    LOG_INF("DECT RX: Received %d bytes", evt->len);
    LOG_HEXDUMP_INF(evt->data, evt->len, "DECT RX Payload:");

    if (evt->len == 0) {
        return;
    }

    if (evt->len > PAYLOAD_LEN) {
        LOG_WRN("DECT packet too large (%d), truncating to %d", evt->len, PAYLOAD_LEN);
    }

    struct data_packet msg_to_spi;
    msg_to_spi.len = (evt->len > PAYLOAD_LEN) ? PAYLOAD_LEN : evt->len;
    memcpy(msg_to_spi.buf, evt->data, msg_to_spi.len);

    if (k_msgq_put(&dect_to_spi_msgq, &msg_to_spi, K_NO_WAIT) != 0) {
        LOG_WRN("dect_to_spi_msgq is full. Dropping packet.");
    }
}

static void on_op_complete(const struct nrf_modem_dect_phy_op_complete_event *evt) {
    k_sem_give(&dect_op_sem);
}

static void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt) {
    switch (evt->id) {
    case NRF_MODEM_DECT_PHY_EVT_INIT:
        on_init(&evt->init);
        break;
    case NRF_MODEM_DECT_PHY_EVT_CONFIGURE:
        on_configure(&evt->configure);
        break;
    case NRF_MODEM_DECT_PHY_EVT_ACTIVATE:
        on_activate(&evt->activate);
        break;
    case NRF_MODEM_DECT_PHY_EVT_PDC:
        on_pdc(&evt->pdc);
        break;
    case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
        on_op_complete(&evt->op_complete);
        break;
    default:
        break;
    }
}

static int dect_transmit(void *data, size_t data_len) {
    struct phy_ctrl_field_common header = {
       .header_format = 0x0,.packet_length_type = 0x0,.packet_length = 0x01,
       .short_network_id = (CONFIG_NETWORK_ID & 0xff),.transmitter_id_hi = (device_id >> 8),
       .transmitter_id_lo = (device_id & 0xff),.transmit_power = CONFIG_TX_POWER,
       .reserved = 0,.df_mcs = CONFIG_MCS,
    };

    struct nrf_modem_dect_phy_tx_params tx_params = {
        .start_time = 0,
        .handle = 0,
        .network_id = CONFIG_NETWORK_ID,
        .phy_type = 0,
        .lbt_rssi_threshold_max = 0,
        .carrier = CONFIG_CARRIER,
        .lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MAX,
        .phy_header = (union nrf_modem_dect_phy_hdr *)&header,
        .data = data,
        .data_size = data_len,
    };

    return nrf_modem_dect_phy_tx(&tx_params);
}

static int dect_receive(void) {
    struct nrf_modem_dect_phy_rx_params rx_params = {
        .start_time = 0,
        .handle = 1,
        .network_id = CONFIG_NETWORK_ID,
        .mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
        .rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
        .link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
        .rssi_level = -60,
        .carrier = CONFIG_CARRIER,
        .duration = CONFIG_RX_PERIOD_S * MSEC_PER_SEC * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
        .filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
        .filter.is_short_network_id_used = 1,
        .filter.receiver_identity = 0,
    };
    return nrf_modem_dect_phy_rx(&rx_params);
}

/* --- Threads --- */

void spi_thread_entry(void) {
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return;
    }
    LOG_INF("SPI Slave Ready");
    gpio_pin_set(gpio_dev, RPIN, 1);
    
    memset(&spi_tx_packet, 0, sizeof(spi_tx_packet));
    spi_tx_packet.sop = SOP_CHAR;
    spi_tx_packet.len = 0;
    spi_tx_packet.checksum = calculate_checksum(&spi_tx_packet);

    struct data_packet msg_q_packet;

    while (1) {
        int ret = spi_transceive(spi_dev, &spi_cfg, &spi_tx_set, &spi_rx_set);
        
        if (ret >= 0) {
            LOG_INF("Received_from_teensy: %02X %02X %02X %02X",
                     spi_rx_packet.payload[0], spi_rx_packet.payload[1], 
                     spi_rx_packet.payload[2], spi_rx_packet.payload[3]);
            
            if (spi_rx_packet.sop == SOP_CHAR) {
                uint8_t expected_checksum = calculate_checksum(&spi_rx_packet);
                if (spi_rx_packet.checksum == expected_checksum) {
                    LOG_INF("SPI RX: Valid packet received, len=%d", spi_rx_packet.len);
                    LOG_HEXDUMP_INF(spi_rx_packet.payload, spi_rx_packet.len, 
                                   "SPI RX Payload (from Teensy):");

                    if (spi_rx_packet.len > 0) {
                        msg_q_packet.len = spi_rx_packet.len;
                        memcpy(msg_q_packet.buf, spi_rx_packet.payload, msg_q_packet.len);

                        if (k_msgq_put(&spi_to_dect_msgq, &msg_q_packet, K_NO_WAIT) != 0) {
                            LOG_WRN("spi_to_dect_msgq is full. Dropping packet.");
                        }
                    }
                } else {
                    LOG_WRN("SPI RX: Checksum mismatch. Got 0x%02X, expected 0x%02X",
                            spi_rx_packet.checksum, expected_checksum);
                }
            } else {
                LOG_WRN("SPI RX: Invalid SOP. Got 0x%02X", spi_rx_packet.sop);
            }
        } else if (ret != -EIO) {
            LOG_ERR("SPI transceive failed: %d", ret);
        }

        if (k_msgq_get(&dect_to_spi_msgq, &msg_q_packet, K_NO_WAIT) == 0) {
            spi_tx_packet.len = msg_q_packet.len;
            memcpy(spi_tx_packet.payload, msg_q_packet.buf, msg_q_packet.len);

            if (msg_q_packet.len < PAYLOAD_LEN) {
                memset(spi_tx_packet.payload + msg_q_packet.len, 0, 
                       PAYLOAD_LEN - msg_q_packet.len);
            }
            
            LOG_INF("SPI TX: Preparing DECT data (len=%d) for master", spi_tx_packet.len);
            LOG_HEXDUMP_INF(spi_tx_packet.payload, spi_tx_packet.len, 
                           "SPI TX Payload (to Teensy):");
        } else {
            spi_tx_packet.len = 0;
            memset(spi_tx_packet.payload, 0, PAYLOAD_LEN);
        }
        spi_tx_packet.sop = SOP_CHAR;
        spi_tx_packet.checksum = calculate_checksum(&spi_tx_packet);
    }
}

void dect_thread_entry(void) {
    int err;
    static struct data_packet dect_tx_msg;

    LOG_INF("DECT thread starting initialization...");

    // Initialize modem library
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return;
    }
    LOG_INF("Modem library initialized");

    // Set event handler
    err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
    if (err) {
        LOG_ERR("Event handler set failed, err %d", err);
        return;
    }
    LOG_INF("Event handler set");

    // Initialize DECT PHY
    err = nrf_modem_dect_phy_init();
    if (err) {
        LOG_ERR("DECT PHY init call failed, err %d", err);
        return;
    }
    LOG_INF("Waiting for DECT init to complete...");
    k_sem_take(&dect_op_sem, K_FOREVER);
    if (dect_exit) return;

    // Configure DECT PHY
    err = nrf_modem_dect_phy_configure(&dect_phy_config_params);
    if (err) {
        LOG_ERR("DECT PHY configure call failed, err %d", err);
        return;
    }
    LOG_INF("Waiting for DECT configure to complete...");
    k_sem_take(&dect_op_sem, K_FOREVER);
    if (dect_exit) return;

    // Activate DECT PHY
    err = nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
    if (err) {
        LOG_ERR("DECT PHY activate call failed, err %d", err);
        return;
    }
    LOG_INF("Waiting for DECT activate to complete...");
    k_sem_take(&dect_op_sem, K_FOREVER);
    if (dect_exit) return;

    hwinfo_get_device_id((void *)&device_id, sizeof(device_id));
    LOG_INF("DECT NR+ PHY fully initialized, device ID: %d", device_id);

    while (1) {
        // Check for data from SPI and transmit it
        if (k_msgq_get(&spi_to_dect_msgq, &dect_tx_msg, K_NO_WAIT) == 0) {
            LOG_INF("DECT TX: Transmitting %d bytes from SPI...", dect_tx_msg.len);
            LOG_HEXDUMP_INF(dect_tx_msg.buf, dect_tx_msg.len, "DECT TX Payload:");

            err = dect_transmit(dect_tx_msg.buf, dect_tx_msg.len);
            
            if (err) {
                LOG_ERR("DECT transmission failed, err %d", err);
            } else {
                k_sem_take(&dect_op_sem, K_FOREVER);
            }
        }

        // Enter receive mode to listen for incoming data
        err = dect_receive();
        if (err) {
            LOG_ERR("DECT reception failed, err %d", err);
        } else {
            k_sem_take(&dect_op_sem, K_FOREVER);
        }
        
        k_msleep(10);
    }
}

K_THREAD_DEFINE(spi_tid, 2048, spi_thread_entry, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(dect_tid, 4096, dect_thread_entry, NULL, NULL, NULL, 8, 0, 0);

int main(void) {
    LOG_INF("Teensy-nRF9161 DECT/SPI Gateway Started");
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("READY pin device not ready");
        return 0;
    }
    gpio_pin_configure(gpio_dev, RPIN, GPIO_OUTPUT_INACTIVE);
    return 0;
}