#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>

#include <string.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>

LOG_MODULE_REGISTER(dect_spi_gw, LOG_LEVEL_INF);

/* --- SPI Configuration --- */
#define SPI_NODE DT_NODELABEL(arduino_spi)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);


static const struct spi_config spi_cfg = {
    .operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8),
    .slave = 0, // This is the 'reg = <0>' from the devicetree
};


// Define a simple framing protocol
#define SOP_CHAR 0x7E
#define PAYLOAD_LEN 32
#define PACKET_LEN (1 + 1 + PAYLOAD_LEN + 1) // SOP, Len, Payload, Checksum

struct spi_packet {
    uint8_t sop;
    uint8_t len;
    uint8_t payload[PAYLOAD_LEN];
    uint8_t checksum;
} __attribute__((packed));

// SPI TX and RX buffers
static struct spi_packet spi_tx_packet;
static struct spi_packet spi_rx_packet;

static struct spi_buf spi_tx_buf = {.buf = &spi_tx_packet,.len = sizeof(spi_tx_packet)};
static struct spi_buf spi_rx_buf = {.buf = &spi_rx_packet,.len = sizeof(spi_rx_packet)};
static const struct spi_buf_set spi_tx_set = {.buffers = &spi_tx_buf,.count = 1};
static const struct spi_buf_set spi_rx_set = {.buffers = &spi_rx_buf,.count = 1};

/* --- Message Queue Configuration --- */
// Queue for data from SPI to be sent over DECT
K_MSGQ_DEFINE(spi_to_dect_msgq, PAYLOAD_LEN, 10, 4);
// Queue for data from DECT to be sent over SPI
K_MSGQ_DEFINE(dect_to_spi_msgq, PAYLOAD_LEN, 10, 4);

/* --- DECT NR+ Configuration (from hello_dect sample) --- */
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

// Callback for when a DECT packet is received (Physical Data Channel)
static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt) {
    LOG_INF("DECT RX: Received %d bytes", evt->len);
    // Put received data into the queue for the SPI thread to pick up
    // Use K_NO_WAIT as we are in an interrupt context
    if (k_msgq_put(&dect_to_spi_msgq, evt->data, K_NO_WAIT)!= 0) {
        LOG_WRN("dect_to_spi_msgq is full. Dropping packet.");
    }
}

// Boilerplate DECT callbacks from the sample
static void on_op_complete(const struct nrf_modem_dect_phy_op_complete_event *evt) {
    k_sem_give(&dect_op_sem);
}

static void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt) {
    switch (evt->id) {
    case NRF_MODEM_DECT_PHY_EVT_PDC:
        on_pdc(&evt->pdc);
        break;
    case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
        on_op_complete(&evt->op_complete);
        break;
    // Other events can be handled here if needed
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
       .handle = 0,.network_id = CONFIG_NETWORK_ID,.carrier = CONFIG_CARRIER,
       .lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MAX,
       .phy_header = (union nrf_modem_dect_phy_hdr *)&header,
       .data = data,.data_size = data_len,
    };

    return nrf_modem_dect_phy_tx(&tx_params);
}

static int dect_receive(void) {
    struct nrf_modem_dect_phy_rx_params rx_params = {
       .handle = 1,.network_id = CONFIG_NETWORK_ID,
       .mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,.carrier = CONFIG_CARRIER,
       .duration = CONFIG_RX_PERIOD_S * 1000 * 1000, // In microseconds
       .filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
       .filter.is_short_network_id_used = 1,
       .filter.receiver_identity = 0, // Listen for everything
    };
    return nrf_modem_dect_phy_rx(&rx_params);
}

/* --- Threads --- */

// Thread to handle SPI communication
void spi_thread_entry(void) {
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return;
    }
    LOG_INF("SPI Slave Ready");

    // Prepare an initial empty packet to send to the master
    memset(&spi_tx_packet, 0, sizeof(spi_tx_packet));
    spi_tx_packet.sop = SOP_CHAR;
    spi_tx_packet.len = 0;
    spi_tx_packet.checksum = calculate_checksum(&spi_tx_packet);

    while (1) {
        // This is a blocking call, waiting for the master to initiate a transaction
        int ret = spi_transceive(spi_dev, &spi_cfg, &spi_tx_set, &spi_rx_set);

        if (ret == 0) {
            
            // Transaction complete, process received data
            if (spi_rx_packet.sop == SOP_CHAR) {
                uint8_t expected_checksum = calculate_checksum(&spi_rx_packet);
                if (spi_rx_packet.checksum == expected_checksum) {
                    LOG_INF("SPI RX: Valid packet received, len=%d", spi_rx_packet.len);
                    // Put the valid payload into the message queue for the DECT thread
                    if (k_msgq_put(&spi_to_dect_msgq, spi_rx_packet.payload, K_NO_WAIT)!= 0) {
                        LOG_WRN("spi_to_dect_msgq is full. Dropping packet.");
                    }
                } else {
                    LOG_WRN("SPI RX: Checksum mismatch. Got 0x%02X, expected 0x%02X",
                            spi_rx_packet.checksum, expected_checksum);
                }
            } else {
                LOG_WRN("SPI RX: Invalid SOP. Got 0x%02X", spi_rx_packet.sop);
            }
        } else if (ret!= -EIO) { // -EIO means no master activity, which is normal
            LOG_ERR("SPI transceive failed: %d", ret);
        }

        // Prepare the next packet to be sent to the master
        // Check if there's any data from DECT waiting to be sent
        if (k_msgq_get(&dect_to_spi_msgq, spi_tx_packet.payload, K_NO_WAIT) == 0) {
            spi_tx_packet.len = PAYLOAD_LEN; // Assume full payload for now
            LOG_INF("SPI TX: Preparing DECT data for master");
        } else {
            // No data from DECT, prepare an empty packet
            spi_tx_packet.len = 0;
            memset(spi_tx_packet.payload, 0, PAYLOAD_LEN);
        }
        spi_tx_packet.sop = SOP_CHAR;
        spi_tx_packet.checksum = calculate_checksum(&spi_tx_packet);
    }
}

// Thread to handle DECT NR+ communication
void dect_thread_entry(void) {
    int err;
    static uint8_t dect_tx_payload[PAYLOAD_LEN];

    // Initialize modem and DECT PHY
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return;
    }
    err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
    err |= nrf_modem_dect_phy_init();
    err |= nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
    if (err) {
        LOG_ERR("DECT PHY init/activate failed, err %d", err);
        dect_exit = true;
    }
    if (dect_exit) return;

    hwinfo_get_device_id((void *)&device_id, sizeof(device_id));
    LOG_INF("DECT NR+ PHY initialized, device ID: %d", device_id);

    while (1) {
        // 1. Check for data from SPI and transmit it
        // This is a blocking call, thread will sleep until a message arrives
        if (k_msgq_get(&spi_to_dect_msgq, dect_tx_payload, K_FOREVER) == 0) {
            LOG_INF("DECT TX: Transmitting data from SPI...");
            err = dect_transmit(dect_tx_payload, PAYLOAD_LEN);
            if (err) {
                LOG_ERR("DECT transmission failed, err %d", err);
            } else {
                // Wait for TX operation to complete
                k_sem_take(&dect_op_sem, K_FOREVER);
            }
        }

        // 2. Enter receive mode to listen for incoming data
        err = dect_receive();
        if (err) {
            LOG_ERR("DECT reception failed, err %d", err);
        } else {
            // Wait for RX operation to complete (or timeout)
            k_sem_take(&dect_op_sem, K_FOREVER);
        }
    }
}

// Thread definitions
K_THREAD_DEFINE(spi_tid, 2048, spi_thread_entry, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(dect_tid, 4096, dect_thread_entry, NULL, NULL, NULL, 8, 0, 0);

// Main function is now just a placeholder as threads do all the work
int main(void) {
    LOG_INF("Teensy-nRF9161 DECT/SPI Gateway Started");
	return 0;
}