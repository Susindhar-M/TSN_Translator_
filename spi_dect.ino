#include <SPI.h>

// Define the same packet structure as the nRF9161
#define SOP_CHAR 0x7E
#define PAYLOAD_LEN 32
#define PACKET_LEN (1 + 1 + PAYLOAD_LEN + 1)

struct spi_packet {
    uint8_t sop;
    uint8_t len;
    uint8_t payload[PAYLOAD_LEN];
    uint8_t checksum;
};

const int RPIN = 11;
// SPI configuration
const int CS_PIN = 10; // Default CS pin for SPI on Teensy 4.1
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // 4 MHz, MSB, Mode 0

// Global packet buffers
spi_packet tx_packet;
spi_packet rx_packet;

uint32_t message_counter = 0;

// Helper to calculate checksum
uint8_t calculate_checksum(const spi_packet* packet) {
    uint8_t chk = packet->sop ^ packet->len;
    for (int i = 0; i < packet->len; i++) {
        chk ^= packet->payload[i];
    }
    return chk;
}

// Helper to print packet contents for debugging
void print_packet(const char* prefix, const spi_packet* packet) {
    Serial.printf("%s SOP: 0x%02X, Len: %d, Payload: ", prefix, packet->sop, packet->len);
    for (int i = 0; i < packet->len; i++) {
        Serial.printf("%02X ", packet->payload[i]);
    }
    Serial.printf(", Checksum: 0x%02X\n", packet->checksum);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        // Wait for serial monitor to open
    }
    Serial.println("Teensy 4.1 SPI Master Initialized");
    pinMode(RPIN, INPUT);
    while (digitalRead(RPIN) == LOW); // wait for slave ready
    delay(5); // small margin
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH); // Deselect slave initially

    SPI1.begin();
}

void loop() {
    // 1. Prepare the packet to transmit
    memset(&tx_packet, 0, sizeof(tx_packet));
    tx_packet.sop = SOP_CHAR;
    tx_packet.len = sizeof(message_counter); // We are just sending a 4-byte counter
    
    // Copy counter value into the payload
    memcpy(tx_packet.payload, &message_counter, sizeof(message_counter));
    
    tx_packet.checksum = calculate_checksum(&tx_packet);

    Serial.printf("\n--- Transaction %u ---\n", message_counter);
    print_packet("Master TX:", &tx_packet);

    // 2. Perform the SPI transaction
    SPI1.beginTransaction(spiSettings);
    digitalWrite(CS_PIN, LOW); // Select the slave

    // SPI is full-duplex, so we send and receive at the same time.
    // We transfer the entire packet structure.
    uint8_t* tx_ptr = (uint8_t*)&tx_packet;
    uint8_t* rx_ptr = (uint8_t*)&rx_packet;
    for (int i = 0; i < sizeof(spi_packet); i++) {
        rx_ptr[i] = SPI1.transfer(tx_ptr[i]);
    }

    digitalWrite(CS_PIN, HIGH); // Deselect the slave
    SPI1.endTransaction();

    // 3. Process the received packet
    print_packet("Master RX:", &rx_packet);

    if (rx_packet.sop == SOP_CHAR) {
        uint8_t expected_checksum = calculate_checksum(&rx_packet);
        if (rx_packet.checksum == expected_checksum) {
            if (rx_packet.len > 0) {
                Serial.println("  -> Valid packet received from slave!");
                // You can now process the data in rx_packet.payload
            } else {
                Serial.println("  -> Valid empty packet received from slave (no new data).");
            }
        } else {
            Serial.printf("  -> Checksum mismatch! Got 0x%02X, expected 0x%02X\n", rx_packet.checksum, expected_checksum);
        }
    } else {
        Serial.printf("  -> Invalid SOP received from slave! Got 0x%02X\n", rx_packet.sop);
    }

    message_counter++;
    delay(2000); // Pace the transmissions
}
