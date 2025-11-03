/*
 * Teensy 4.1 SPI1 Master Example
 * Communicates with an SPI slave (e.g., nRF9161 DK)
 */

#include <SPI.h>

#define CS_PIN 10   // Chip Select pin (connect to slave's CS)

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz, MSB first, Mode 0

void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect slave

  Serial.begin(115200);
  while (!Serial) { }  // Wait until serial monitor is opened

  SPI1.begin(); // Initialize SPI1 as master
  Serial.println("SPI1 Master Ready");
}

void loop() {
  uint8_t txData[1] = {0xA1};
  uint8_t rxData[1] = {0};

  digitalWrite(CS_PIN, LOW);  // Select the slave
  //delayMicroseconds(10);       // Small delay for stability

  SPI1.beginTransaction(spiSettings);
  for (int i = 0; i < 1; i++) {
    rxData[i] = SPI1.transfer(txData[i]);
  }
  //delayMicroseconds(15);
  SPI1.endTransaction();
  
  digitalWrite(CS_PIN, HIGH); // Deselect the slave
  
  Serial.print("Sent: ");
  for (int i = 0; i < 4; i++) {
    Serial.print("0x");
    Serial.print(txData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Received: ");
  for (int i = 0; i < 4; i++) {
    Serial.print("0x");
    Serial.print(rxData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(100);  // Repeat every 1s

}
