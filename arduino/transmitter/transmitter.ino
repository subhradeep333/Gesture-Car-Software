/*
 * Arduino Nano - nRF24L01 Transmitter Sketch
 * Architecture: Laptop USB Serial -> Arduino Nano -> nRF24L01
 * 
 * Hardware Pinout:
 * - nRF24L01 CE   -> D9
 * - nRF24L01 CSN  -> D10
 * - nRF24L01 SCK  -> D13
 * - nRF24L01 MOSI -> D11
 * - nRF24L01 MISO -> D12
 * - nRF24L01 VCC  -> 3.3V (Add 10uF capacitor across VCC and GND!)
 * - nRF24L01 GND  -> GND
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Initialize RF24 radio object (CE, CSN)
RF24 radio(9, 10);

// Address pipe (5 bytes string, must match receiver)
const byte address[6] = "CAR01";

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to open
  }

  // Initialize nRF24L01 Radio
  if (!radio.begin()) {
    Serial.println("RF_ERROR: nRF24L01 hardware not responding!");
  } else {
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_HIGH);    // RF power level: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate(RF24_250KBPS);   // Lower data rate increases range and reliability
    radio.setRetries(3, 5);             // (delay_count, retry_count)
    radio.stopListening();             // Transmitter mode
    Serial.println("TX_READY: Transmitter initialized");
  }
}

void loop() {
  // Read incoming command from laptop USB Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Filter valid single-byte commands
    if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S' || cmd == 'E') {
      // Transmit single-character payload over RF
      bool tx_success = radio.write(&cmd, sizeof(cmd));

      if (tx_success) {
        Serial.print("ACK:");
        Serial.println(cmd);
      } else {
        Serial.print("RF_FAIL:");
        Serial.println(cmd);
      }
    }
  }
}
