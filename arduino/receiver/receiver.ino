/*
 * Arduino Nano - nRF24L01 Receiver & L298N Motor Driver Sketch
 * Architecture: nRF24L01 -> Arduino Nano (Car Receiver) -> L298N -> 4 DC Motors
 * 
 * Pinout Mapping:
 * - nRF24L01 CE   -> D9
 * - nRF24L01 CSN  -> D10
 * - nRF24L01 SCK  -> D13
 * - nRF24L01 MOSI -> D11
 * - nRF24L01 MISO -> D12
 * - nRF24L01 VCC  -> 3.3V (Add 10uF capacitor across VCC and GND!)
 * - nRF24L01 GND  -> GND
 * 
 * L298N Motor Driver Pinout:
 * - Motor Left  (ENA PWM) -> D5
 * - Motor Left  (IN1)     -> D2
 * - Motor Left  (IN2)     -> D3
 * - Motor Right (ENB PWM) -> D6
 * - Motor Right (IN3)     -> D4
 * - Motor Right (IN4)     -> D7
 * - L298N 12V / Power In  -> Car Battery Pack (+7.4V to +12V)
 * - L298N GND             -> Battery GND & Arduino GND (COMMON GROUND!)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// L298N Motor Driver Pins
const int ENA = 5;  // PWM Left Motors
const int IN1 = 2;
const int IN2 = 3;

const int ENB = 6;  // PWM Right Motors
const int IN3 = 4;
const int IN4 = 7;

// Default Motor PWM Speed (0-255)
int motor_speed = 220;

// Initialize RF24 radio object (CE, CSN)
RF24 radio(9, 10);
const byte address[6] = "CAR01";

// Safety Watchdog Timer
unsigned long last_packet_time = 0;
const unsigned long SIGNAL_TIMEOUT_MS = 500;  // Auto-stop if no packet received within 500ms

void setup() {
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initial state: Motors STOPPED
  stopMotors();

  // Initialize nRF24L01 Radio
  if (!radio.begin()) {
    Serial.println("RF_ERROR: nRF24L01 receiver hardware not responding!");
  } else {
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();  // Receiver mode
    Serial.println("RX_READY: Receiver initialized");
  }

  last_packet_time = millis();
}

void loop() {
  // Check for incoming RF packet
  if (radio.available()) {
    char cmd = 'S';
    radio.read(&cmd, sizeof(cmd));
    last_packet_time = millis();  // Reset watchdog timer

    Serial.print("RCV_CMD:");
    Serial.println(cmd);

    executeCommand(cmd);
  }

  // Fail-safe Watchdog check: Auto-STOP motors if transmitter signal drops
  if (millis() - last_packet_time > SIGNAL_TIMEOUT_MS) {
    stopMotors();
  }
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'F':
      moveForward();
      break;
    case 'B':
      moveBackward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'S':
      stopMotors();
      break;
    case 'E':
      emergencyStop();
      break;
    default:
      stopMotors();
      break;
  }
}

void moveForward() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);

  // Left Motors Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right Motors Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);

  // Left Motors Backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right Motors Backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);

  // Skid Steer Left: Left motors backward, Right motors forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);

  // Skid Steer Right: Left motors forward, Right motors backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void emergencyStop() {
  stopMotors();
  // Additional safety delay/lockout if desired
}
