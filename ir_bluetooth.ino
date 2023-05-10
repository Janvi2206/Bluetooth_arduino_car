#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(3 4); // RX, TX pins for Bluetooth module
char receivedChar;
bool obstacleDetected = false;
int irSensorPin = A0;    // Pin for IR sensor
int irThreshold = 500;   // Threshold value for IR sensor

void setup() {
  pinMode(13, OUTPUT);   // Left motors forward
  pinMode(12, OUTPUT);   // Left motors reverse
  pinMode(11, OUTPUT);   // Right motors forward
  pinMode(9, OUTPUT);    // LED
  pinMode(irSensorPin, INPUT);   // IR sensor input pin
  Serial.begin(9600);  // Serial Monitor
  bluetoothSerial.begin(9600);  // Bluetooth module
}

void loop() {
  readBluetoothData();
  checkObstacle();
  performAction();
}

void readBluetoothData() {
  if (bluetoothSerial.available()) {
    receivedChar = bluetoothSerial.read();
    Serial.println(receivedChar);
  }
}

void checkObstacle() {
  int irValue = analogRead(irSensorPin);   // Read IR sensor value
  obstacleDetected = (irValue > irThreshold);
}

void performAction() {
  if (obstacleDetected) {
    stopMotors();
  } else {
    switch (receivedChar) {
      case 'F':   // Move forward
        moveForward();
        break;
      case 'B':   // Move reverse
        moveReverse();
        break;
      case 'L':   // Turn right
        turnRight();
        break;
      case 'R':   // Turn left
        turnLeft();
        break;
      case 'W':   // Turn LED on
        turnLEDOn();
        break;
      case 'w':   // Turn LED off
        turnLEDOff();
        break;
      case 'S':   // Stop
        stopMotors();
        break;
      default:
        break;
    }
  }
}

void moveForward() {
  digitalWrite(13, HIGH);  // Left motors forward
  digitalWrite(11, HIGH);  // Right motors forward
  digitalWrite(12, LOW);   // Left motors reverse
}

void moveReverse() {
  digitalWrite(12, HIGH);  // Left motors reverse
  digitalWrite(13, LOW);   // Left motors forward
  digitalWrite(11, LOW);   // Right motors forward
}

void turnRight() {
  digitalWrite(11, LOW);   // Right motors forward
  digitalWrite(12, LOW);   // Left motors reverse
  digitalWrite(13, HIGH);  // Left motors forward
}

void turnLeft() {
  digitalWrite(13, LOW);   // Left motors forward
  digitalWrite(11, HIGH);  // Right motors forward
  digitalWrite(12, LOW);   // Left motors reverse
}

void stopMotors() {
  digitalWrite(13, LOW);   // Left motors forward
  digitalWrite(12, LOW);   // Left motors reverse
  digitalWrite(11, LOW);   // Right motors forward
}

void turnLEDOn() {
  digitalWrite(9, HIGH);   // Turn LED on
}

void turnLEDOff() {
  digitalWrite(9, LOW);   // Turn LED off
}
