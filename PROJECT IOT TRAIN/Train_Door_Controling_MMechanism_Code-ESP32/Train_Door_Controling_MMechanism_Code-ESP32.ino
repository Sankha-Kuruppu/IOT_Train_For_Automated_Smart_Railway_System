#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// Pins
const int irPinA = 34;  // IR sensor A (increment)
const int irPinB = 35;  // IR sensor B (decrement)
const int trigPin = 25;
const int echoPin = 26;
const int servoPin = 27;

Servo myServo;

// Variables
int humanCount = 0;
int servoAngle = 0;
unsigned long lastIrTime = 0;
const unsigned long servoCloseDelay = 5000;
bool servoOpen = false;
bool lastIrAHigh = false;
bool lastIrBHigh = false;
bool processActive = false;  // run only when ESP32 #1 triggers

// MAC of ESP32 #1
uint8_t esp32_1_mac[] = { 0x6C, 0xC8, 0x40, 0x90, 0x13, 0xA0 };

// ✅ Send callback (new API)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Message sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ✅ Receive callback (new API)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  char msg[16];
  memcpy(msg, incomingData, len);
  msg[len] = '\0';  // null-terminate

  if (strcmp(msg, "START") == 0) {
    Serial.println("Received START from ESP32 #1");
    processActive = true;
    servoAngle = 180;
    myServo.write(servoAngle);
    servoOpen = true;
    lastIrTime = millis();
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, esp32_1_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(irPinA, INPUT);
  pinMode(irPinB, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin, 500, 2500);
  myServo.write(0);  // keep closed until triggered
}

long readUltrasonicCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  if (!processActive) {
    delay(100);
    return;
  }

  int irValueA = analogRead(irPinA);
  int irValueB = analogRead(irPinB);
  long distance = readUltrasonicCM();
  unsigned long currentTime = millis();

  if (servoOpen) {
    // IR Sensor A → increment
    if (irValueA > 2000 && !lastIrAHigh) {
      humanCount++;
      lastIrTime = currentTime;
      lastIrAHigh = true;
      Serial.println("IR A detected! Human count incremented.");
    } else if (irValueA <= 2000) {
      lastIrAHigh = false;
    }

    // IR Sensor B → decrement
    if (irValueB > 2000 && !lastIrBHigh) {
      if (humanCount > 0) humanCount--;  // prevent negative count
      lastIrTime = currentTime;
      lastIrBHigh = true;
      Serial.println("IR B detected! Human count decremented.");
    } else if (irValueB <= 2000) {
      lastIrBHigh = false;
    }
  }

  // Close servo + send back count
  if (servoOpen && (currentTime - lastIrTime > servoCloseDelay)) {
    servoAngle = 0;
    myServo.write(servoAngle);
    servoOpen = false;
    processActive = false;  // process finished

    // 🔹 Send back humanCount to ESP32 #1
    esp_now_send(esp32_1_mac, (uint8_t *)&humanCount, sizeof(humanCount));
    Serial.print("Process finished. Sent HumanCount back to ESP32 #1: ");
    Serial.println(humanCount);
  }

  Serial.print("Servo Angle: ");
  Serial.print(servoAngle);
  Serial.print(" | IR A: ");
  Serial.print(irValueA);
  Serial.print(" | IR B: ");
  Serial.print(irValueB);
  Serial.print(" | Ultrasonic: ");
  Serial.print(distance);
  Serial.print(" cm");
  Serial.print(" | Human Count: ");
  Serial.println(humanCount);

  delay(200);
}
