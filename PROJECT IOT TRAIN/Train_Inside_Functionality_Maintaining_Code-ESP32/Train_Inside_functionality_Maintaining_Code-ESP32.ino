#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// OLED configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Ultrasonic pins
const int trigPin = 25;
const int echoPin = 26;

// Motor pins (L298N)
const int motorIn1 = 14;
const int motorIn2 = 12;
const int motorIn3 = 32;
const int motorIn4 = 33;

// IR sensor pin
const int irPin = 34;

// LED pin
const int ledPin = 18;

// Thresholds
const int distanceThreshold = 20;  // cm
const int irThreshold = 3800;

// Stop count
int stopCount = 0;

// Flags
bool irWasBelowThreshold = true;
bool waitingForHumanCount = false;

// MAC Address of ESP32 #2
uint8_t esp32_2_mac[] = { 0x6C, 0xC8, 0x40, 0x56, 0xA9, 0x44 };

// Callback when data is sent (new API IDF5+)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Message sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Callback when data is received (new API IDF5+)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  int receivedHumanCount;
  memcpy(&receivedHumanCount, incomingData, sizeof(receivedHumanCount));

  Serial.print("Received Human Count from ESP32 #2: ");
  Serial.println(receivedHumanCount);

  // 🔹 LED control: ON if count <= 2, OFF if > 2
  if (receivedHumanCount > 2) {
    digitalWrite(ledPin, LOW);  // OFF
  } else {
    digitalWrite(ledPin, HIGH);  // ON
  }

  waitingForHumanCount = false;  // allow loop to continue
}

void setup() {
  Serial.begin(115200);

  // Init Wi-Fi in STA mode
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, esp32_2_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize OLED
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.print("Station 0");
  display.display();

  // Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);

  // LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  // LED ON at start

  // Start motors
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
}

void loop() {
  if (waitingForHumanCount) {
    // Wait until ESP32 #2 replies
    delay(100);
    return;
  }

  // ----- Measure ultrasonic distance -----
  long duration;
  int distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;  // cm

  // ----- Read IR sensor -----
  int irValue = analogRead(irPin);

  // ----- Motor control -----
  bool stopMotor = false;

  if (distance < distanceThreshold) {
    stopMotor = true;
  }

  if (irValue > irThreshold && irWasBelowThreshold) {
    stopMotor = true;
    stopCount++;

    Serial.print("Stop Count: ");
    Serial.print(stopCount);
    Serial.print(" | IR: ");
    Serial.print(irValue);
    Serial.print(" | Distance: ");
    Serial.println(distance);

    // Update OLED
    display.clearDisplay();
    display.setCursor(0, 20);
    display.print("Station ");
    display.print(stopCount);
    display.display();

    // 🔹 Send "START" to ESP32 #2
    const char *msg = "START";
    esp_now_send(esp32_2_mac, (uint8_t *)msg, strlen(msg) + 1);
    waitingForHumanCount = true;
  }

  irWasBelowThreshold = (irValue < irThreshold);

  if (irValue < irThreshold && distance > distanceThreshold) {
    stopMotor = false;
  }

  // Apply motor states to both motors
  if (stopMotor) {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, LOW);
  } else {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
  }

  Serial.print("Stop Count: ");
  Serial.print(stopCount);
  Serial.print(" | IR: ");
  Serial.print(irValue);
  Serial.print(" | Distance: ");
  Serial.println(distance);

  delay(100);
}