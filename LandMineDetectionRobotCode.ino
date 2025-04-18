// Blynk configuration
#define BLYNK_TEMPLATE_ID           "TMPL6h_Nzp5sN"
#define BLYNK_TEMPLATE_NAME         "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "D2S8VufW32sWqDMuWWUapDlGMT2iGRfz"

// Include necessary libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Wi-Fi credentials
char ssid[] = "Tanush's Phone";
char pass[] = "TANUSH007$";

// Motor control pins
#define IN1 14  // Left Front Motor
#define IN2 27  // Left Front Motor
#define IN3 26  // Right Front Motor
#define IN4 25  // Right Front Motor
#define IN5 32  // Left Back Motor
#define IN6 33  // Left Back Motor
#define IN7 18  // Right Back Motor
#define IN8 19  // Right Back Motor

// Sensor pins
#define KY038_ANALOG_PIN 34  // KY-038 analog output on GPIO 34 (ADC1_6)
#define KY038_THRESHOLD 500  // Adjust this threshold based on beep sound level (0-4095)

// GPS configuration
#define RXD2 16  // GPS TX to ESP32 GPIO 16
#define TXD2 17  // GPS RX to ESP32 GPIO 17
HardwareSerial SerialGPS(1);  // Use Serial1 for GPS
TinyGPSPlus gps;

// Variables for joystick and speed
int xValue = 0;  // X-axis (left/right) from V4
int yValue = 0;  // Y-axis (forward/backward) from V5
int motorSpeed = 255;  // Default speed (0-255), controlled by V6

// Metal detection variables
bool metalDetected = false;
unsigned long lastDetectionTime = 0;
const unsigned long debounceDelay = 500; // 500ms debounce

// GPS update interval
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 5000; // Update Blynk every 5 seconds

// Function to control a motor with speed (using PWM)
void controlMotor(int in1, int in2, int speed, String direction) {
  if (speed == 0 || direction == "STOP") {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  } else if (direction == "FORWARD") {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (direction == "BACKWARD") {
    analogWrite(in1, 0);
    analogWrite(in2, speed);
  }
}

// Function to move the robot based on direction and speed
void moveRobot(String direction) {
  if (direction == "FORWARD") {
    controlMotor(IN1, IN2, motorSpeed, "FORWARD");  // Left Front
    controlMotor(IN3, IN4, motorSpeed, "FORWARD");  // Right Front
    controlMotor(IN5, IN6, motorSpeed, "FORWARD");  // Left Back
    controlMotor(IN7, IN8, motorSpeed, "FORWARD");  // Right Back
  } else if (direction == "BACKWARD") {
    controlMotor(IN1, IN2, motorSpeed, "BACKWARD");  // Left Front
    controlMotor(IN3, IN4, motorSpeed, "BACKWARD");  // Right Front
    controlMotor(IN5, IN6, motorSpeed, "BACKWARD");  // Left Back
    controlMotor(IN7, IN8, motorSpeed, "BACKWARD");  // Right Back
  } else if (direction == "LEFT") {
    controlMotor(IN1, IN2, motorSpeed, "BACKWARD");  // Left Front
    controlMotor(IN3, IN4, motorSpeed, "FORWARD");   // Right Front
    controlMotor(IN5, IN6, motorSpeed, "BACKWARD");  // Left Back
    controlMotor(IN7, IN8, motorSpeed, "FORWARD");   // Right Back
  } else if (direction == "RIGHT") {
    controlMotor(IN1, IN2, motorSpeed, "FORWARD");   // Left Front
    controlMotor(IN3, IN4, motorSpeed, "BACKWARD");  // Right Front
    controlMotor(IN5, IN6, motorSpeed, "FORWARD");   // Left Back
    controlMotor(IN7, IN8, motorSpeed, "BACKWARD");  // Right Back
  } else if (direction == "STOP") {
    controlMotor(IN1, IN2, 0, "STOP");  // Left Front
    controlMotor(IN3, IN4, 0, "STOP");  // Right Front
    controlMotor(IN5, IN6, 0, "STOP");  // Left Back
    controlMotor(IN7, IN8, 0, "STOP");  // Right Back
  }
}

// Update robot movement based on X and Y values
void updateRobotMovement() {
  if (yValue > 150) {  // Forward
    moveRobot("FORWARD");
  } else if (yValue < -150) {  // Backward
    moveRobot("BACKWARD");
  } else if (xValue > 150) {  // Right
    moveRobot("RIGHT");
  } else if (xValue < -150) {  // Left
    moveRobot("LEFT");
  } else {  // Stop
    moveRobot("STOP");
  }
}

// Check for metal detection via KY-038 sound
void checkMetalDetection() {
  int soundLevel = analogRead(KY038_ANALOG_PIN); // Read sound level on GPIO 34
  unsigned long currentTime = millis();

  if (soundLevel > KY038_THRESHOLD && !metalDetected && (currentTime - lastDetectionTime >= debounceDelay)) {
    metalDetected = true;
    lastDetectionTime = currentTime;
    Serial.println("Metal detected! Sound level: " + String(soundLevel));

    // Get current GPS location
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Update coordinates on Blynk widgets only when metal is detected
    if (gps.location.isValid()) {
      Blynk.virtualWrite(V7, String(latitude, 6));  // Latitude to V7
      Blynk.virtualWrite(V8, String(longitude, 6)); // Longitude to V8
      Serial.println("Metal detected! Coordinates updated. Location: " + String(latitude, 6) + ", " + String(longitude, 6));
    }
  } else if (soundLevel <= KY038_THRESHOLD) {
    metalDetected = false; // Reset when sound level drops
  }
}

// Blynk virtual pin handler for X-axis (V4)
BLYNK_WRITE(V4) {
  xValue = param.asInt();  // Read X-axis value (left/right)
  updateRobotMovement();   // Update robot movement
}

// Blynk virtual pin handler for Y-axis (V5)
BLYNK_WRITE(V5) {
  yValue = param.asInt();  // Read Y-axis value (forward/backward)
  updateRobotMovement();   // Update robot movement
}

// Blynk virtual pin handler for speed control (V6)
BLYNK_WRITE(V6) {
  motorSpeed = param.asInt();  // Read speed value (0-255)
  Serial.print("Speed set to: ");
  Serial.println(motorSpeed);
  updateRobotMovement();  // Update robot movement with new speed
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(100);  // Small delay to allow Serial to initialize

  // Initialize GPS serial
  SerialGPS.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Stop all motors initially
  moveRobot("STOP");

  // Add a delay to ensure the ESP32 stabilizes before connecting to Wi-Fi
  Serial.println("Starting Wi-Fi connection...");
  delay(2000);  // 2-second delay to stabilize power

  // Connect to Wi-Fi manually
  WiFi.begin(ssid, pass);
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
    return;
  }

  // Configure Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  if (Blynk.connect(10000)) {
    Serial.println("Blynk connected");
  } else {
    Serial.println("Failed to connect to Blynk");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.run();  // Run Blynk only if connected

    // Parse GPS data
    while (SerialGPS.available() > 0) {
      if (gps.encode(SerialGPS.read()) && gps.location.isValid()) {
        // Update coordinates on V7 and V8 every updateInterval
        if (millis() - lastUpdateTime >= updateInterval) {
          Blynk.virtualWrite(V7, String(gps.location.lat(), 6));  // Latitude to V7
          Blynk.virtualWrite(V8, String(gps.location.lng(), 6));  // Longitude to V8
          Blynk.virtualWrite(V9, "GPS Fix Acquired");  // GPS status to V9
          Serial.print("Latitude: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print(" Longitude: ");
          Serial.println(gps.location.lng(), 6);
          lastUpdateTime = millis();
        }
      } else {
        Blynk.virtualWrite(V9, "No GPS Fix");  // Update GPS status
      }
    }

    checkMetalDetection();  // Check for metal detection
  } else {
    // Attempt to reconnect if disconnected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting to reconnect...");
      WiFi.reconnect();
      delay(5000);  // Wait 5 seconds before retrying
    }
    if (!Blynk.connected()) {
      Serial.println("Blynk disconnected, attempting to reconnect...");
      Blynk.connect(10000);  // 10-second timeout
    }
  }
}