#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include "MPU6050.h"

// ======================================================
//                    WiFi Credentials
// ======================================================
const char* ssid = "Subject of Ymir";
const char* password = "1122334455";

WiFiServer server(1234);
WiFiClient client;

// ======================================================
//                      Objects
// ======================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
MPU6050 mpu;

// ======================================================
//                     Motor Pins
// ======================================================
#define IN1 14
#define IN2 12
#define IN3 13
#define IN4 15
#define ENA 32
#define ENB 5

#define ENA_CHANNEL 0
#define ENB_CHANNEL 1

// ======================================================
//                    Sensor Pins
// ======================================================
//#define VIB_PIN      34
#define BUZZER       25
#define BTN_CANCEL    4

// ======================================================
//                 UART Configuration
// ======================================================
HardwareSerial GPS_Serial(2);   // RX=16, TX=17
HardwareSerial GSM_Serial(1);   // RX=26, TX=27

// ======================================================
//                   Threshold Values
// ======================================================
const float ACC_THRESHOLD  = 2.5;
const float TILT_THRESHOLD = 45.0;

// ======================================================
//                    Global Variables
// ======================================================
bool accidentDetected = false;
bool alertSent = false;
unsigned long accidentTime = 0;

// ======================================================
//                        Setup
// ======================================================
void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin(21, 22);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // MPU6050
  mpu.initialize();

  // Sensor Pins
  //pinMode(VIB_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_CANCEL, INPUT_PULLUP);

  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM Setup
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  // GPS
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);

  // GSM
  GSM_Serial.begin(9600, SERIAL_8N1, 26, 27);

  // WiFi
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  server.begin();

  Serial.println("\nWiFi Connected");
  Serial.println(WiFi.localIP());

  lcd.clear();
  lcd.print("System Ready");
  delay(2000);
}

// ======================================================
//                         Loop
// ======================================================
void loop() {
  readGPS();
  handleWiFiControl();

  if (!accidentDetected) {
    checkAccident();
  } else {
    handleAccident();
  }
}

// ======================================================
//                  WiFi Robot Control
// ======================================================
void handleWiFiControl() {
  if (!client || !client.connected()) {
    client = server.available();
  }

  if (client && client.available()) {
    char cmd = client.read();
    Serial.println(cmd);

    switch (cmd) {
      case 'F': forward(); break;
      case 'B': backward(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      case 'S': stopMotors(); break;
    }
  }
}

// ======================================================
//                      GPS Reading
// ======================================================
void readGPS() {
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }
}

// ======================================================
//                   Accident Detection
// ======================================================
void checkAccident() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float acc = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  float tilt = atan2(ax_g, az_g) * 180.0 / PI;

  //int vibration = digitalRead(VIB_PIN);

  if (acc > ACC_THRESHOLD || abs(tilt) > TILT_THRESHOLD) {

    accidentDetected = true;
    alertSent = false;
    accidentTime = millis();
 
    stopMotors();

    digitalWrite(BUZZER, HIGH);

    lcd.clear();
    lcd.print("Accident!");
    lcd.setCursor(0, 1);
    lcd.print("Sending Alert");
  }
}

// ======================================================
//                  Accident Handling
// ======================================================
void handleAccident() {

  // Cancel Emergency
  if (digitalRead(BTN_CANCEL) == LOW) {
    accidentDetected = false;
    alertSent = false;

    digitalWrite(BUZZER, LOW);

    lcd.clear();
    lcd.print("Alert Cancelled");
    delay(2000);

    lcd.clear();
    lcd.print("System Ready");
    return;
  }

  // Wait 10 seconds
  if (millis() - accidentTime < 10000) {
    return;
  }

  // Send Alert Once
  if (!alertSent) {
    sendSMS();
    makeCall();

    digitalWrite(BUZZER, LOW);

    lcd.clear();
    lcd.print("Alert Sent");

    alertSent = true;
  }
}

// ======================================================
//                    Motor Functions
// ======================================================
void forward() {
  if (accidentDetected) return;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setSpeed(200, 200);
}

void backward() {
  if (accidentDetected) return;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setSpeed(200, 200);
}

void left() {
  if (accidentDetected) return;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setSpeed(200, 200);
}

void right() {
  if (accidentDetected) return;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setSpeed(200, 200);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  setSpeed(0, 0);
}

void setSpeed(int leftSpeed, int rightSpeed) {
  ledcWrite(ENA_CHANNEL, leftSpeed);
  ledcWrite(ENB_CHANNEL, rightSpeed);
}

// ======================================================
//                      Send SMS
// ======================================================
void sendSMS() {
  String message = "Accident Detected! ";

  if (gps.location.isValid()) {
    message += "Location: https://maps.google.com/?q=";
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);
  } else {
    message += "Location Not Available";
  }

  GSM_Serial.println("AT+CMGF=1");
  delay(1000);

  GSM_Serial.println("AT+CMGS=\"+91XXXXXXXXXX\"");
  delay(1000);

  GSM_Serial.print(message);
  delay(500);

  GSM_Serial.write(26);
  delay(3000);
}

// ======================================================
//                      Make Call
// ======================================================
void makeCall() {
  GSM_Serial.println("ATD+91XXXXXXXXXX;");
  delay(20000);
  GSM_Serial.println("ATH");
}
\
