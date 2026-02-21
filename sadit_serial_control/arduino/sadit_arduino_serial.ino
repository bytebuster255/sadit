#include <Arduino.h>

/*
 * SADIT Robot Arduino Serial Control (2WD Versiyon)
 * Sadece arka tekerlekleri kontrol eder.
 */

// Motor pin tanımlamaları - Sadece arka tekerlekler
// Not: Pinler önceki kodunuzdaki gibi tanımlanmıştır.
#define LPWM1 5 // Sağ arka motor
#define RPWM1 4 // Sağ arka motor
#define LEN1  3 // Sağ arka motor
#define REN1  2 // Sağ arka motor

#define LPWM2 12 // Sol arka motor
#define RPWM2 11 // Sol arka motor
#define LEN2  10 // Sol arka motor
#define REN2  9 // Sol arka motor

// Motor hızları
int motor1_speed = 0; // Sağ arka
int motor2_speed = 0; // Sol arka

// PWM rampa parametreleri - daha hızlı tepki için
int pwmRampSpeed = 25;
int currentSpeed1 = 0;
int currentSpeed2 = 0;

// PWM sınırları
const int MAX_PWM = 140;
const int MIN_PWM = 0;

// Seri iletişim değişkenleri
String inputString = "";
bool stringComplete = false;
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 100;

// Batarya voltajı (opsiyonel)
#define BATTERY_PIN A2
float batteryVoltage = 0.0;

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  
  // Motor pinlerini ayarla
  setupMotorPins();
  
  // Batarya pinini ayarla
  pinMode(BATTERY_PIN, INPUT);
  
  Serial.println("SADIT Robot Arduino Ready");
  Serial.println("CMD_ACK:READY");
}

void setupMotorPins() {
  // Motor 1 (Sağ arka)
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(LEN1, OUTPUT);
  pinMode(REN1, OUTPUT);
  digitalWrite(LEN1, HIGH);
  digitalWrite(REN1, HIGH);

  // Motor 2 (Sol arka)
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LEN2, OUTPUT);
  pinMode(REN2, OUTPUT);
  digitalWrite(LEN2, HIGH);
  digitalWrite(REN2, HIGH);
}

void loop() {
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  if (millis() - lastStatusTime >= STATUS_INTERVAL) {
    sendStatus();
    lastStatusTime = millis();
  }
  
  readBatteryVoltage();
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processSerialCommand(String command) {
  command.trim();
  
  if (command.startsWith("MOVE:")) {
    // MOVE:motor1,motor2 formatı
    command = command.substring(5);
    
    int comma = command.indexOf(',');
    
    if (comma != -1) {
      motor1_speed = command.substring(0, comma).toInt();
      motor2_speed = command.substring(comma + 1).toInt();
      
      controlMotors();
      
      Serial.println("CMD_ACK:MOVE");
    } else {
      Serial.println("ERROR:Invalid MOVE command format");
    }
  }
  else if (command.startsWith("STOP")) {
    stopAllMotors();
    Serial.println("CMD_ACK:STOP");
  }
  else if (command.startsWith("STATUS")) {
    sendStatus();
  }
  else {
    Serial.print("ERROR:Unknown command: ");
    Serial.println(command);
  }
}

void controlMotors() {
  currentSpeed1 = motor1_speed;
  currentSpeed2 = motor2_speed;
  
  // Motor 1 (Sağ arka)
  if (currentSpeed1 > 0) {
    analogWrite(LPWM1, constrain(currentSpeed1, MIN_PWM, MAX_PWM));
    analogWrite(RPWM1, 0);
  } else if (currentSpeed1 < 0) {
    analogWrite(LPWM1, 0);
    analogWrite(RPWM1, constrain(-currentSpeed1, MIN_PWM, MAX_PWM));
  } else {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
  }
  
  // Motor 2 (Sol arka)
  if (currentSpeed2 > 0) {
    analogWrite(LPWM2, constrain(currentSpeed2, MIN_PWM, MAX_PWM));
    analogWrite(RPWM2, 0);
  } else if (currentSpeed2 < 0) {
    analogWrite(LPWM2, 0);
    analogWrite(RPWM2, constrain(-currentSpeed2, MIN_PWM, MAX_PWM));
  } else {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
  }
}

void stopAllMotors() {
  motor1_speed = 0;
  motor2_speed = 0;
  
  currentSpeed1 = 0;
  currentSpeed2 = 0;
  
  analogWrite(LPWM1, 0);
  analogWrite(RPWM1, 0);
  analogWrite(LPWM2, 0);
  analogWrite(RPWM2, 0);
}

void sendStatus() {
  Serial.print("STATUS: motor1=");
  Serial.print(currentSpeed1);
  Serial.print(",motor2=");
  Serial.print(currentSpeed2);
  Serial.print(",battery=");
  Serial.println(batteryVoltage, 1);
}

void readBatteryVoltage() {
  int rawValue = analogRead(BATTERY_PIN);
  batteryVoltage = (rawValue * 5.0 * 2.4) / 1024.0;
}

