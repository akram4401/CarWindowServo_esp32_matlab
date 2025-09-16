#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==== OLED Setup ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==== Pins ====
const int potPin = 34;     // Potentiometer feedback
const int motorLeft = 16;  // Motor driver left (HIGH = left)
const int motorRight = 17; // Motor driver right (HIGH = right)

// ==== PID Settings ====
float Kp = 2.0;   // Proportional
float Ki = 0.1;   // Integral
float Kd = 0.05;  // Derivative

// ==== Control Settings ====
int setpoint = 90;          // Desired angle (deg)
int deadband = 2;           // tolerance ±deg (can be changed via Serial)
int maxPWM = 255;           // output limit (used for control math)

// ==== Variables ====
int position = 0;
float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(motorLeft, OUTPUT);
  pinMode(motorRight, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("DC Motor Servo Ready");
  display.display();
  delay(1000);

  Serial.println("Commands:");
  Serial.println("SP <angle>   -> set target angle (0-270)");
  Serial.println("KP <value>   -> set Kp");
  Serial.println("KI <value>   -> set Ki");
  Serial.println("KD <value>   -> set Kd");
  Serial.println("DB <value>   -> set deadband tolerance (deg)");
}

void loop() {
  // === Handle Serial Commands ===
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("SP")) {
      setpoint = input.substring(2).toInt();
      if (setpoint < 0) setpoint = 0;
      if (setpoint > 270) setpoint = 270;
      Serial.print("Setpoint updated: "); Serial.println(setpoint);
    } 
    else if (input.startsWith("KP")) {
      Kp = input.substring(2).toFloat();
      Serial.print("Kp updated: "); Serial.println(Kp);
    } 
    else if (input.startsWith("KI")) {
      Ki = input.substring(2).toFloat();
      Serial.print("Ki updated: "); Serial.println(Ki);
    } 
    else if (input.startsWith("KD")) {
      Kd = input.substring(2).toFloat();
      Serial.print("Kd updated: "); Serial.println(Kd);
    } 
    else if (input.startsWith("DB")) {
      deadband = input.substring(2).toInt();
      if (deadband < 0) deadband = 0;
      Serial.print("Deadband updated: ±"); Serial.println(deadband);
    }
    else {
      Serial.println("Unknown command");
    }
  }

  // === Read feedback (0-4095 -> 0-270 deg) ===
  position = map(analogRead(potPin), 0, 4095, 0, 270);

  // === PID Calculation ===
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  float error = setpoint - position;

  if (abs(error) <= deadband) {
    // Inside tolerance → stop motor
    digitalWrite(motorLeft, LOW);
    digitalWrite(motorRight, LOW);
    integral = 0;  // reset integral to prevent windup
  } else {
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Limit output
    if (output > maxPWM) output = maxPWM;
    if (output < -maxPWM) output = -maxPWM;

    // Motor control (bang-bang with direction only)
    if (output > 0) {
      digitalWrite(motorLeft, HIGH);
      digitalWrite(motorRight, LOW);
    } else {
      digitalWrite(motorLeft, LOW);
      digitalWrite(motorRight, HIGH);
    }

    prevError = error;
  }

  // === OLED Display ===
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("SP: "); display.println(setpoint);
  display.print("POS: "); display.println(position);
  display.print("ERR: "); display.println(error);
  display.print("DB: ±"); display.println(deadband);
  display.print("Kp: "); display.print(Kp);
  display.print(" Ki: "); display.print(Ki);
  display.print(" Kd: "); display.println(Kd);
  display.display();

  // === Telemetry Output (for MATLAB) ===
  Serial.print("T,"); Serial.print(millis());
  Serial.print(",SP,"); Serial.print(setpoint);
  Serial.print(",POS,"); Serial.print(position);
  Serial.print(",ERR,"); Serial.print(error);
  Serial.print(",OUT,"); Serial.println(prevError);  

  delay(20); // control loop speed ~50 Hz
}
