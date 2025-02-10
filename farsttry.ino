#include <Wire.h>
#include <VL53L0X.h>

// Motor pins
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
#define ENA 26
#define ENB 25

// IR sensor pins
#define IR_LEFT 34
#define IR_RIGHT 35

// Encoder pins
#define ENCA_A 4
#define ENCA_B 18
#define ENCB_A 19
#define ENCB_B 23

// LiDAR
VL53L0X lidar;

// Encoder variables
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;

// Function prototypes for encoder interrupts
void IRAM_ATTR encoderAInterrupt();
void IRAM_ATTR encoderBInterrupt();
void IRAM_ATTR encoderCInterrupt();
void IRAM_ATTR encoderDInterrupt();

void setup() {
  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR sensors setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Encoder setup
  pinMode(ENCA_A, INPUT);
  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_A, INPUT);
  pinMode(ENCB_B, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), encoderBInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderCInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), encoderDInterrupt, CHANGE);

  // LiDAR setup
  Wire.begin();
  lidar.init();
  lidar.setTimeout(500);

  // Serial setup
  Serial.begin(115200);
}

void loop() {
  // Read sensor data
  bool irLeft = digitalRead(IR_LEFT);   // Left IR sensor (1 = obstacle detected, 0 = clear)
  bool irRight = digitalRead(IR_RIGHT); // Right IR sensor (1 = obstacle detected, 0 = clear)
  int distance = lidar.readRangeSingleMillimeters() / 10; // LiDAR distance in cm

  // Print sensor data for debugging
  Serial.print("Left IR: ");
  Serial.print(irLeft);
  Serial.print(" | Right IR: ");
  Serial.print(irRight);
  Serial.print(" | LiDAR Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Decision-making logic
  if (distance < 30) { // If obstacle is within 30 cm
    stopMotors(); // Stop the robot
    delay(500); // Wait for 500 ms

    // Turn left or right based on IR sensor readings
    if (irLeft == 1 && irRight == 0) { // Obstacle on the left
      turnRight(); // Turn right
    } else if (irLeft == 0 && irRight == 1) { // Obstacle on the right
      turnLeft(); // Turn left
    } else { // Obstacle on both sides or unclear
      turnRight(); // Default to turning right
    }
    delay(500); // Turn for 500 ms
  } else { // If no obstacle is detected
    moveForward(); // Move forward
  }
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Encoder interrupt functions
void IRAM_ATTR encoderAInterrupt() {
  if (digitalRead(ENCA_B)) {
    pulseCountA++;
  } else {
    pulseCountA--;
  }
}

void IRAM_ATTR encoderBInterrupt() {
  if (digitalRead(ENCA_A)) {
    pulseCountA--;
  } else {
    pulseCountA++;
  }
}

void IRAM_ATTR encoderCInterrupt() {
  if (digitalRead(ENCB_B)) {
    pulseCountB++;
  } else {
    pulseCountB--;
  }
}

void IRAM_ATTR encoderDInterrupt() {
  if (digitalRead(ENCB_A)) {
    pulseCountB--;
  } else {
    pulseCountB++;
  }
}