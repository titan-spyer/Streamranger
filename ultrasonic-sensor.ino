#include <ESP32Servo.h>
#define TRIG_PIN 4
#define ECHO_PIN 5
#define PAN_PIN 14
#define TILT_PIN 12

// Define motor pins
struct MOTOR_PINS
{
  int pinEn;  
  int pinIN1;
  int pinIN2;    
};

std::vector<MOTOR_PINS> motorPins = 
{
  {2, 1, 13}, //RIGHT_MOTOR Pins (EnA, IN1, IN2)
  {2, 3, 15}, //LEFT_MOTOR  Pins (EnB, IN3, IN4)
};

// Initialize servo and ultrasonic sensor
Servo myservo;
long duration, distance;

void setup() {
  // Set up serial communication at 9600 baud
  Serial.begin(9600);

  // Initialize servo motor
  myservo.attach(PAN_PIN);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize motor pins
  for (auto motor : motorPins) {
    pinMode(motor.pinEn, OUTPUT);
    pinMode(motor.pinIN1, OUTPUT);
    pinMode(motor.pinIN2, OUTPUT);
  }
}

void loop() {
  // Sweep the servo motor in the range of 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle += 10) {
    myservo.write(angle); // Set servo motor angle
    delay(500); // Wait for servo to reach position

    // Measure distance using ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;

    // If an obstacle is detected, move the robot in the opposite direction
    if (distance < 30) { // Change this distance value as needed
      // Set motor direction based on servo angle
      if (angle <= 90) { // Obstacle to the left
        // Turn right
        digitalWrite(motorPins[0].pinIN1, LOW);
        digitalWrite(motorPins[0].pinIN2, HIGH);
        digitalWrite(motorPins[1].pinIN1, LOW);
        digitalWrite(motorPins[1].pinIN2, HIGH);
      } else { // Obstacle to the right
        // Turn left
        digitalWrite(motorPins[0].pinIN1, HIGH);
        digitalWrite(motorPins[0].pinIN2, LOW);
        digitalWrite(motorPins[1].pinIN1, HIGH);
        digitalWrite(motorPins[1].pinIN2, LOW);
      }
      // Move backward
      analogWrite(motorPins[0].pinEn, 255);
      analogWrite(motorPins[1].pinEn, 255);
      delay(1000);
    } else { // No obstacle detected
      // Move motors forward
      digitalWrite(motorPins[0].pinIN1, HIGH);
      digitalWrite(motorPins[0].pinIN2, LOW);
      digitalWrite(motorPins[1].pinIN1, HIGH);
      digitalWrite(motorPins[1].pinIN2, LOW);
    // Set motors speed
      analogWrite(motorPins[0].pinEn, 255);
      analogWrite(motorPins[1].pinEn, 255);
      }
delay(50); // delay for servo and motor movements
}
}

void Servo_loop() {
int   getDistance
// Rotate servo from left to right
for (int getDistance = 0; getDistance <= 180; getDistance++) {
int distance = getDistance; // Get distance using HCSR04 sensor at current servo position
checkObstacle(distance); // Check for obstacle and move motors accordingly
delay(10); // Small delay for smooth movement
}

// Rotate servo from right to left
for (int getDistance = 180; getDistance >= 0; getDistance--) {
int distance = getDistance; // Get distance using HCSR04 sensor at current servo position
checkObstacle(distance); // Check for obstacle and move motors accordingly
delay(10); // Small delay for smooth movement
}
}      
