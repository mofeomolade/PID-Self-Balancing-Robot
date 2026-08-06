#include <MPU6050_Driver.h>

#define IN1_PIN 20
#define IN2_PIN 8
#define IN3_PIN 9
#define IN4_PIN 10

// put function declarations here:


void setup() {
  // put your setup code here, to run once:
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
}

void loop() {
  digitalWrite (IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

// put function definitions here:
void drive_motors();