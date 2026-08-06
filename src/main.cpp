#include <MPU6050_Driver.h>

#define IN1_PIN 5
#define IN2_PIN 4
#define IN3_PIN 3
#define IN4_PIN 2
#define ENA_PIN 10
#define ENB_PIN 9

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
void drive_motors()