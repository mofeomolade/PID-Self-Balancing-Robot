#include <MPU6050_Driver.h>

#define SDA 6
#define SCL 7

#define IN1_PIN 20
#define IN2_PIN 8
#define IN3_PIN 9
#define IN4_PIN 10
//IN2 & IN3 = Forwards (if pitch is negative)
//IN1 & IN4 = Backwards (if pitch is positive)

#define ENA_PIN 3
#define ENB_PIN 2

#define TARGET_ANGLE 0
#define MAXIMUM_ANGLE 45 //Shutdown angle to prevent

#define KP 0
#define KI 0
#define KD 0

//Structs to utilize custom MPU_6050 library
struct IMU imu;
struct Offset offset;
struct Attitude attitude;

//All relevant variables for PID control can be called by function in a single struct
struct PID{
  float IMU_measurement;
  float current_error;
  float last_error = 0;
  float output;
};

struct PID pid; //Initialize PID struct to access control data globally

unsigned long current_time;
unsigned long last_time;
float dt;

// put function declarations here:
void drive_forward (void); //Use to adjust when pitch is positive
void drive_backwards (void); //Use to adjust when pitch is negatibe
void motor_shutdown (void);
void PID_control (PID &controller, IMU &data, float dt);

void setup() {
  // put your setup code here, to run once:
  
  //Set GPIO pins to output to drive motors
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  Wire.begin(SDA, SCL); //Initialize I2C as controller
  Serial.begin(115200);

  write_byte(MAIN_REGISTER, WAKE_REGISTER, WAKE_CMD); //Deactivate sleep mode by writing to PWR_MGMT_1 register

  offset_test(imu, offset); //Run initial baseline offset test and store

  //Seeding for accurate readings in main loop()
  if(read_IMU(imu) == 0){
    correct_IMU(imu, offset);

    //Initial orientation calculation so starting position reflects actual orientation
    attitude.pitch = atan2(-imu.accel_x, sqrt( imu.accel_y * imu.accel_y+ imu.accel_z * imu.accel_z)) * RAD_TO_DEGREES;
    attitude.roll = atan2(imu.accel_y, imu.accel_z) * RAD_TO_DEGREES;
    attitude.yaw = 0.0; //Yaw set to zero with respect to initial position

    last_time = micros();
  }
}

void loop() {
  /*
  current_time = micros();
  dt = (current_time - last_time) / 1000000.0; //Measure dt for PID calculus and IMU data filtering
  last_time = current_time;

  //Only perform calculations upon successful read
  if(read_IMU(imu) == 0){
    correct_IMU(imu, offset);
    filter_IMU(imu, attitude, dt);

    PID_control(pid, imu, dt);
  }
  */
  analogWrite(ENA_PIN, 255); // Full speed enable
  analogWrite(ENB_PIN, 255);

  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

}

// put function definitions here:
void drive_forward (void){
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void drive_backwards (void){
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN4_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
}

void motor_shutdown (void){
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
}

void PID_control (PID &controller, IMU &data, float dt) {

}