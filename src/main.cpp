#include <Arduino.h>

#define PULSE_PER_METER 2343.0
#define ENC_R1 32
#define ENC_R2 33
#define MOTOR_R1_PIN 25
#define MOTOR_R2_PIN 26
#define ENC_L1 18
#define ENC_L2 19
#define MOTOR_L1_PIN 16
#define MOTOR_L2_PIN 17
#define MOTOR_L1 0
#define MOTOR_L2 1
#define MOTOR_R1 2
#define MOTOR_R2 3

volatile int enc_val_l = 0;
volatile int enc_val_r = 0;
const double Kp_l = 19.0; 
const double Ki_l = 2.0;
const double Kd_l = 2.5;

const double Kp_r = 20.0; 
const double Ki_r = 2.5;
const double Kd_r = 2.5;
// m 20 20
double Ierror_l = 0.0;
double Ierror_r = 0.0;
double prev_error_l = 0.0;
double prev_error_r = 0.0;
int prev_enc_l = enc_val_l;
int prev_enc_r = enc_val_r;
unsigned long prev_time = millis();


int pulse_per_cycle_left = 0;
int pulse_per_cycle_right = 0;
bool run_closed_loop = false;

void IRAM_ATTR ENC_L() {
  if (digitalRead(ENC_L1) == digitalRead(ENC_L2)) {
    enc_val_l++;
  } else {
    enc_val_l--;
  }
}

void IRAM_ATTR ENC_R() {
  if (digitalRead(ENC_R1) == digitalRead(ENC_R2)) {
    enc_val_r++;
  } else {
    enc_val_r--;
  }
}

void setMotorSpeed(int motor1, int motor2, int speed) {
  if (speed >= 0) {
    ledcWrite(motor1, speed);
    ledcWrite(motor2, 0);
  } else {
    ledcWrite(motor1, 0);
    ledcWrite(motor2, -speed);
  }
}

void openLoopControl(int speed_left, int speed_right) {
  setMotorSpeed(MOTOR_L1, MOTOR_L2, speed_left);
  setMotorSpeed(MOTOR_R1, MOTOR_R2, speed_right);
}

void readEncoders() {
  double left_distance = (enc_val_l / PULSE_PER_METER) * 1000.0;
  double right_distance = (enc_val_r / PULSE_PER_METER) * 1000.0;
  Serial.print("Left encoder (mm): ");
  Serial.print(left_distance);
  Serial.print(" Right encoder (mm): ");
  Serial.println(right_distance);
}

void resetEncoders() {
  enc_val_l = 0;
  enc_val_r = 0;
  Serial.println("Encoders reset");
}

void setup() {
  Serial.begin(9600);
  pinMode(ENC_L1, INPUT_PULLUP);
  pinMode(ENC_L2, INPUT_PULLUP);
  pinMode(ENC_R1, INPUT_PULLUP);
  pinMode(ENC_R2, INPUT_PULLUP);
  
  ledcAttachPin(MOTOR_L1_PIN, MOTOR_L1);
  ledcAttachPin(MOTOR_L2_PIN, MOTOR_L2);
  ledcSetup(MOTOR_L1, 20000, 8);
  ledcSetup(MOTOR_L2, 20000, 8);
  ledcAttachPin(MOTOR_R1_PIN, MOTOR_R1);
  ledcAttachPin(MOTOR_R2_PIN, MOTOR_R2);
  ledcSetup(MOTOR_R1, 20000, 8);
  ledcSetup(MOTOR_R2, 20000, 8);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L1), ENC_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R1), ENC_R, RISING);
}

void closedLoopSpeedControl() {
  unsigned long current_time = millis();
  double dt = (current_time - prev_time);  // Time difference in milliseconds
  if (dt >= 25) {  // Update every 25ms
    double error_l = pulse_per_cycle_left - (enc_val_l - prev_enc_l);
    Ierror_l += error_l;
    double output_l = ((Kp_l * error_l) + (Kd_l * (error_l - prev_error_l)) + (Ki_l*Ierror_l));

    double error_r = pulse_per_cycle_right - (enc_val_r - prev_enc_r);
    Ierror_r += error_r;
    double output_r = ((Kp_r * error_r) + (Kd_r * (error_r - prev_error_r)) + (Ki_r*Ierror_r));

    output_l = constrain(output_l, -255, 255);
    output_r = constrain(output_r, -255, 255);
  
    setMotorSpeed(MOTOR_L1, MOTOR_L2, output_l);
    setMotorSpeed(MOTOR_R1, MOTOR_R2, output_r);

    Serial.print(error_l);
    Serial.print("\t");
    Serial.println(error_r);

    prev_error_r = error_r;
    prev_error_l = error_l;
    prev_enc_r = enc_val_r;
    prev_enc_l = enc_val_l;
    prev_time = current_time;
  }
}

void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.startsWith("o ")) {
    int speed_left = command.substring(2, command.indexOf(' ', 2)).toInt();
    int speed_right = command.substring(command.lastIndexOf(' ') + 1).toInt();
    run_closed_loop = false;
    openLoopControl(speed_left, speed_right);
  } else if (command == "e") {
    readEncoders();
  } else if (command == "r") {
    resetEncoders();
  } else if (command.startsWith("m ")) {
    pulse_per_cycle_left = command.substring(2, command.indexOf(' ', 2)).toInt();
    pulse_per_cycle_right = command.substring(command.lastIndexOf(' ') + 1).toInt();
    Ierror_l = 0.0;
    Ierror_r = 0.0;
    prev_error_l = 0.0;
    prev_error_r = 0.0;
    prev_enc_l = enc_val_l;
    prev_enc_r = enc_val_r;
    prev_time = millis();

    run_closed_loop = true;
  } else if (command == "s") {
    run_closed_loop = false;
    setMotorSpeed(MOTOR_L1, MOTOR_L2, 0);
    setMotorSpeed(MOTOR_R1, MOTOR_R2, 0);
  } else {
    Serial.println("Invalid command");
  }
}

void loop() {
  if (Serial.available()) {
    processSerialCommand();
  }
  
  if (run_closed_loop) {
    closedLoopSpeedControl();
  }
}

