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
const float Kp = 300.0; // Proportional gain for closed-loop control

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

void setup() {
  Serial.begin(115200);
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
  while (true) {
    if (Serial.available()) {
      setMotorSpeed(MOTOR_L1, MOTOR_L2, 0);
      setMotorSpeed(MOTOR_R1, MOTOR_R2, 0);
      break;  // Exit if new command received
    }
  }

}

void readEncoders() {
  float left_distance = (enc_val_l / PULSE_PER_METER) * 1000.0;
  float right_distance = (enc_val_r / PULSE_PER_METER) * 1000.0;
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

void closedLoopSpeedControl(int base_speed) {
  int prev_enc_left = enc_val_l;
  int prev_enc_right = enc_val_r;
  unsigned long prev_time = millis();

  while (true) {
    if (Serial.available()) {
      setMotorSpeed(MOTOR_L1, MOTOR_L2, 0);
      setMotorSpeed(MOTOR_R1, MOTOR_R2, 0);
      break;  // Exit if new command received
    }

    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;  // Time difference in seconds

    if (dt >= 0.1) {  // Update every 100ms
      float actual_speed_left = ((enc_val_l - prev_enc_left) / PULSE_PER_METER) / dt;
      float actual_speed_right = ((enc_val_r - prev_enc_right) / PULSE_PER_METER) / dt;

        // Calculate error and adjust motor speeds
      float error = actual_speed_right - actual_speed_left;
      int pwm_left = base_speed + int(Kp * error);
      int pwm_right = base_speed - int(Kp * error);

      // Ensure motor speeds are within valid range
      pwm_left = constrain(pwm_left, 0, 255);
      pwm_right = constrain(pwm_right, 0, 255);

      setMotorSpeed(MOTOR_L1, MOTOR_L2, pwm_left);
      setMotorSpeed(MOTOR_R1, MOTOR_R2, pwm_right);

      prev_enc_left = enc_val_l;
      prev_enc_right = enc_val_r;
      prev_time = current_time;

      Serial.print("Left speed(m/s): ");
      Serial.print(actual_speed_left);
      Serial.print(" Right speed(m/s): ");
      Serial.println(actual_speed_right);
    }

    delay(10);
  }
}

void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.startsWith("o ")) {
    int speed_left = command.substring(2, command.indexOf(' ', 2)).toInt();
    int speed_right = command.substring(command.lastIndexOf(' ') + 1).toInt();
    openLoopControl(speed_left, speed_right);
  } else if (command == "e") {
    readEncoders();
  } else if (command == "r") {
    resetEncoders();
  } else if (command.startsWith("m ")) {
    int base_speed = command.substring(command.lastIndexOf(' ') + 1).toInt();
    closedLoopSpeedControl(base_speed);
  } else {
    Serial.println("Invalid command");
  }
}

void loop() {
  if (Serial.available()) {
    processSerialCommand();
  }
  delay(10);
}