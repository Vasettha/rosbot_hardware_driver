#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>

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
// m 20 20
// m 15 15

volatile int enc_val_l = 0;
volatile int enc_val_r = 0;
const double Kp_l = 25.0; 
const double Ki_l = 0.72;
const double Kd_l = 2.15;

const double Kp_r = 25.0; 
const double Ki_r = 0.72;
const double Kd_r = 2.15;

double Ierror_l = 0.0;
double Ierror_r = 0.0;
double prev_error_l = 0.0;
double prev_error_r = 0.0;
const double MAX_IERROR = 255.0; // Define maximum integral error
int prev_enc_l = enc_val_l;
int prev_enc_r = enc_val_r;
unsigned long prev_time = 0;

int pulse_per_cycle_left = 0;
int pulse_per_cycle_right = 0;
bool run_closed_loop = false;

QueueHandle_t commandQueue;
TaskHandle_t closedLoopTaskHandle = NULL;
TimerHandle_t watchdogTimer;

// Function prototypes
void IRAM_ATTR ENC_L();
void IRAM_ATTR ENC_R();
void setMotorSpeed(int motor1, int motor2, int speed);
void openLoopControl(int speed_left, int speed_right);
void readEncoders();
void resetEncoders();
void stopMotors();
void watchdogCallback(TimerHandle_t xTimer);
void serialCommandTask(void *pvParameters);
void startClosedLoopTask();
void updateClosedLoopParameters(int left, int right);
void processSerialCommand(String command);
void closedLoopSpeedControl();
void closedLoopControlTask(void *pvParameters);
void commandProcessingTask(void *pvParameters);


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

void stopMotors() {
  run_closed_loop = false;
  setMotorSpeed(MOTOR_L1, MOTOR_L2, 0);
  setMotorSpeed(MOTOR_R1, MOTOR_R2, 0);
  Serial.println("Motors stopped");
}

void watchdogCallback(TimerHandle_t xTimer) {
  stopMotors();
  if (closedLoopTaskHandle != NULL) {
    vTaskDelete(closedLoopTaskHandle);
    closedLoopTaskHandle = NULL;
    Serial.println("Closed-loop control stopped due to inactivity");
  }
}

void serialCommandTask(void *pvParameters) {
  String command;
  for (;;) {
    if (Serial.available()) {
      command = Serial.readStringUntil('\n');
      command.trim();
      xQueueSend(commandQueue, &command, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void startClosedLoopTask() {
  if (closedLoopTaskHandle == NULL) {
    Ierror_l = 0.0;
    Ierror_r = 0.0;
    prev_error_l = 0.0;
    prev_error_r = 0.0;
    prev_enc_l = enc_val_l;
    prev_enc_r = enc_val_r;
    prev_time = millis();
    xTaskCreatePinnedToCore(closedLoopControlTask, "ClosedLoopControl", 2048, NULL, 2, &closedLoopTaskHandle, 1);
    Serial.println("Closed-loop control task started");
  }
  run_closed_loop = true;
  xTimerReset(watchdogTimer, 0);
}

void updateClosedLoopParameters(int left, int right) {
  pulse_per_cycle_left = left;
  pulse_per_cycle_right = right;
  xTimerReset(watchdogTimer, 0);
  Serial.println("Closed-loop parameters updated");
}

void processSerialCommand(String command) {
  if (command.startsWith("o ")) {
    int speed_left = command.substring(2, command.indexOf(' ', 2)).toInt();
    int speed_right = command.substring(command.lastIndexOf(' ') + 1).toInt();
    stopMotors();
    openLoopControl(speed_left, speed_right);
  } else if (command == "e") {
    readEncoders();
  } else if (command == "r") {
    resetEncoders();
  } else if (command.startsWith("m ")) {
    int left = command.substring(2, command.indexOf(' ', 2)).toInt();
    int right = command.substring(command.lastIndexOf(' ') + 1).toInt();
    if (closedLoopTaskHandle == NULL) {
      startClosedLoopTask();
    }
    updateClosedLoopParameters(left, right);
  } else if (command == "s") {
    stopMotors();
  } else {
    Serial.println("Invalid command");
  }
}

unsigned long prev_time_micros = 0;

void closedLoopSpeedControl() {
  unsigned long current_time_micros = micros();
  double dt = (current_time_micros - prev_time_micros) / 1000.0;  // Time difference in milliseconds
  if (dt >= 25) {  // Update every 25ms
    double error_l = pulse_per_cycle_left - (enc_val_l - prev_enc_l);
    Ierror_l += error_l;
    Ierror_l = constrain(Ierror_l, -MAX_IERROR, MAX_IERROR);  // Constrain integral term
    double output_l = ((Kp_l * error_l) + (Kd_l * (error_l - prev_error_l)) + (Ki_l * Ierror_l));

    double error_r = pulse_per_cycle_right - (enc_val_r - prev_enc_r);
    Ierror_r += error_r;
    Ierror_r = constrain(Ierror_r, -MAX_IERROR, MAX_IERROR);  // Constrain integral term
    double output_r = ((Kp_r * error_r) + (Kd_r * (error_r - prev_error_r)) + (Ki_r * Ierror_r));

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
    prev_time_micros = current_time_micros;
  }
}

void closedLoopControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25);  // 40Hz
    
    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (run_closed_loop) {
            closedLoopSpeedControl();
        }
    }
}

void commandProcessingTask(void *pvParameters) {
  String command;
  for (;;) {
    if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdTRUE) {
      processSerialCommand(command);
    }
  }
}

// Add new task for encoder publishing
void encoderPublishTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25);  // 40Hz
    
    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (Serial.availableForWrite() > 40) {  // Ensure enough buffer space
            double left_distance = (enc_val_l / PULSE_PER_METER) * 1000.0;
            double right_distance = (enc_val_r / PULSE_PER_METER) * 1000.0;
            Serial.print("Left encoder (mm): ");
            Serial.print(left_distance, 2);
            Serial.print(" Right encoder (mm): ");
            Serial.println(right_distance, 2);
        }
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

  commandQueue = xQueueCreate(10, sizeof(String));

  watchdogTimer = xTimerCreate("WatchdogTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, watchdogCallback);

   // Increase serial buffer sizes
  Serial.setRxBufferSize(256);
  Serial.setTxBufferSize(256);
  
  // Create tasks with appropriate priorities
  xTaskCreatePinnedToCore(serialCommandTask, "SerialCommand", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(commandProcessingTask, "CommandProcessing", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(encoderPublishTask, "EncoderPublish", 2048, NULL, 3, NULL, 1);
  

}

void loop() {
  // Empty, as tasks are now handling the main functionality
}