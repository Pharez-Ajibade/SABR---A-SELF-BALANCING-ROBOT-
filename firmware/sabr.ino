// PLEASE UNDERSTAND I WRITE COMMENTS MY OWN WAY,
// BUT NONETHELESS THE BUILD VIDEO FOR THIS IS ON YOUTUBE @LIGIENCE
// Link: https://youtube.com/@LIGIENCE

#include <Wire.h>
#include <math.h> // Required for sqrt, atan2f, fabsf

// -------------------- IMU VARIABLES --------------------

float Gyro_Roll, Gyro_Pitch, Gyro_Yaw;
//       p         q          r

float AccX, AccY, AccZ;
//      x     y     z

float Acc_Roll, Acc_Pitch;

// Sampling rate
// 0.004s approx 200-250Hz depending on execution timing
float sampling_rate = 0.004f;
unsigned long sampling_rate_timer = 4000; // microseconds

unsigned long SR_current_timer;
unsigned long SR_last_timer;


// --------------------- MPU6050 ACCELEROMETER AND GYRO READINGS ---------------------

void Accel_gyro_signals(void) {

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();


  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();


  // Gyro -> deg/s (with calibration offsets)
  Gyro_Roll  = (float)GyroX / 65.5 + 1.40;
  Gyro_Pitch = (float)GyroY / 65.5 - 4.13;
  Gyro_Yaw   = (float)GyroZ / 65.5 + 0.78;


  // Accelerometer -> g (with offsets)
  AccX = (float)AccXLSB / 4096.0f - 0.02f;
  AccY = (float)AccYLSB / 4096.0f - 0.05f;
  AccZ = (float)AccZLSB / 4096.0f - 0.029f;


  // Always normalize (but avoid division by zero)
  float acc_magnitude = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);

  if (acc_magnitude > 0.1f) {
    AccX /= acc_magnitude;
    AccY /= acc_magnitude;
    AccZ /= acc_magnitude;
  }


  // Convert to angles
  Acc_Pitch = atan2f(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;
  Acc_Roll  = atan2f(AccY, AccZ) * RAD_TO_DEG;
}


// -------------------- KALMAN FILTER VARIABLES --------------------

float KalmanAngleRoll = 0.0f;
float KalmanUncertaintyRoll = 2.0f * 2.0f;

float KalmanAnglePitch = 0.0f;
float KalmanUncertaintyPitch = 2.0f * 2.0f;


// Noise tuning (this is where the magic pain lives)
const float Q_angle = 0.003f; // gyro trust
const float R_angle = 0.05f;  // accel trust


// -------------------- KALMAN FILTER --------------------

void Kalman_1d(float &KalmanState,
               float &KalmanUncertainty,
               float KalmanInput,
               float KalmanMeasurement) {

  // Prediction (gyro)
  KalmanState += KalmanInput * sampling_rate;
  KalmanUncertainty += Q_angle * sampling_rate;

  // Kalman gain
  float KalmanGain = KalmanUncertainty /
                     (KalmanUncertainty + R_angle);

  // Update
  KalmanState += KalmanGain *
                (KalmanMeasurement - KalmanState);

  KalmanUncertainty =
      (1.0f - KalmanGain) * KalmanUncertainty;
}


// -------------------- CONTROL MODE --------------------

uint8_t control_mode = 1;  // 0 = angle only, 1 = angle + position


// -------------------- MOTOR POSITION TRACKING --------------------

volatile int32_t motor_left_steps = 0;
volatile int32_t motor_right_steps = 0;

int32_t position_avg = 0;


// -------------------- ANGLE PID --------------------

float angle_setpoint = 3.0f; // yes this moves dynamically later
float angle_error = 0.0f;

float KP_angle = 280.0f;
float Ki_angle = 20.0f;
float Kd_angle = 2.5f;

float Integral_angle = 0.0f;
float Integral_angle_max = 500.0f;

float PID_angle_output = 0.0f;


// -------------------- POSITION PID --------------------

float position_setpoint = 0.0f;
float position_error = 0.0f;

float KP_pos = 1.5f;
float Ki_pos = 0.0f;
float Kd_pos = 0.0f;

float Integral_pos = 0.0f;
float Integral_pos_max = 35.0f;

float position_prev = 0.0f;

float PID_pos_output = 0.0f;

#define PID_POS_MAX 35.0f

float POSITION_DEADZONE = 0.5f;


// -------------------- ANGLE PID FUNCTION --------------------

void angle_PID() {

  angle_error = angle_setpoint - KalmanAnglePitch;

  float P = KP_angle * angle_error;

  // anti-windup (because reality is annoying)
  if (fabsf(angle_error) < 8.0f &&
      fabsf(PID_angle_output) < 500.0f) {

    Integral_angle += angle_error * sampling_rate;
    Integral_angle =
      constrain(Integral_angle,
                -Integral_angle_max,
                 Integral_angle_max);
  } else {
    Integral_angle = 0.0f;
  }

  float I = Ki_angle * Integral_angle;

  // derivative = gyro (because noise is pain)
  float D = -Kd_angle * Gyro_Pitch;

  PID_angle_output = P + I + D;
}


// -------------------- POSITION PID FUNCTION --------------------

void position_PID() {

  position_error =
    (position_setpoint - position_avg) / 1000.0f;

  float P = KP_pos * position_error;

  Integral_pos += position_error * sampling_rate;
  Integral_pos =
    constrain(Integral_pos,
              -Integral_pos_max,
               Integral_pos_max);

  if (fabsf(position_error) > 2.0f) {
    Integral_pos = 0.0f;
  }

  float I = Ki_pos * Integral_pos;

  float pos_rate =
    (position_avg - position_prev) / sampling_rate;

  position_prev = position_avg;

  float D = -Kd_pos * (pos_rate / 1000.0f);

  PID_pos_output = P + I + D;

  PID_pos_output =
    constrain(PID_pos_output,
              -PID_POS_MAX,
               PID_POS_MAX);
}


// -------------------- STEPPER MOTOR CONFIG --------------------

float p;
int steps_per_rev = 1600;


// -------------------- PINS --------------------

const int DIR = 18;
const int STEP = 19;
const int MS1_M1 = 16;
const int MS2_M1 = 4;

const int DIR2 = 17;
const int STEP2 = 5;
const int MS1_M2 = 0;
const int MS2_M2 = 2;


// -------------------- TIMERS --------------------

unsigned long stepper_motor_us_pulse;
unsigned long M_Current_timer;
unsigned long M_Last_timer;


// -------------------- MICROSTEPPING --------------------

uint8_t microStep = 8;


// -------------------- MICROSTEP FUNCTION --------------------

void setMicroStep(uint8_t uStep) {

  switch (uStep) {

    case 2:
      digitalWrite(MS1_M1, LOW);
      digitalWrite(MS2_M1, HIGH);
      digitalWrite(MS1_M2, LOW);
      digitalWrite(MS2_M2, HIGH);
      steps_per_rev = 400;
      break;

    case 4:
      digitalWrite(MS1_M1, HIGH);
      digitalWrite(MS2_M1, LOW);
      digitalWrite(MS1_M2, HIGH);
      digitalWrite(MS2_M2, LOW);
      steps_per_rev = 800;
      break;

    case 8:
      digitalWrite(MS1_M1, LOW);
      digitalWrite(MS2_M1, LOW);
      digitalWrite(MS1_M2, LOW);
      digitalWrite(MS2_M2, LOW);
      steps_per_rev = 1600;
      break;

    case 16:
      digitalWrite(MS1_M1, HIGH);
      digitalWrite(MS2_M1, HIGH);
      digitalWrite(MS1_M2, HIGH);
      digitalWrite(MS2_M2, HIGH);
      steps_per_rev = 3200;
      break;

    default:
      steps_per_rev = 1600;
      uStep = 8;
      break;
  }

  microStep = uStep;
}


// -------------------- TASK HANDLES --------------------

TaskHandle_t Task1;
TaskHandle_t Task2;


// -------------------- SETUP --------------------

void setup() {

  Serial.begin(115200);

  Wire.setClock(400000);
  Wire.begin();

  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();


  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1_M1, OUTPUT);
  pinMode(MS2_M1, OUTPUT);

  pinMode(STEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(MS1_M2, OUTPUT);
  pinMode(MS2_M2, OUTPUT);


  setMicroStep(8);
  control_mode = 1;

  delay(250);

  M_Last_timer = micros();
  SR_last_timer = micros();


  // Task 1 (IMU + control)
  xTaskCreatePinnedToCore(
    Task1code,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    0
  );

  delay(500);


  // Task 2 (motor stepping)
  xTaskCreatePinnedToCore(
    Task2code,
    "Task2",
    10000,
    NULL,
    1,
    &Task2,
    1
  );

  delay(500);
}


// -------------------- TASK 1 --------------------

void Task1code(void *pvParameters) {

  Serial.println("Task1 running on core " +
                 String(xPortGetCoreID()));

  for (;;) {

    SR_current_timer = micros();

    if (SR_current_timer - SR_last_timer >=
        sampling_rate_timer) {

      Accel_gyro_signals();

      Kalman_1d(KalmanAnglePitch,
                KalmanUncertaintyPitch,
                Gyro_Pitch,
                Acc_Pitch);

      position_avg =
        (motor_left_steps +
         motor_right_steps) / 2;


      if (control_mode == 0) {

        angle_setpoint = 3.0f;
        angle_PID();

      } else {

        position_PID();
        angle_setpoint = PID_pos_output + 2.0f;
        angle_PID();
      }


      p = fabsf(PID_angle_output);
      p = constrain(p, 0.001f, 40000.0f);


      setMicroStep(8);


      if (PID_angle_output > 0) {

        digitalWrite(DIR, LOW);
        digitalWrite(DIR2, HIGH);

        stepper_motor_us_pulse =
          (unsigned long)(1000000.0f / p);

      } else {

        digitalWrite(DIR, HIGH);
        digitalWrite(DIR2, LOW);

        stepper_motor_us_pulse =
          (unsigned long)(1000000.0f / p);
      }


      if (fabsf(angle_error) < 0.3f) {
        stepper_motor_us_pulse = 100000UL;
      }

      if (fabsf(angle_error) > 45.0f) {
        stepper_motor_us_pulse = 100000UL;
      }


      SR_last_timer = SR_current_timer;
    }
  }
}


// -------------------- TASK 2 --------------------

void Task2code(void *pvParameters) {

  Serial.println("Task2 running on core " +
                 String(xPortGetCoreID()));

  for (;;) {

    M_Current_timer = micros();

    if (M_Current_timer - M_Last_timer >=
        stepper_motor_us_pulse) {

      digitalWrite(STEP, HIGH);
      digitalWrite(STEP2, HIGH);

      delayMicroseconds(1);

      digitalWrite(STEP, LOW);
      digitalWrite(STEP2, LOW);


      if (digitalRead(DIR) == LOW) {
        motor_left_steps++;
      } else {
        motor_left_steps--;
      }

      if (digitalRead(DIR2) == HIGH) {
        motor_right_steps++;
      } else {
        motor_right_steps--;
      }

      M_Last_timer = M_Current_timer;
    }
  }
}


// -------------------- LOOP --------------------

void loop() {
  // intentionally empty (RTOS takes over)
}