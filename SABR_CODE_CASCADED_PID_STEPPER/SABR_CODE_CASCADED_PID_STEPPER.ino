// PLE3ASE UNDERSTAND I WRITE COMMENTS MY OWN WAY, BUT NONETHELESSTHE BUILD VIDEO FOR THIS IS ON YOUTUBE @ LIGIENCE 
// Link; https://youtu.be/h8H8bgWLwpI

#include <Wire.h>
float Gyro_Roll, Gyro_Pitch, Gyro_Yaw;
//       p         q          r
float AccX, AccY, AccZ;
//      x     y     z 
float Acc_Roll, Acc_Pitch;

float sampling_rate = 0.004f; // sampling rate is seconds - 0.005 seconds which is = 200Hz 
unsigned long sampling_rate_timer = 4000; //  timer microseconds
unsigned long SR_current_timer;
unsigned long SR_last_timer;

// ---------------------MPU6050 ACCELEROMETER AND GYRO TRADINGS TO ROLL AND PITCH------------------

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
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read(); // raw counts sensitivity of +- 8g
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  Gyro_Roll=(float)GyroX/65.5 + 1.40; // to degrees/s then minus to calibrate to 0 
  Gyro_Pitch=(float)GyroY/65.5 - 4.13;
  Gyro_Yaw=(float)GyroZ/65.5 + 0.78;

// ---------------Accelerometer direct values to acceleration in g, calibration offfset and then to pitch and roll 

  AccX=(float)AccXLSB/4096.0f - 0.02f; // acceleration in g - minus is for manual calibration from readingd when at rest , can be converted to m/s and g would then = 9.81
  AccY=(float)AccYLSB/4096.0f - 0.05f; 
  AccZ=(float)AccZLSB/4096.0f - 0.029f;
  
   // Always normalize, just prevent division by zero
  float acc_magnitude = sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ);

  if (acc_magnitude > 0.1f) {
   AccX /= acc_magnitude;
   AccY /= acc_magnitude;
   AccZ /= acc_magnitude;
   }

   Acc_Pitch = atan2f(-AccX, sqrt(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG; // Value in Radians --Convert to Degrees -- sqrt(AccX² + AccY² + AccZ²) ≈ 1 g If acceleration is in g then use 1.o as "g" in any calculations but if it is in m/s2 use 9.81
   Acc_Roll = atan2f(AccY, AccZ) * RAD_TO_DEG; // Value in Radians --Convert to Degrees 
    // RAD TO DEGREE  = (180.0f / 3.14159f )
}



//-------------------Kalman filter variables-----------------------------------
// Kalman filter for roll and pitch
float KalmanAngleRoll = 0.0f;              // Filtered roll angle
float KalmanUncertaintyRoll = 2.0f * 2.0f; // Initial uncertainty

float KalmanAnglePitch = 0.0f;              // Filtered pitch angle
float KalmanUncertaintyPitch = 2.0f * 2.0f; // Initial uncertainty

// Process and measurement noise (tune these)
const float Q_angle = 0.003f; // process noise - how much you trust the gyro - higher value means higher noise = lesss trust 
const float R_angle = 0.05f;  // measurement noise - how much you trust the accelerometer - higher value means higher noise = lesss trust 


//-------------------Kalman filter function -----------------------------------

void Kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    // 1. Prediction (gyro integration)
    KalmanState += KalmanInput * sampling_rate;           // gyro * sampling_rate
    KalmanUncertainty += Q_angle * sampling_rate;        // uncertainty grows

    // 2. Compute Kalman gain
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + R_angle);

    // 3. Update state with measurement (accelerometer)
    KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);

    // 4. Update uncertainty
    KalmanUncertainty = (1.0f - KalmanGain) * KalmanUncertainty;
}

//------------------------------CONTROL MODES ----------------------

uint8_t control_mode = 1;  // 0 = angle only, 1 = angle + position

//------------------------------MOTOR POSITION TRACKING ----------------------

// Position tracking (in motor steps)
volatile int32_t motor_left_steps = 0;    // volatile because accessed in interrupt
volatile int32_t motor_right_steps = 0;   // volatile because accessed in interrupt
int32_t position_avg = 0;                 // Average position of both motors

// Control mode 0: Angle only (your original system)
// Control mode 1: Angle + Position (new cascaded system)

//------------------------------ ANGLE PID  ----------------------

float angle_setpoint = 3.0f;  // Renamed from 'setpoint' - this will be DYNAMIC now!
float angle_error = 0.0f;

float KP_angle = 280.0f;  // Renamed from KP
float Ki_angle = 20.0;    // Renamed from Ki  
float Kd_angle = 2.5;    // Renamed from Kd

float Integral_angle = 0.0F;     // Renamed from Integral
float Integral_angle_max = 500.0f;

// Derivative filter
float Derivative_angle = 0.0f;

float angle_error_previous = 0.0f;

float PID_angle_output = 0.0F;   // Renamed from PID_output


//------------------------------POSITION PID (NEW!) ----------------------

float position_setpoint = 0.0f;  // Target position (0 = stay where we started)
float position_error = 0.0f;

float KP_pos = 1.5f;     // Position P gain - START CONSERVATIVE
float Ki_pos = 0.0f;     // Position I gain - START WITH ZERO
float Kd_pos = 0.0f;     // Position D gain - helps damping

float Integral_pos = 0.0f;
float Integral_pos_max = 35.0f;  // Limit position integral (degrees)

float Derivative_pos = 0.0f;
float position_prev = 0;

float position_error_previous = 0.0f;

float PID_pos_output = 0.0f;  // This feeds into angle_setpoint!

#define PID_POS_MAX 35.0f  // Maximum angle setpoint from position control

float POSITION_DEADZONE = 0.5f;

//------------------------------ANGLE PID FUNCTION ----------------------

void angle_PID() {

    angle_error = angle_setpoint - KalmanAnglePitch;

    // -------- PROPORTIONAL --------
    float P = KP_angle * angle_error;

    // -------- INTEGRAL (ANTI-WINDUP) --------
    if (fabsf(angle_error) < 8.0f && fabsf(PID_angle_output) < 500.0f) {
        Integral_angle += angle_error * sampling_rate;
        Integral_angle = constrain(Integral_angle, -Integral_angle_max, Integral_angle_max);
    }
    else {
        Integral_angle = 0.0f;  // ← ADDED: Reset when conditions not met
    }

    float I = Ki_angle * Integral_angle;

    // -------- DERIVATIVE ---------

   // Derivative_angle = (angle_error - angle_error_previous) / sampling_rate ;

   // angle_error_previous = angle_error;

   // float D = Kd_angle * Derivative_angle;

    float D = -Kd_angle * Gyro_Pitch;

    // -------- OUTPUT--------
    PID_angle_output = P + I + D;
}

//------------------------------POSITION PID FUNCTION ----------------------

void position_PID() {
    // Calculate position error (convert steps to normalized value)
     position_error = (position_setpoint - position_avg) / 1000.0f;

    float P = KP_pos * position_error;

    Integral_pos += position_error * sampling_rate;
    Integral_pos = constrain(Integral_pos, -Integral_pos_max, Integral_pos_max);
    
    // Reset if very far
    if (fabsf(position_error) > 2.0f) {
        Integral_pos = 0.0f;
    }

    float I = Ki_pos * Integral_pos;

    // In position_PID():
    float pos_rate = (float)(position_avg - position_prev) / sampling_rate;
    //                ← Cast to float
    position_prev = position_avg;

    float D = -Kd_pos * (pos_rate / 1000.0f);

    PID_pos_output = P + I + D;

    PID_pos_output = constrain( PID_pos_output,-PID_POS_MAX, PID_POS_MAX);
}


//------------------------------MotorFUNCTION ----------------------

//-----------------------------MOTOR ESSENTIAL DATA 
// steps per revolution for my nema 17 is 200 at 1.8degrees per step 
// using the microstep of 1/16th gives 32000 steps per revolution
// Currently set to 1/8th step on a 200 steps per revolution 
float p ;

int steps_per_rev = 1600; // 1/8th step on a 200 steps per revolution 

//---------------------------- stepper motor 1 
const int DIR = 18;
const int STEP = 19;
const int MS1_M1 = 16;
const int MS2_M1 = 4;

//----------------------------stepper motor 2 

const int DIR2 = 17;
const int STEP2 = 5;
const int MS1_M2 = 0;
const int MS2_M2 = 2;

unsigned long stepper_motor_us_pulse ;

unsigned long M_Current_timer ;
unsigned long M_Last_timer ;

//------------------------------DYNAMIC MICROSTEPPING ----------------------

uint8_t microStep = 8;           // Start at 1/4 step (less vibration)
uint8_t lastMicroStep = 8;

//------------------------------SET MICROSTEPPING FUNCTION (CORRECTED) ----------------------

void setMicroStep(uint8_t uStep) {
    switch(uStep) {
        case 2:  // Half step (1/2) - HIGH torque
            digitalWrite(MS1_M1, LOW);
            digitalWrite(MS2_M1, HIGH);
            digitalWrite(MS1_M2, LOW);
            digitalWrite(MS2_M2, HIGH);
            steps_per_rev = 400;
            break;
            
        case 4:  // Quarter step (1/4) - Good balance
            digitalWrite(MS1_M1, HIGH);
            digitalWrite(MS2_M1, LOW);
            digitalWrite(MS1_M2, HIGH);
            digitalWrite(MS2_M2, LOW);
            steps_per_rev = 800;
            break;
            
        case 8:  // Eighth step (1/8) - DEFAULT, smooth
            digitalWrite(MS1_M1, LOW);
            digitalWrite(MS2_M1, LOW);
            digitalWrite(MS1_M2, LOW);
            digitalWrite(MS2_M2, LOW);
            steps_per_rev = 1600;
            break;
            
        case 16:  // 1/16 step - Smoothest (with internal interpolation)
            digitalWrite(MS1_M1, HIGH);
            digitalWrite(MS2_M1, HIGH);
            digitalWrite(MS1_M2, HIGH);
            digitalWrite(MS2_M2, HIGH);
            steps_per_rev = 3200;
            break;
            
        default:  // Fallback to 1/8 (most common default)
            digitalWrite(MS1_M1, LOW);
            digitalWrite(MS2_M1, LOW);
            digitalWrite(MS1_M2, LOW);
            digitalWrite(MS2_M2, LOW);
            steps_per_rev = 1600;
            uStep = 8;
            break;
    }
    
    microStep = uStep;
    
   //Serial.print("Microstepping: 1/");
   //Serial.println(microStep);
   //Serial.print(" = ");
   //Serial.print(steps_per_rev);
   //Serial.println(" steps/rev");
}

// -------MultiCore Processing--------------
TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  // Serial initiialisation
  Serial.begin(115200);
  // I2C Initialisation
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1_M1 , OUTPUT);
  pinMode(MS2_M1 , OUTPUT);

  
  pinMode(STEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(MS1_M2, OUTPUT);
  pinMode(MS2_M2, OUTPUT);

  setMicroStep(8);
  control_mode = 1;

  delay(250);

  M_Last_timer = micros(); 
  SR_last_timer = micros();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 

}

  // CALCULATION TASKS RUNNING ON CORE 0 

  void Task1code( void * pvParameters ){

  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
   
   SR_current_timer = micros();
   if (SR_current_timer - SR_last_timer >= sampling_rate_timer ){

    Accel_gyro_signals();
    
    // Apply Kalman filter
   //Kalman_1d(KalmanAngleRoll, KalmanUncertaintyRoll, Gyro_Roll, Acc_Roll);
   Kalman_1d(KalmanAnglePitch, KalmanUncertaintyPitch, Gyro_Pitch, Acc_Pitch);

   //Serial.print("Kalman Roll: "); Serial.println(KalmanAngleRoll, 2);
   Serial.print("Kalman_Pitch:"); Serial.println(KalmanAnglePitch, 2);
   
   // Average position from both motors (in steps)
   position_avg = (motor_left_steps + motor_right_steps) / 2;

   //Serial.print(" Pos:");
   //Serial.print(position_avg);
   //Serial.print(" L:");
   //Serial.print(motor_left_steps);
   //Serial.print(" R:");
   //Serial.println(motor_right_steps);

// ===== CASCADED CONTROL =====
    
    if (control_mode == 0) {
        // Mode 0: Angle only (original behavior)
        angle_setpoint = 3.0f;
        angle_PID();
        
    } else if (control_mode == 1) {
        // Mode 1: Position + Angle (cascaded)
        position_PID();                    // Outer loop: calculate desired angle
        angle_setpoint = PID_pos_output + 2.0f ;  // Add your C.G. offset
        angle_PID();                       // Inner loop: achieve that angle
    }

// Convert PID output to motor speed (same as before)
    p = fabsf(PID_angle_output);  // Changed from PID_output
    //Serial.print("PID_ANGLE_OUTPUT: ");   Serial.println(PID_angle_output);  
    p = constrain(p, 0.001f, 40000.0f);
    
// ======== DYNAMIC MICROSTEPPING ==========
   setMicroStep(8);
   // lastMicroStep = microStep;

   // if (microStep == 16) {
      // Currently at 1/16 → Switch to 1/8 if tilting
   //   if (fabsf(angle_error) > 1.0f) {
   //    microStep = 8;
   //   }
   // }
   // else if (microStep == 8) {
      // Currently at 1/8 → Only switch back to 1/16 (NO 1/4 step!)
   //  if (fabsf(angle_error) < 0.5f) {
   //   microStep = 16;
   //   }
   // }

   // if (microStep != lastMicroStep) { setMicroStep(microStep); }

// ======== DIRECTION CONTROL ==========    
    if (PID_angle_output > 0) {  // Changed from PID_output
        digitalWrite(DIR, LOW);
        digitalWrite(DIR2, HIGH);
        stepper_motor_us_pulse = (unsigned long)(1000000.0f/p);
    }
    else if(PID_angle_output < 0) {  // Changed from PID_output
        digitalWrite(DIR, HIGH);
        digitalWrite(DIR2, LOW);
        stepper_motor_us_pulse = (unsigned long)(1000000.0f/p);
    }
  
    // Fall detection (same as before)
   if (fabsf(angle_error) < 0.3f) {
    stepper_motor_us_pulse = 100000UL;
    }

    if (fabsf(angle_error) > 45.0f) {  // Changed from error
        stepper_motor_us_pulse = 100000UL;
    }
    //Serial.print("stepper_motor_us_pulse: ");
    //Serial.println(stepper_motor_us_pulse);

    SR_last_timer = SR_current_timer;
    }

   } 

  }



  void Task2code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    
    M_Current_timer = micros();
    if (M_Current_timer - M_Last_timer >= stepper_motor_us_pulse){
     
     digitalWrite(STEP, HIGH);
     digitalWrite(STEP2, HIGH);
     delayMicroseconds(1);        // 1–2 µs pulse width
     digitalWrite(STEP, LOW);
     digitalWrite(STEP2, LOW);

     
  //    ORRRRRRRR........
  //    digitalWrite(STEP, !digitalRead(STEP));
  //   digitalWrite(STEP2, !digitalRead(STEP2));


    // Check direction and increment/decrement step counters
     
     // Motor 1 (left)
     if (digitalRead(DIR) == LOW) {
       motor_left_steps++;   // Moving forward
     } else {
       motor_left_steps--;   // Moving backward
     }
     
     // Motor 2 (right)
     if (digitalRead(DIR2) == HIGH) {
       motor_right_steps++;  // Moving forward
     } else {
       motor_right_steps--;  // Moving backward
     }

    M_Last_timer = M_Current_timer;

    }

  } 
  }



  void loop() {                         }
