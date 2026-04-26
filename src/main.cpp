
/* 2804 gimbal motor (BLDC) test code.  This was completely written by chatGPT.
   Reads a potentiometer and sets the speed to between 50-400rpm.  Varies torque
   to keep the rpm constant.

   dlf  4/25/2026
*/


#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// ======================================================
// Pin Definitions
// ======================================================

// 3PWM outputs to DRV8313 board
#define PWM_U      12
#define PWM_V      15
#define PWM_W      26

// Driver enable
#define DRIVER_EN  32

// I2C for AS5600
#define I2C_SDA    21
#define I2C_SCL    22

// Potentiometer input
#define POT_PIN    33

// ======================================================
// Motor Objects
// ======================================================

// 7 pole pair gimbal motor
BLDCMotor motor = BLDCMotor(7);

// DRV8313 3PWM driver
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_U, PWM_V, PWM_W, DRIVER_EN);

// AS5600 encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ======================================================

const float SUPPLY_VOLTAGE = 12.0;
const float VOLTAGE_LIMIT  = 6.0;   // safe startup
const float MIN_RPM = 50.0;
const float MAX_RPM = 400.0;

unsigned long lastPrint = 0;

// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("DRV8313 SimpleFOC Velocity Control");

  // ADC setup
  analogReadResolution(12);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Sensor
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.pwm_frequency = 25000;   // ideal for DRV8313
  driver.init();

  motor.linkDriver(&driver);

  // Velocity control mode
  motor.controller = MotionControlType::velocity;

  // PID tuning
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 3.0;
  motor.PID_velocity.D = 0.0;

  // Motor was shaking using this setting.  Needed more low-pass-filtering of the AS5600
  // Set it to .03 and it smoothed out.
  //motor.LPF_velocity.Tf = 0.01;  
  motor.LPF_velocity.Tf = 0.03;

  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 60.0;   // rad/s (~573 rpm)

  motor.init();
  motor.initFOC();

  Serial.println("Motor Ready");
}

// ======================================================

void loop() {

  // FOC algorithm
  motor.loopFOC();

  // Read potentiometer
  int raw = analogRead(POT_PIN);

  // Scale to RPM
  float targetRPM = MIN_RPM + ((float)raw / 4095.0f) * (MAX_RPM - MIN_RPM);

  // RPM -> rad/s
  float targetRad = targetRPM * _2PI / 60.0f;

  // Command motor
  motor.move(targetRad);

  // Serial monitor
  if (millis() - lastPrint > 250) {
    lastPrint = millis();

    float actualRPM = motor.shaft_velocity * 60.0f / _2PI;

    Serial.print("Pot=");
    Serial.print(raw);
    Serial.print("  Target=");
    Serial.print(targetRPM, 1);
    Serial.print(" RPM   Actual=");
    Serial.print(actualRPM, 1);
    Serial.println(" RPM");
  }
}