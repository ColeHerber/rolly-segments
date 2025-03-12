// #include <FreeRTOS.h>
#include "pins_arduino.h" // Include our custom pins for AXIS board
#include <SimpleFOC.h>

#include "SimpleFOCDrivers.h"

#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include <ESP32Servo.h>

int pole_pairs = 7;
// BLDCMotor motor0 = BLDCMotor(pole_pairs);
BLDCMotor motor1 = BLDCMotor(pole_pairs);

// BLDCDriver6PWM driver0 = BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(CH1_UH, CH1_UL, CH1_VH, CH1_VL, CH1_WH, CH1_WL);

// Setup sensors for the two channels
// MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);
MagneticSensorMT6701SSI encoder1(CH1_ENC_CS);

SPIClass hspi = SPIClass(HSPI);


void setup() {

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  encoder1.init(&hspi);

  motor1.linkSensor(&encoder1);


  driver1.init();
  motor1.linkDriver(&driver1);

  motor1.controller = MotionControlType::velocity;

  // init motor hardware
  motor1.init();
  motor1.initFOC();

  motor1.current_limit = 1; // Amps

  motor1.PID_velocity.P = 0.5;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0.001;

  // motor1.LPF_velocity.Tf = 0.01;
  // motor1.PID_velocity.output_ramp = 1000;

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");
  motor1.useMonitoring(Serial);

  delay(1000);



}
  
// velocity set point variable
float target_velocity = 10; // 10Rad/s ~ 200rpm

void loop() {
  // main FOC algorithm function
  motor1.loopFOC();

  // Motion control function
  motor1.move(target_velocity);


  // Get encoder velocity (rad/s)
  float velocity = encoder1.getVelocity();

  // Print velocity
  Serial.print("Velocity (rad/s): ");
  Serial.println(velocity);

  delay(100);


}
