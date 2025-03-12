// #include <FreeRTOS.h>
#include "pins_arduino.h" // Include our custom pins for AXIS board
#include <SimpleFOC.h>

#include "SimpleFOCDrivers.h"

#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include <ESP32Servo.h>

int pole_pairs = 7;
BLDCMotor motor0 = BLDCMotor(pole_pairs);
BLDCMotor motor1 = BLDCMotor(pole_pairs);

BLDCDriver6PWM driver0 = BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(CH1_UH, CH1_UL, CH1_VH, CH1_VL, CH1_WH, CH1_WL);

// Setup sensors for the two channels
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);
MagneticSensorMT6701SSI encoder1(CH1_ENC_CS);

SPIClass hspi = SPIClass(HSPI);


void setup() {


  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");
  // motor0.useMonitoring(Serial);

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  // encoder0.init(&hspi);
  encoder1.init(&hspi);

  // motor0.linkSensor(&encoder0);
  motor1.linkSensor(&encoder1);


  // driver0.voltage_power_supply = 12;
  // driver0.init();
  // motor0.linkDriver(&driver0);

    driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);


  // motor0.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  // // init motor hardware
  // motor0.init();
  // motor0.initFOC();

  motor1.init();

  motor1.initFOC();


  // // motor0.current_limit = 1; // Amps

  // motor0.PID_velocity.P = 0.1;
  // motor0.PID_velocity.I = 10;
  // motor0.PID_velocity.D = 1;


  motor1.PID_velocity.P = 0.1;
  motor1.PID_velocity.I = 0.5;
  motor1.PID_velocity.D = -1;

  // motor0.voltage_limit = 12; // Volts - default driver.voltage_limit


  // motor0.P_angle.P = 10; 
  // motor0.P_angle.I = 5;  // usually only P controller is enough 
  // motor0.P_angle.D = -0.001;  // usually only P controller is enough 


  motor1.P_angle.P = 1; 
  motor1.P_angle.I = 0;  // usually only P controller is enough 
  motor1.P_angle.D = 0;  // usually only P controller is enough 


  // motor0.LPF_velocity.Tf = 0.01;
  // motor0.PID_velocity.output_ramp = 1000;

  motor1.LPF_velocity.Tf = 0.01;
  motor1.PID_velocity.output_ramp = 1; //VERY IMPORTANT SAFTEY FEATURE
  motor1.P_angle.output_ramp = 10; // default 1e6 rad/s^2

  motor1.velocity_limit = 2; // rad/s - default 20


  delay(1000);



}
  
// velocity set point variable
float target_velocity = 1; 

void loop() {
  // main FOC algorithm function
  // motor0.loopFOC();

  motor1.loopFOC();

  // Motion control function
  // motor0.move(target_velocity);
  motor1.move(target_velocity);


  // Get encoder velocity (rad/s)
  float angle = encoder1.getAngle();

  // Print velocity
  Serial.print("angle: ");
  Serial.println(angle);

  delay(10);


}
