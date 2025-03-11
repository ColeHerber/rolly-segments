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

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
int minUs = 500;
int maxUs = 2500;

int pos = 60;      // position in degrees
ESP32PWM pwm;

void setup() {
  //Unknown, example preallocates before serial
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");
  delay(1000);

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);


  // initialise encoder hardware
  // encoder0.init(&hspi);
  encoder1.init(&hspi);


  // motor0.linkSensor(&encoder0);
  motor1.linkSensor(&encoder1);


  // driver0.voltage_power_supply = 12;
  driver1.voltage_power_supply = 14.8;
  
  // driver0.voltage_limit = 8;
  // driver1.voltage_limit = 8;

  // driver0.init();
  driver1.init();

  // motor0.linkDriver(&driver0);
  motor1.linkDriver(&driver1);

  // motor0.voltage_sensor_align = 1;
  motor1.voltage_sensor_align = 1;

  // motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // motor0.torque_controller = TorqueControlType::voltage;
  motor1.torque_controller = TorqueControlType::voltage;

  // set pid values for velocity controller
  // motor0.PID_velocity.P = 0.2;
  // motor0.PID_velocity.I = 0.1;
  // motor0.PID_velocity.D = 0.01;
  // motor0.PID_velocity.output_ramp = 1;

  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 0.1;
  motor1.PID_velocity.D = 0.01;
  // motor1.PID_velocity.output_ramp = 1;


  // motor0.LPF_velocity.Tf = 0.01;
  // motor1.LPF_velocity.Tf = 0.01;


  // motor0.P_angle.P = 0.1;
  // motor0.P_angle.I = 1;  // usually only P controller is enough 
  // motor0.P_angle.D = 0.01;  // usually only P controller is enough 

  motor1.P_angle.P = 0.1;
  motor1.P_angle.I = 1;  
  motor1.P_angle.D = 0.01; 

  // motor0.velocity_limit = 2;
  // motor1.velocity_limit = 2;


  // motor0.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  servo0.setPeriodHertz(50);      // Standard 50hz servo
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo2.setPeriodHertz(50);      // Standard 50hz servo
	servo3.setPeriodHertz(200);      // Standard 50hz servo
	
  servo0.attach(PWM_0, minUs, maxUs);
  servo1.attach(PWM_1, minUs, maxUs);
  servo2.attach(PWM_2, minUs, maxUs);
  servo3.attach(PWM_3, minUs, maxUs);


  
  
  if (motor1.initFOC())  Serial.println("FOC init 1 success!");
  else{
    Serial.println("FOC init 1 failed!");
    servo0.write(120);
    return;
    delay(1000);
  }
  // if (motor0.initFOC())  Serial.println("FOC init 0 success!");
  // else{
  //   Serial.println("FOC init 0 failed!");
  //   return;
  // }
  
  
  // motor0.KV_rating = 460;
  motor1.KV_rating = 460;


  delay(1000);

  Serial.println("Setup complete.");
}
float target_angle = 1;
// timestamp for changing direction
long timestamp_us = _micros();
  

void loop() {// angle set point variable
    // each one second
    if(_micros() - timestamp_us > 1e6) {
        timestamp_us = _micros();
        // inverse angle
        target_angle = -target_angle;   
        pos = -pos;
    }
  
  // encoder0.update();
  encoder1.update();

  servo0.write(pos);
  servo1.write(pos);
  servo2.write(pos);      
  servo3.write(pos);

  // motor0.loopFOC();
  motor1.loopFOC();

  // motor0.move(target_angle);
  motor1.move(target_angle);
  
  delay(10);

    
  // display the angle and the angular velocity to the terminal
  // Serial.print(encoder0.getSensorAngle());

  Serial.print(" ");
  Serial.print(encoder1.getSensorAngle());
  Serial.print("\t");


}