// #include <FreeRTOS.h>
#include "pins_arduino.h" // Include custom pins for AXIS board
#include <SimpleFOC.h>
// #include <BluetoothSerial.h> //might not work look here if failing
#include <ESP32Servo.h>

#include "SimpleFOCDrivers.h"

#include <encoders/mt6701/MagneticSensorMT6701SSI.h>


int pole_pairs = 7;
BLDCMotor motor0 = BLDCMotor(pole_pairs);
BLDCMotor motor1 = BLDCMotor(pole_pairs);

BLDCDriver6PWM driver0 = BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(CH1_UH, CH1_UL, CH1_VH, CH1_VL, CH1_WH, CH1_WL);

// Setup sensors for the two channels
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);
MagneticSensorMT6701SSI encoder1(CH1_ENC_CS);

SPIClass hspi = SPIClass(HSPI);

// BluetoothSerial SerialBT; 
// int servo_angles[4] = {90,90,90,90}; // Default positions

void setup() {
  // // Bluetooth setup
  // SerialBT.begin("ESP32_RC_Car");
  // Serial.println("Bluetooth started!");


  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  encoder0.init(&hspi);
  encoder1.init(&hspi);

  motor0.linkSensor(&encoder0);
  motor1.linkSensor(&encoder1);


  driver0.voltage_power_supply = 12;
  driver0.init();
  motor0.linkDriver(&driver0);

  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);


  motor0.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  // // init motor hardware
  motor0.init();
  motor0.initFOC();

  motor1.init();
  motor1.initFOC();


  motor0.PID_velocity.P = 0.5;
  motor0.PID_velocity.I = 2;
  motor0.PID_velocity.D = 0.001;


  motor1.PID_velocity.P = 0.5;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0.001;

  // motor0.P_angle.P = 10; 
  // motor0.P_angle.I = 5;  // usually only P controller is enough 
  // motor0.P_angle.D = -0.001;  // usually only P controller is enough 


  // motor1.P_angle.P = 1; 
  // motor1.P_angle.I = 0;  // usually only P controller is enough 
  // motor1.P_angle.D = 0;  // usually only P controller is enough 


  motor0.LPF_velocity.Tf = 0.01;
  motor0.PID_velocity.output_ramp = 5; //VERY IMPORTANT SAFTEY FEATURE sets jerk to be this rad/s^3 (5 is good)
  // motor0.P_angle.output_ramp = 10; // default 1e6 rad/s^2
  motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor0.velocity_limit = 500; // rad/s, max rpm is about this
  motor0.current_limit = 2;


  motor1.LPF_velocity.Tf = 0.01;
  motor1.PID_velocity.output_ramp = 5; //VERY IMPORTANT SAFTEY FEATURE sets jerk to be this rad/s^3 (5 is good)
  // motor1.P_angle.output_ramp = 10; // default 1e6 rad/s^2
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.velocity_limit = 500; // rad/s, max rpm is about this
  motor1.current_limit = 2;



  // Servo servo1, servo2, servo3, servo4;
  // servo1.attach(13);
  // servo2.attach(12);
  // servo3.attach(14);
  // servo4.attach(27);

  delay(1000);



}
float forward_velocity = 0;
float turn_velocity = 0;

// velocity set point variable
float target_velocity = 20; 

void loop() {
  // main FOC algorithm function

  motor0.loopFOC();
  motor0.move(target_velocity);

  motor1.loopFOC();
  motor1.move(target_velocity);

  // Get encoder velocity (rad/s)
  float angle = encoder1.getAngle();

  // Print velocity
  Serial.print("angle: ");
  Serial.println(angle);

  delay(10);
  // handleBluetoothInput();
  // servo1.write(servo_angles[0]);
  // servo2.write(servo_angles[1]);
  // servo3.write(servo_angle[2]);
  // servo4.write(servo_angle[3]);


}

// void handleBluetoothInput() {
//   if (SerialBT.available()) {
//     String input = SerialBT.readStringUntil('\n');
//     parseCommand(input);
//   }
// }

// void parseCommand(String msg) {
//   // Expected: forward,turn,s1,s2,s3,s4\n
//   int idx1 = msg.indexOf(',');
//   int idx2 = msg.indexOf(',', idx1+1);
//   int idx3 = msg.indexOf(',', idx2+1);
//   int idx3 = msg.indexOf(',', idx2+1);
//   int idx4 = msg.indexOf(',', idx2+1);
//   int idx5 = msg.indexOf(',', idx3+1);

//   if (idx5 == -1) return; // Ensure full message received

//   forward_velocity = msg.substring(0, idx1).toFloat();
//   turn_velocity = msg.substring(idx1+1, idx2).toFloat();
//   servo_angle[0] = msg.substring(idx2+1, idx3).toInt();
//   servo_angle[1] = msg.substring(idx3+1, idx4).toInt();
//   servo_angle[2] = msg.substring(idx4+1, idx5).toInt();
//   servo_angle[3] = msg.substring(idx5+1).toInt();
// }