#include "pins_arduino.h" // Include custom pins for AXIS board
#include <SimpleFOC.h>
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

// Time point for referencing in loop serial log
unsigned long time_point = 0;

// make a task handle for the servo
TaskHandle_t servoTask;
uint8_t servoTaskFreq = 1000; // 1kHz

// Servos on pins 37, 39, 40, 38
uint8_t servoPins[4] = {37, 39, 40, 38};
Servo servo[4];

// servoTask callback to run a sinusoidal motion on the servos
// it will run at the frequency of servoTaskFreq in Hz
// the amplitude of the motion is 75 degrees
void servoTaskCallback(void *pvParameters) {
  // set the initial position
  float angle = 0;
  // set the amplitude of the sinusoidal motion
  float amplitude = 75;
  // set the frequency of the sinusoidal motion
  float frequency = 0.3;
  // set the initial time
  unsigned long time = millis();

  while(1) {
    // calculate the new angle
    angle = amplitude * sin(2 * PI * frequency * (millis() - time) / 1000);
    // write the new angle to the servos
    for (int i = 0; i < 4; i++) {
      servo[i].write(90 + angle);
    }
    // delay to control the frequency of the task
    vTaskDelay(1000 / servoTaskFreq / portTICK_PERIOD_MS);
  }
}


void setup() {

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

  // attach the servos to their pins
  for (int i = 0; i < 4; i++) {
    servo[i].attach(servoPins[i]);
  }

  // start the servo task pinned to core 1
  xTaskCreatePinnedToCore(servoTaskCallback, "servoTask", 2048, NULL, 1, &servoTask, 1);

  delay(1000);
}

// velocity set point variable
float target_velocity = 100; 

void loop() {
  // main FOC algorithm function

  motor0.loopFOC();
  motor0.move(target_velocity);

  motor1.loopFOC();
  motor1.move(target_velocity);

  // Get encoder velocity (rad/s)
  float angle = encoder1.getAngle();
  float velocity = encoder1.getVelocity();

  // // Print velocity on 100hz updates
  // if (millis() - time_point > 100) {
  //   time_point = millis();
  //   Serial.print("Velocity: ");
  //   Serial.print(velocity);
  //   Serial.print("rad/s   |   Angle: ");
  //   Serial.print(angle);
  //   Serial.println("rad");
  // }

  delay(1);
}
