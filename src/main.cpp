#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <FreeRTOS.h>
#include <SimpleFOC.h>
#include <wifi.h>

#include <atomic>

#include "axis_mqtt_tools.h"    // Include our WiFi header
#include "axis_wifi_manager.h"  // Include our MQTT header
#include "imu.h"
#include "pins_arduino.h"  // Include our custom pins for AXIS board
#define VERSION "1.0.157"   // updated dynamically from python script

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

// motor parameters
int pole_pairs = 7;
float phase_resistance = 3.5;
float kv = 82.5;

// Setup the motor and driver0
BLDCMotor motor0 = BLDCMotor(pole_pairs, phase_resistance, kv);
BLDCDriver6PWM driver0 =
    BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);

BLDCMotor motor1 = BLDCMotor(pole_pairs, phase_resistance, kv);
BLDCDriver6PWM driver1 =
    BLDCDriver6PWM(CH1_UH, CH1_UL, CH1_VH, CH1_VL, CH1_WH, CH1_WL);

// make encoder for simplefoc
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);
MagneticSensorMT6701SSI encoder1(CH1_ENC_CS);


// calibrated sensor object from simplefoc
CalibratedSensor sensor0 = CalibratedSensor(encoder0);
CalibratedSensor sensor1 = CalibratedSensor(encoder1);


// IMU
Imu::Imu imu;

// global atomic variable for the motor stuff to be set by mqtt
std::atomic<float> last_commanded_target0 = 0;
std::atomic<float> last_commanded_target1 = 0;

std::atomic<float> command_vel_p_gain = 0.03;
std::atomic<float> command_vel_i_gain = 0.6;
std::atomic<float> command_vel_d_gain = 0.0;
std::atomic<float> command_vel_lpf = 0.0001;

std::atomic<bool> enable_flag = true;
std::atomic<bool> disable_flag = false;
std::atomic<bool> motors_enabled = true;

// 0 for torque, 1 for velocity, 2 for position
std::atomic<uint> last_commanded_mode = 1;

// make a separate thread for the OTA
TaskHandle_t loop_foc_task;
// make a separate thread for the MQTT publishing
TaskHandle_t mqtt_publish_task;
const int interval_ms = 500;
void mqtt_publish_thread(void *pvParameters)
{
  while (1)
  {
    static unsigned long lastMsg = millis();
    // Handle MQTT connection
    if (!isMQTTConnected())
    {                   // Use our MQTT connection check function
      reconnectMQTT();  // Use our MQTT reconnect function
    }
    mqttLoop();  // Handle MQTT client loop (IMPORTANT)

    // Publish data periodically
    if (millis() - lastMsg > interval_ms)
    {
      // Create JSON document to send data in
      StaticJsonDocument<512> doc;

      // print target of foc
      doc["target0"] = motor0.target;
      doc["target1"] = motor1.target;

      // print the encoder position
      // doc["pos"] = motor0.shaft_angle;
      // print the encoder velocity
      doc["vel0"] = motor0.shaft_velocity;
      doc["vel1"] = motor0.shaft_velocity;


      // print the gains
      doc["vel_p"] = motor0.PID_velocity.P;
      doc["vel_i"] = motor0.PID_velocity.I;
      doc["vel_d"] = motor0.PID_velocity.D;
      doc["vel_lpf"] = motor0.LPF_velocity.Tf;
      Imu::gravity_vector_t gravity = imu.get_gravity_vector();
      doc["gravity_x"] = gravity.x;
      doc["gravity_y"] = gravity.y;
      doc["gravity_z"] = gravity.z;

      // Serialize JSON to string
      char buffer[512];
      serializeJson(doc, buffer, sizeof(buffer));

      // Publish the message
      publishMQTT(buffer);  // Use our MQTT publish function
    }
    ArduinoOTA.handle();
    vTaskDelay(interval_ms / portTICK_PERIOD_MS);


  }

}

void loop_foc_thread(void *pvParameters)
{
  while (1)
  {
    // // Service flags
    // if (enable_flag)
    // {
    //   //   Serial.println("Motors are enabled");
    // }
    // else if (disable_flag)
    // {
    //   //   Serial.println("Motors are disabled");
    //   motor0.disable();
    //   motor1.disable();

    //   disable_flag.store(false);
    //   motors_enabled.store(false);
    // }

    // loop simplefoc
    motor0.move(last_commanded_target0.load());
    motor0.loopFOC();
    motor1.move(last_commanded_target1.load());
    motor1.loopFOC();

    imu.loop();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting setup...");
  Serial.print("Version: ");
  Serial.println(VERSION);

  // Initialize WiFi
  setupWiFi();  // Call our WiFi setup function

  // Init and calibrate the IMU
  imu.init(true);

  ArduinoOTA.setHostname(NAME);
  ArduinoOTA.onStart(
      []()
      {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
          type = "sketch";
        }
        else
        {  // U_SPIFFS
          type = "filesystem";
        }
        Serial.println("Start updating " + type);
      });


  ArduinoOTA.onEnd([]() { Serial.println("\nEnd OTA Update"); });

  ArduinoOTA.onProgress(
      [](unsigned int progress, unsigned int total)
      { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError(
      [](ota_error_t error)
      {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();

  // LED indicator setup
  // pinMode(LED_BUILTIN, OUTPUT);  // BLUE LED 44
  // pinMode(43, OUTPUT);           // GREEN LED 43

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  delay(1000);
  // initialize encoder
  encoder0.init(&hspi);
  encoder1.init(&hspi);

  // calibrated sensor

  // motor driver0 setup
  driver0.voltage_power_supply = 16;
  driver0.voltage_limit = 16;
  driver0.init();

  // motor driver0 setup
  driver1.voltage_power_supply = 16;
  driver1.voltage_limit = 16;
  driver1.init();

  // link motor to driver0 and set up
  motor0.linkDriver(&driver0);
  // motor0.voltage_sensor_align = 0.25;
  motor0.current_limit = 5;
  motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor0.torque_controller = TorqueControlType::voltage;

  // link motor to driver0 and set up
  motor1.linkDriver(&driver1);
  // motor0.voltage_sensor_align = 0.25;
  motor1.current_limit = 5;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.torque_controller = TorqueControlType::voltage;
 

  // set pid values for velocity controller
  motor0.PID_velocity.P = 0.025;
  motor0.PID_velocity.I = 0.3;
  motor0.PID_velocity.D = 0;
  motor0.PID_velocity.output_ramp = 50;
  motor0.PID_velocity.limit = 100;
  motor0.LPF_velocity.Tf = 0.01;
  motor0.P_angle.P = 10;
  motor0.controller = MotionControlType::velocity;

  motor0.init();

  motor0.linkSensor(&sensor0);

  if (motor0.initFOC()) {
    digitalWrite(LED_BUILTIN, LOW);

  }


  motor1.PID_velocity.P = 0.025;
  motor1.PID_velocity.I = 0.3;
  motor1.PID_velocity.D = 0;
  motor1.PID_velocity.output_ramp = 50;
  motor1.PID_velocity.limit = 100;
  motor1.LPF_velocity.Tf = 0.001;
  motor1.P_angle.P = 10;
  motor1.controller = MotionControlType::velocity;

  motor1.init();

  // align sensor and start FOC
  // sensor.voltage_calibration = 0.5;
  // sensor.calibrate(motor);
  motor1.linkSensor(&sensor1);

  if (motor1.initFOC()) {
    digitalWrite(43, LOW);

  }


  setupMQTT();                                 // Call our MQTT setup function
  xTaskCreatePinnedToCore(mqtt_publish_thread, /* Task function. */
                          "MQTT_Publish",      /* String with name of task. */
                          10000,               /* Stack size in bytes. */
                          NULL, /* Parameter passed as input of the task */
                          1,    /* Priority of the task. */
                          &mqtt_publish_task, /* Task handle. */
                          1); /* Core 1 because wifi runs on core 0 */

  // task for arduinoOTA


  xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 1,
                          &loop_foc_task, 1);

  Serial.println("Setup complete.");

  motor0.enable();
  motor1.enable();

  enable_flag.store(true);
  motors_enabled.store(true);

}

void loop()
{
  // if commands have changed, disable the motor, update the values, and
  // re-enable the motor
  if (last_commanded_mode.load() != motor0.controller)
  {
    disable_flag.store(true);
    delay(1);
    while (motors_enabled.load())
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    uint controlmode = (MotionControlType)last_commanded_mode.load();
    switch (controlmode)
    {
      case 0:
        motor0.controller = MotionControlType::velocity;
        motor1.controller = MotionControlType::velocity;
        break;
      case 1:
        motor0.controller = MotionControlType::torque;
        motor1.controller = MotionControlType::torque;
        break;
      case 2:
        motor0.controller = MotionControlType::velocity_openloop;
        motor1.controller = MotionControlType::velocity_openloop;
        break;
      default:
        motor0.controller = MotionControlType::velocity;
        motor1.controller = MotionControlType::velocity;
        break;
    }
    enable_flag.store(true);
  }

  //   if the gains have changed, disable the motor, update the values, and
  //   re-enable the motor
  if ((command_vel_p_gain.load() != motor0.PID_velocity.P) ||
      (command_vel_i_gain.load() != motor0.PID_velocity.I) ||
      (command_vel_d_gain.load() != motor0.PID_velocity.D) ||
      (command_vel_lpf.load() != motor0.LPF_velocity.Tf) )
  {
    disable_flag.store(true);
    delay(1);
  while (motors_enabled.load())
  {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

    motor0.PID_velocity.P = command_vel_p_gain.load();
    motor0.PID_velocity.I = command_vel_i_gain.load();
    motor0.PID_velocity.D = command_vel_d_gain.load();
    motor0.LPF_velocity.Tf = command_vel_lpf.load();

    motor1.PID_velocity.P = command_vel_p_gain.load();
    motor1.PID_velocity.I = command_vel_i_gain.load();
    motor1.PID_velocity.D = command_vel_d_gain.load();
    motor1.LPF_velocity.Tf = command_vel_lpf.load();

    enable_flag.store(true);
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);

  //   Handle OTA updates
  ArduinoOTA.handle();

}