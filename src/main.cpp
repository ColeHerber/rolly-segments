#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <FreeRTOS.h>
#include <SimpleFOC.h>
#include <wifi.h>

#include <atomic>
#include <array>

#include "axis_mqtt_tools.h"    // Include our WiFi header
#include "axis_wifi_manager.h"  // Include our MQTT header
#include "imu.h"
#include "pins_arduino.h"  // Include our custom pins for AXIS board
#define VERSION "2.0.142"   // updated dynamically from python script

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

// motor parameters
int pole_pairs = 7;
float phase_resistance = 3.5;
float kv = 82.5;

// Global atomic variables for the balance PID gains for theta error
std::atomic<float> command_bal_p_gain_theta = 16;
std::atomic<float> command_bal_i_gain_theta = 1;
std::atomic<float> command_bal_d_gain_theta = -0.1;

// Global atomic variables for the balance PID gains for theta_dot
std::atomic<float> command_bal_p_gain_theta_dot = 1.3;
std::atomic<float> command_bal_i_gain_theta_dot = 0.0;
std::atomic<float> command_bal_d_gain_theta_dot = 0.0;

// --- Custom Manual PID Controller for Balancing ---
class ManualBalancePID {
public:
    // Public members to reflect the current gains
    float P, I, D;
    int integral_window = 16;

    ManualBalancePID(const std::atomic<float>& p, const std::atomic<float>& i, const std::atomic<float>& d) {
        // Initialize gains from global variables
        P = p.load();
        I = i.load();
        D = d.load();
        error_buffer.fill(0.0f);
    }

    float compute(float error) {
        // Proportional term
        float p_term = P * error;

        // Integral term (sum over the window)
        float integral_sum = 0.0f;
        // Store current error in circular buffer before calculating sum
        error_buffer[buffer_index] = error;
        for (int i = 0; i < integral_window; ++i) {
            int index = (buffer_index - i + BUFFER_SIZE) % BUFFER_SIZE;
            integral_sum += error_buffer[index];
        }
        float i_term = I * integral_sum/integral_window;

        // Derivative term
        float error_change = error - previous_error;
        float d_term = D * error_change;

        // Update for next iteration
        previous_error = error;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;

        return p_term + i_term + d_term;
    }

private:
    static const int BUFFER_SIZE = 50;
    std::array<float, BUFFER_SIZE> error_buffer;
    int buffer_index = 0;
    float previous_error = 0.0f;
};
// --- End of Custom PID Controller ---

// Setup the motor and driver
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, kv);
BLDCDriver6PWM driver =
    BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);

// make encoder for simplefoc
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI encoder(CH1_ENC_CS);

// calibrated sensor object from simplefoc
CalibratedSensor sensor = CalibratedSensor(encoder);

// IMU
Imu::Imu imu;

// Create an instance of our new manual PID controller
ManualBalancePID balance_pid_manual_theta(command_bal_p_gain_theta, command_bal_i_gain_theta, command_bal_d_gain_theta);
ManualBalancePID balance_pid_manual_theta_dot(command_bal_p_gain_theta_dot, command_bal_i_gain_theta_dot, command_bal_d_gain_theta_dot);

// global atomic variable for the motor stuff to be set by mqtt
std::atomic<float> last_commanded_target = 0;
std::atomic<uint> last_commanded_mode = 0;

std::atomic<float> command_vel_p_gain = 0.025;
std::atomic<float> command_vel_i_gain = 0.3;
std::atomic<float> command_vel_d_gain = 0.0;

std::atomic<float> zero_point = 0.4;
std::atomic<bool> enable_flag = true;

// make a separate thread for the OTA
TaskHandle_t loop_foc_task;
// make a separate thread for the MQTT publishing
TaskHandle_t mqtt_publish_task;
const int interval_ms = 500;
std::atomic<float> pitch_dot_global = 0.0f;
constexpr float THETA_DOT_LPF_TAU = 0.02f; // seconds, first-order LPF for theta dot
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

      doc["target"] = motor.target;
      float motor_speed = sensor.getVelocity(); // velocity from encoder
      doc["vel"] = motor_speed;
      doc["motor_speed"] = motor_speed;

      // Report gains from the manual PID object
      doc["bal_p_theta"] = balance_pid_manual_theta.P;
      doc["bal_i_theta"] = balance_pid_manual_theta.I;
      doc["bal_d_theta"] = balance_pid_manual_theta.D;

      doc["bal_p_theta_dot"] = balance_pid_manual_theta_dot.P;
      doc["bal_i_theta_dot"] = balance_pid_manual_theta_dot.I;
      doc["bal_d_theta_dot"] = balance_pid_manual_theta_dot.D;

      doc["pitch_dot"] = pitch_dot_global.load();


      // doc["vel_p"] = motor.PID_velocity.P;
      // doc["vel_i"] = motor.PID_velocity.I;
      // doc["vel_d"] = motor.PID_velocity.D;
      
      Imu::RotationVector rot = imu.get_game_rotation();
      // doc["i"] = rot.i;  
      doc["j"] = rot.j;
      // doc["k"] = rot.k;

      doc["zero"] = zero_point.load();





      char buffer[512];
      serializeJson(doc, buffer, sizeof(buffer));
      publishMQTT(buffer);
    }

    vTaskDelay(interval_ms / portTICK_PERIOD_MS);
  }
}

void loop_foc_thread(void *pvParameters)
{
  bool motor_is_currently_enabled = true;
  float last_pitch = 0.0;
  float pitch_dot_filtered = 0.0f;
  unsigned long last_time = micros();

  // get initial pitch to avoid a huge derivative on the first loop
  Imu::RotationVector rot_init = imu.get_game_rotation();
  last_pitch = rot_init.j;

  while (1)
  {
    if (enable_flag.load()){
      if (!motor_is_currently_enabled) {
        motor.enable();
        motor_is_currently_enabled = true;
      }
      Imu::RotationVector rot = imu.get_game_rotation();
      float pitch = rot.j;

      unsigned long now = micros();
      float dt = (now - last_time) * 1e-6f;
      last_time = now;
      
      float pitch_dot = 0;
      if (dt > 0) {
        pitch_dot = (pitch - last_pitch) / dt;
      }
      last_pitch = pitch;
      float alpha = (dt > 0) ? dt / (THETA_DOT_LPF_TAU + dt) : 0.0f;
      pitch_dot_filtered += alpha * (pitch_dot - pitch_dot_filtered);
      pitch_dot_global.store(pitch_dot_filtered);

      float error_theta = pitch - zero_point.load();
      float output_theta = balance_pid_manual_theta.compute(error_theta);

      float error_theta_dot = pitch_dot_filtered;
      float output_theta_dot = balance_pid_manual_theta_dot.compute(error_theta_dot);

      float target_velocity = output_theta + output_theta_dot;


      last_commanded_target.store(target_velocity);
      motor.loopFOC();
      motor.move(target_velocity);

      imu.loop();
    } else {
      if (motor_is_currently_enabled) {
        motor.disable();
        motor_is_currently_enabled = false;
      }
    }
    vTaskDelay(portTICK_PERIOD_MS);
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);


  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");
  Serial.print("Version: ");
  Serial.println(VERSION);

  setupWiFi();

  ArduinoOTA.setHostname(NAME);
  ArduinoOTA.onStart(
      []()
      {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch";
        else type = "filesystem";
        Serial.println("Start updating " + type);
        enable_flag.store(false); // Stop motors on OTA
        delay(100);
      });
  ArduinoOTA.onEnd([]() { Serial.println("\nEnd OTA Update"); });
  ArduinoOTA.onProgress(
      [](unsigned int progress, unsigned int total)
      { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError(
      [](ota_error_t error)
      {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });
  ArduinoOTA.begin();
  
  imu.init();

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  delay(1000);
  encoder.init(&hspi);


  driver.voltage_power_supply = 12;
  // driver.voltage_limit = 12;
  driver.init();

  motor.linkDriver(&driver);
  motor.current_limit = 100;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
;

  motor.PID_velocity.P = 0.03;
  motor.PID_velocity.I = 0.6;
  motor.PID_velocity.D = 0.00001;
  motor.PID_velocity.output_ramp = 100000;
  // motor.PID_velocity.limit = 12;
  motor.LPF_velocity.Tf = 0.0001;

  motor.init();

  motor.linkSensor(&sensor);


  setupMQTT();
  Serial.println("MQTT very done");

  xTaskCreatePinnedToCore(mqtt_publish_thread, "MQTT_Publish", 10000, NULL, 1,
                          &mqtt_publish_task, 1);



  if (!motor.initFOC()){
    Serial.println("FOC init failed");
  }
  else {
    digitalWrite(LED_STATUS, HIGH);
    delay(1000);

      // MAKE IT SO IT zeroes at this point after this pause
    Imu::RotationVector rot = imu.get_game_rotation();
    zero_point.store(rot.j);
    Serial.print("Zero point set to: ");
    Serial.println(zero_point.load());


    xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 1,
                &loop_foc_task, 1);
    motor.enable();

    enable_flag.store(true);
  }


  Serial.println("FOC done");

 

  Serial.println("Setup complete.");
}


void loop()
{
  if ((command_bal_p_gain_theta.load() != balance_pid_manual_theta.P) ||
      (command_bal_i_gain_theta.load() != balance_pid_manual_theta.I) ||
      (command_bal_d_gain_theta.load() != balance_pid_manual_theta.D))
  {
    enable_flag.store(false);
    delay(10);
    balance_pid_manual_theta.P = command_bal_p_gain_theta.load();
    balance_pid_manual_theta.I = command_bal_i_gain_theta.load();
    balance_pid_manual_theta.D = command_bal_d_gain_theta.load();
    
    enable_flag.store(true);
  }

  if ((command_bal_p_gain_theta_dot.load() != balance_pid_manual_theta_dot.P) ||
      (command_bal_i_gain_theta_dot.load() != balance_pid_manual_theta_dot.I) ||
      (command_bal_d_gain_theta_dot.load() != balance_pid_manual_theta_dot.D))
  {
    enable_flag.store(false);
    delay(10);
    balance_pid_manual_theta_dot.P = command_bal_p_gain_theta_dot.load();
    balance_pid_manual_theta_dot.I = command_bal_i_gain_theta_dot.load();
    balance_pid_manual_theta_dot.D = command_bal_d_gain_theta_dot.load();
    
    enable_flag.store(true);
  }

  if ((command_vel_p_gain.load() != motor.PID_velocity.P) ||
      (command_vel_i_gain.load() != motor.PID_velocity.I) ||
      (command_vel_d_gain.load() != motor.PID_velocity.D))
    {
        enable_flag.store(false);
        delay(10);
        motor.PID_velocity.P = command_vel_p_gain.load();
        motor.PID_velocity.I = command_vel_i_gain.load();
        motor.PID_velocity.D = command_vel_d_gain.load();

        enable_flag.store(true);
    }


  ArduinoOTA.handle();

  vTaskDelay(10 / portTICK_PERIOD_MS);
}
