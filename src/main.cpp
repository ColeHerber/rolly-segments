// #include <FreeRTOS.h>
#include "pins_arduino.h" // Include our custom pins for AXIS board
#include <SimpleFOC.h>

#include "SimpleFOCDrivers.h"

#include <encoders/mt6701/MagneticSensorMT6701SSI.h>


// Setup sensors for the two channels
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);
MagneticSensorMT6701SSI encoder1(CH1_ENC_CS);


void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting setup...");
    delay(1000);

    // initialise encoder hardware
    encoder0.init();
    encoder1.init();

    delay(1000);

    Serial.println("Setup complete.");
}

void loop() {
  encoder0.update();
  encoder1.update();

  // display the angle and the angular velocity to the terminal
  Serial.print(encoder0.getSensorAngle());
  Serial.print("hello");

  Serial.print(encoder1.getSensorAngle());


  Serial.print("\t");
  delay(10);

    
}