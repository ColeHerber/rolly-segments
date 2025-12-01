#include "imu.h"
#include "pins_arduino.h"
#include <math.h>
#include <Wire.h>

// You can change the I2C address here for trial and error
#define BNO08X_I2C_ADDR 0x4A

namespace Imu
{
    // Private member to hold the last read rotation vector
    RotationVector _last_rotation = {0, 0, 0};

    // Conversion helper modeled after Adafruit example quaternionToEuler
    static void quaternionToEuler(float qr, float qi, float qj, float qk, RotationVector *ypr, bool degrees = false)
    {
        const float sqr = sq(qr);
        const float sqi = sq(qi);
        const float sqj = sq(qj);
        const float sqk = sq(qk);

        ypr->k = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)); // yaw
        ypr->j = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)); // pitch
        ypr->i = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)); // roll

        if (degrees)
        {
            ypr->i *= RAD_TO_DEG;
            ypr->j *= RAD_TO_DEG;
            ypr->k *= RAD_TO_DEG;
        }
    }

    Imu::Imu() {}

    void Imu::init()
    {
        // Start I2C bus
        Wire.begin(SDA, SCL);
        delay(1000);
        if (!bno08x.begin_I2C(BNO08X_I2C_ADDR))
        {
            Serial.println("Failed to find BNO08x chip");
            while (1)
            {
                delay(10);
            }
        }
        Serial.println("BNO08x Found!");
        setReports();
    }

    void Imu::loop()
    {
        if (bno08x.wasReset())
        {
            Serial.print("sensor was reset ");
            setReports();
        }
    }

    void Imu::setReports()
    {
        Serial.println("Setting desired report to SH2_ROTATION_VECTOR");
        // Set the report to 400Hz (2500 us interval), which is a high-speed rate for this sensor.
        if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 2500))
        {
            Serial.println("Could not enable game rotation vector");
        }
    }

    RotationVector Imu::get_game_rotation()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
            {

                
            //    _last_rotation.real = sensorValue.un.rotationVector.real;
               
            //    _last_rotation.i = sensorValue.un.rotationVector.i;
            //     _last_rotation.j = sensorValue.un.rotationVector.j;
            //    _last_rotation.k = sensorValue.un.rotationVector.k;

                quaternionToEuler(
                    sensorValue.un.rotationVector.real,
                    sensorValue.un.rotationVector.i,
                    sensorValue.un.rotationVector.j,
                    sensorValue.un.rotationVector.k,
                    &_last_rotation,
                    false);
            }
        }
        // Return the last successfully read value
        return _last_rotation;
    }

} // namespace Imu
