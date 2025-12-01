#include "imu.h"
#include "pins_arduino.h"
#include <Wire.h>

// You can change the I2C address here for trial and error
#define BNO08X_I2C_ADDR 0x4A
// #define BNO08X_I2C_ADDR_2 0x4B

namespace Imu
{

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
        Serial.println("Setting desired reports");
        if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
        {
            Serial.println("Could not enable rotation vector");
        }
        if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
        {
            Serial.println("Could not enable linear acceleration");
        }
        if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
        {
            Serial.println("Could not enable gyroscope");
        }
    }

    float Imu::get_roll()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
            {
                return sensorValue.un.rotationVector.k;
            }
        }
        return 0;
    }

    float Imu::get_pitch()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
            {
                return sensorValue.un.rotationVector.j;
            }
        }
        return 0;
    }

    float Imu::get_yaw()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
            {
                return sensorValue.un.rotationVector.i;
            }
        }
        return 0;
    }

    float Imu::get_accel_x()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
            {
                return sensorValue.un.linearAcceleration.x;
            }
        }
        return 0;
    }

    float Imu::get_accel_y()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
            {
                return sensorValue.un.linearAcceleration.y;
            }
        }
        return 0;
    }

    float Imu::get_accel_z()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
            {
                return sensorValue.un.linearAcceleration.z;
            }
        }
        return 0;
    }

    float Imu::get_gyro_x()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                return sensorValue.un.gyroscope.x;
            }
        }
        return 0;
    }

    float Imu::get_gyro_y()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                return sensorValue.un.gyroscope.y;
            }
        }
        return 0;
    }

    float Imu::get_gyro_z()
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                return sensorValue.un.gyroscope.z;
            }
        }
        return 0;
    }

} // namespace Imu