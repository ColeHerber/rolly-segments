
#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>
#include <Arduino.h>

namespace Imu
{
    class Imu
    {
    public:
        Imu();
        void init();
        void loop();

        float get_roll();
        float get_pitch();
        float get_yaw();

        float get_accel_x();
        float get_accel_y();
        float get_accel_z();

        float get_gyro_x();
        float get_gyro_y();
        float get_gyro_z();

    private:
        Adafruit_BNO08x bno08x;
        sh2_SensorValue_t sensorValue;

        void setReports();
    };
} // namespace Imu

#endif // IMU_H