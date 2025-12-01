
#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>
#include <Arduino.h>

namespace Imu
{
    // Struct holding roll (i), pitch (j), yaw (k) in radians derived from the game rotation quaternion
    struct RotationVector {
        float i;
        float j;
        float k;
    };

    class Imu
    {
    public:
        Imu();
        void init();
        void loop();

        // Single function to get the game rotation vector
        RotationVector get_game_rotation();

    private:
        Adafruit_BNO08x bno08x;
        sh2_SensorValue_t sensorValue;

        void setReports();
    };
} // namespace Imu

#endif // IMU_H
