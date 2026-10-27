#include "ImuSensor.h"

ImuSensor::ImuSensor(uint8_t i2cAddress)
    : imu(I2C_MODE, i2cAddress) {}

bool ImuSensor::begin() {
    // Erhöhte Abtastrate fuer schnellere Updates (siehe vorherige Optimierung)
    imu.settings.accelSampleRate = 416;
    imu.settings.gyroSampleRate = 416;

    return imu.begin() == 0;
}

ImuReading ImuSensor::read() {
    ImuReading r;

    // Achsentausch Y<->Z: XIAO ist seitlich statt liegend in der Rakete verbaut
    r.accelX = imu.readFloatAccelX();
    r.accelY = imu.readFloatAccelZ();
    r.accelZ = imu.readFloatAccelY();

    r.gyroX = imu.readFloatGyroX();
    r.gyroY = imu.readFloatGyroZ();
    r.gyroZ = imu.readFloatGyroY();

    return r;
}