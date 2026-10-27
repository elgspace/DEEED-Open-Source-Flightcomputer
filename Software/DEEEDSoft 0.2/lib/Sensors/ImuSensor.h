#pragma once
#include <Arduino.h>
#include "LSM6DS3.h"
 
// Ein IMU-Messwert im RAKETEN-Koordinatensystem
// (nicht im rohen Chip-Koordinatensystem - Achsen sind wegen des
// seitlichen Einbaus des XIAO bereits entsprechend zugeordnet).
struct ImuReading {
    float accelX, accelY, accelZ; // in g
    float gyroX, gyroY, gyroZ;    // in Grad/Sekunde
};
 
class ImuSensor {
public:
    explicit ImuSensor(uint8_t i2cAddress = 0x6A);
 
    // Initialisiert den Sensor inkl. erhöhter Abtastrate.
    // Rueckgabe: true bei Erfolg.
    bool begin();
 
    // Liest aktuelle Werte aus und liefert sie bereits im
    // Raketen-Koordinatensystem (Y/Z vertauscht wegen Einbaulage).
    ImuReading read();
 
private:
    LSM6DS3 imu;
};