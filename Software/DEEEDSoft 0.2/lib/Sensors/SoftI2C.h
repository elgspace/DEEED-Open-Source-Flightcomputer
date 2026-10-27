#pragma once
#include <Arduino.h>

// Minimaler I2C-Master in Software ("Bit-Banging") - mit Zeitlimit.
//
// Warum nicht die normale Wire-Bibliothek?
// Die Wire-Bibliothek des nRF52-Cores wartet in mehreren Schleifen ohne
// jede Zeitbegrenzung auf die I2C-Hardware. Haelt ein Sensor eine Leitung
// fest (genau das hat der BMP280 beim Bodentest am 29.09. getan), bleibt
// der komplette Flight Computer stehen, bis der Watchdog neu startet.
//
// Hier wird jedes Bit von Hand erzeugt, und jedes Warten auf die
// SCL-Leitung ist begrenzt. Ein haengender Sensor fuehrt deshalb nur zu
// einem "false" als Rueckgabe - die Hauptschleife laeuft weiter.
//
// Die Leitungen werden wie bei echtem I2C nie aktiv auf HIGH getrieben:
// "loslassen" heisst Eingang mit Pull-Up, "LOW" heisst Ausgang auf LOW.
class SoftI2C {
public:
    SoftI2C(uint8_t sdaPin, uint8_t sclPin);

    // Beide Leitungen loslassen (Ruhezustand).
    void begin();

    // Beide Leitungen aktiv auf LOW ziehen. Wird beim Abschalten des
    // Sensors gebraucht: sonst wuerde er ueber die Pull-Up-Widerstaende an
    // SDA/SCL weiter mit Strom versorgt und nicht wirklich ausgehen.
    void driveLow();

    // true, wenn SDA und SCL beide HIGH sind (Bus frei).
    bool busIdle();

    // Bis zu 9 Taktimpulse + STOP, um einen Sensor zu befreien, der mitten
    // in einer Uebertragung stehengeblieben ist und SDA festhaelt.
    void recover();

    bool writeReg(uint8_t addr, uint8_t reg, uint8_t value);
    bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);

private:
    uint8_t sda;
    uint8_t scl;

    void release(uint8_t pin);
    void pullLow(uint8_t pin);
    bool releaseScl();   // SCL loslassen und warten, bis sie HIGH ist
    void halfBit();

    bool start();
    void stop();
    bool writeByte(uint8_t b);
    bool readByte(uint8_t& out, bool ack);
};