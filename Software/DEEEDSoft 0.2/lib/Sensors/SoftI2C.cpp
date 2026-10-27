#include "SoftI2C.h"

// Wie lange auf eine SCL-Leitung gewartet wird, die ein Sensor festhaelt
// ("clock stretching"). Der BMP280 macht das im Normalbetrieb gar nicht -
// haengt SCL laenger, stimmt etwas nicht. Gezaehlt wird in Durchlaeufen
// statt ueber micros(), damit die Grenze nicht von einem Timer abhaengt.
// 1000 Durchlaeufe a ca. 1 us = rund 1 ms.
static const uint16_t SCL_WAIT_LOOPS = 1000;

SoftI2C::SoftI2C(uint8_t sdaPin, uint8_t sclPin)
    : sda(sdaPin), scl(sclPin) {}

void SoftI2C::begin() {
    release(sda);
    release(scl);
}

void SoftI2C::driveLow() {
    pullLow(sda);
    pullLow(scl);
}

void SoftI2C::release(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

void SoftI2C::pullLow(uint8_t pin) {
    // Erst den Ausgangswert auf LOW setzen, dann auf Ausgang schalten -
    // so gibt es keinen kurzen HIGH-Puls beim Umschalten.
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

void SoftI2C::halfBit() {
    // Ergibt zusammen mit der Laufzeit der Pin-Funktionen rund 100 kHz.
    delayMicroseconds(2);
}

bool SoftI2C::releaseScl() {
    release(scl);
    for (uint16_t i = 0; i < SCL_WAIT_LOOPS; i++) {
        if (digitalRead(scl) == HIGH) {
            return true;
        }
        delayMicroseconds(1);
    }
    return false; // Sensor haelt SCL fest
}

bool SoftI2C::busIdle() {
    release(sda);
    release(scl);
    delayMicroseconds(5);
    return digitalRead(sda) == HIGH && digitalRead(scl) == HIGH;
}

void SoftI2C::recover() {
    release(sda);
    for (uint8_t i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
        pullLow(scl);
        halfBit();
        releaseScl();
        halfBit();
    }
    stop();
}

bool SoftI2C::start() {
    // Funktioniert als normaler START und als "repeated START" mitten in
    // einer Uebertragung: SDA hoch, SCL hoch, dann SDA fallen lassen.
    release(sda);
    halfBit();
    if (!releaseScl()) {
        return false;
    }
    halfBit();
    if (digitalRead(sda) == LOW) {
        return false; // jemand anderes haelt SDA fest
    }
    pullLow(sda);
    halfBit();
    pullLow(scl);
    halfBit();
    return true;
}

void SoftI2C::stop() {
    pullLow(sda);
    halfBit();
    releaseScl(); // auch bei Zeitueberschreitung weitermachen
    halfBit();
    release(sda);
    halfBit();
}

bool SoftI2C::writeByte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x80) {
            release(sda);
        } else {
            pullLow(sda);
        }
        halfBit();
        if (!releaseScl()) {
            return false;
        }
        halfBit();
        pullLow(scl);
        b <<= 1;
    }

    // ACK vom Sensor lesen: er zieht SDA auf LOW, wenn er das Byte hat.
    release(sda);
    halfBit();
    if (!releaseScl()) {
        return false;
    }
    halfBit();
    bool ack = (digitalRead(sda) == LOW);
    pullLow(scl);
    halfBit();
    return ack;
}

bool SoftI2C::readByte(uint8_t& out, bool ack) {
    release(sda);
    out = 0;
    for (uint8_t i = 0; i < 8; i++) {
        halfBit();
        if (!releaseScl()) {
            return false;
        }
        halfBit();
        out = (uint8_t)((out << 1) | (digitalRead(sda) == HIGH ? 1 : 0));
        pullLow(scl);
    }

    // ACK = "weiter senden", NACK beim letzten Byte = "fertig".
    if (ack) {
        pullLow(sda);
    } else {
        release(sda);
    }
    halfBit();
    if (!releaseScl()) {
        return false;
    }
    halfBit();
    pullLow(scl);
    release(sda);
    halfBit();
    return true;
}

bool SoftI2C::writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!start()) {
        stop();
        return false;
    }
    bool ok = writeByte((uint8_t)(addr << 1)) &&
              writeByte(reg) &&
              writeByte(value);
    stop();
    return ok;
}

bool SoftI2C::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    if (!start()) {
        stop();
        return false;
    }
    bool ok = writeByte((uint8_t)(addr << 1)) &&
              writeByte(reg) &&
              start() &&
              writeByte((uint8_t)((addr << 1) | 1));

    for (uint8_t i = 0; ok && i < len; i++) {
        ok = readByte(buf[i], i < len - 1);
    }
    stop();
    return ok;
}