#include "Actuators.h"

Actuators::Actuators(uint8_t igniterPin, uint8_t buzzerPin)
    : igniterPin(igniterPin), buzzerPin(buzzerPin) {}

void Actuators::begin() {
    pinMode(igniterPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(igniterPin, LOW);
    digitalWrite(buzzerPin, LOW);
}

void Actuators::fireIgniter(uint32_t durationMs) {
    if (igniterHasFired) {
        Serial.println("[ACT] Zuenddraht bereits ausgeloest - Anforderung ignoriert (Einmal-Sperre).");
        return;
    }

    igniterHasFired = true;
    igniterActive = true;
    igniterStartedAt = millis();
    igniterDurationMs = durationMs;
    digitalWrite(igniterPin, HIGH);

    Serial.print("[ACT] Zuenddraht AN fuer ");
    Serial.print(durationMs);
    Serial.println(" ms");
}

void Actuators::startBuzzer(uint32_t onMs, uint32_t offMs) {
    buzzerEnabled = true;
    buzzerOnMs = onMs;
    buzzerOffMs = offMs;
    buzzerCurrentlyOn = true;
    buzzerLastToggle = millis();
    digitalWrite(buzzerPin, HIGH);

    Serial.println("[ACT] Buzzer gestartet");
}

void Actuators::stopBuzzer() {
    buzzerEnabled = false;
    buzzerCurrentlyOn = false;
    digitalWrite(buzzerPin, LOW);
}

void Actuators::update() {
    uint32_t now = millis();

    // Zuenddraht nach Ablauf der Dauer selbsttaetig abschalten.
    // Das ist die wichtigste Aufgabe dieser Klasse: der Draht darf auf
    // keinen Fall dauerhaft bestromt bleiben.
    if (igniterActive && (now - igniterStartedAt >= igniterDurationMs)) {
        digitalWrite(igniterPin, LOW);
        igniterActive = false;
        Serial.println("[ACT] Zuenddraht AUS");
    }

    // Buzzer takten (an/aus im eingestellten Rhythmus)
    if (buzzerEnabled) {
        uint32_t interval = buzzerCurrentlyOn ? buzzerOnMs : buzzerOffMs;
        if (now - buzzerLastToggle >= interval) {
            buzzerCurrentlyOn = !buzzerCurrentlyOn;
            digitalWrite(buzzerPin, buzzerCurrentlyOn ? HIGH : LOW);
            buzzerLastToggle = now;
        }
    }
}

bool Actuators::isIgniterActive() const {
    return igniterActive;
}

bool Actuators::hasIgniterFired() const {
    return igniterHasFired;
}

void Actuators::resetIgniterLock() {
    igniterHasFired = false;
    Serial.println("[ACT] Einmal-Sperre des Zuenddrahts aufgehoben (Bodentest).");
}