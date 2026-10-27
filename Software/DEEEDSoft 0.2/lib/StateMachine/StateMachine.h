#pragma once
#include <Arduino.h>
#include "ImuSensor.h"
#include "BaroSensor.h"

// Flugphasen
enum class FlightState {
    PAD,
    BOOST,
    COAST,
    APOGEE,
    DESCENT,
    LANDED
};

// Warum das Apogaeum ausgeloest wurde - wird nach dem Flug ausgegeben und
// hilft bei der Auswertung zu erkennen, ob die normale Erkennung gegriffen
// hat oder nur das Sicherheitsnetz.
enum class ApogeeTrigger {
    NONE,
    BAROMETRIC,   // Hoehe faellt und Sinkrate ist plausibel - der Normalfall
    BACKUP_TIMER, // Erkennung hat nicht gegriffen, Zeitlimit abgelaufen
    MANUAL        // Override-Pin im Bodentest
};

class StateMachine {
public:
    StateMachine();

    FlightState getState() const;

    // Wechselt den Zustand und loggt den Uebergang automatisch ueber Serial.
    void transitionTo(FlightState newState);

    // Wie lange befinden wir uns schon im aktuellen Zustand (ms)
    uint32_t timeInState() const;

    static const char* stateName(FlightState state);
    static const char* apogeeTriggerName(ApogeeTrigger trigger);

    // Automatische Erkennung anhand aktueller Sensordaten.
    // Bei jedem Sample-Durchlauf aufrufen.
    void update(const ImuReading& imuData, const BaroReading& baroData);

    // Erzwingt den Wechsel in den naechsten Zustand, unabhaengig von
    // Sensorbedingungen. Fuer Bodentests (z.B. per Pin-auf-GND).
    void forceNextState();

    ApogeeTrigger getApogeeTrigger() const;

private:
    FlightState currentState;
    uint32_t stateEnteredAt;

    ApogeeTrigger apogeeTrigger = ApogeeTrigger::NONE;

    // --- interner Zustand fuer die Erkennungslogik ---
    uint32_t liftoffCounter = 0;
    uint32_t burnoutCounter = 0;

    float maxAltitudeSeen = -10000.0f;
    uint32_t apogeeCounter = 0;

    float lastAltitudeForLanding = 0.0f;
    uint32_t landingStableSince = 0;
    bool landingWindowStarted = false;

    // Landungserkennung ueber die IMU: seit wann ist die Rakete ruhig?
    uint32_t imuCalmSince = 0;
    bool imuCalmWindowStarted = false;
};