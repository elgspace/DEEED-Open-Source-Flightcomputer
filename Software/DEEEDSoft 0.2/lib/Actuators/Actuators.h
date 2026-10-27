#pragma once
#include <Arduino.h>

// Steuert die beiden MOSFET-Ausgaenge (Zuenddraht und Buzzer).
//
// WICHTIG: Alle Methoden sind NICHT-BLOCKIEREND. Es wird nirgends delay()
// benutzt, weil waehrend eines Fluges die Hauptschleife weiterlaufen muss:
// Sensoren abtasten, Zustandsuebergaenge erkennen, Watchdog fuettern.
// Ein delay() beim Apogaeum wuerde Flugdaten verschlucken und die
// Landungserkennung verzoegern. Stattdessen merkt sich die Klasse
// Startzeitpunkte und update() schaltet zum richtigen Zeitpunkt selbst ab.
class Actuators {
public:
    Actuators(uint8_t igniterPin, uint8_t buzzerPin);

    // Pins als Ausgang setzen und sicher auf LOW ziehen.
    // Muss frueh in setup() aufgerufen werden, damit ein undefinierter
    // Pin-Zustand beim Booten nicht versehentlich ausloest.
    void begin();

    // Zuenddraht fuer durationMs Millisekunden bestromen.
    // Der Aufruf kehrt sofort zurueck; das Abschalten uebernimmt update().
    //
    // EINMAL-SPERRE: Nach dem ersten Ausloesen wird der Kanal fuer den Rest
    // des Bootzyklus gesperrt. Ein Zuenddraht ist ein Einweg-Bauteil - er
    // brennt durch. Erneutes Bestromen kann nichts mehr bewirken, wuerde bei
    // einem noch intakten Draht aber unkontrolliert Strom ziehen (im
    // Kurzschlussfall bis zur Zerstoerung des MOSFETs oder Tiefentladung
    // des Akkus). Ein Sensorglitch, der die State Machine mehrfach durch
    // APOGEE laufen laesst, kann so keinen Schaden anrichten.
    void fireIgniter(uint32_t durationMs);

    // Dauerhaftes Piepen starten (Intervall an/aus in ms).
    void startBuzzer(uint32_t onMs = 500, uint32_t offMs = 500);

    void stopBuzzer();

    // Muss bei JEDEM Schleifendurchlauf aufgerufen werden - hier passiert
    // das zeitgesteuerte Abschalten des Zuenddrahts und das Takten des Buzzers.
    void update();

    bool isIgniterActive() const;

    // Wurde der Zuenddraht in diesem Bootzyklus bereits ausgeloest?
    bool hasIgniterFired() const;

    // Hebt die Einmal-Sperre auf - NUR fuer Bodentests gedacht, damit man
    // mehrere Testlaeufe hintereinander machen kann, ohne neu zu booten.
    void resetIgniterLock();

private:
    uint8_t igniterPin;
    uint8_t buzzerPin;

    bool igniterActive = false;
    bool igniterHasFired = false;
    uint32_t igniterStartedAt = 0;
    uint32_t igniterDurationMs = 0;

    bool buzzerEnabled = false;
    bool buzzerCurrentlyOn = false;
    uint32_t buzzerLastToggle = 0;
    uint32_t buzzerOnMs = 500;
    uint32_t buzzerOffMs = 500;
};