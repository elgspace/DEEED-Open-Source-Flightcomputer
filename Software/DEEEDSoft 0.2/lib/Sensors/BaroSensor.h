#pragma once
#include <Arduino.h>
#include "SoftI2C.h"

// Groesse des Medianfilters auf der Hoehe.
// 5 Werte entfernen Einzelausreisser zuverlaessig - ein einzelner Ausreisser
// kann den Median einer 5er-Gruppe nicht verschieben - verzoegern echte
// Trends aber nur um 2 Samples, also 20 ms bei 100 Hz.
static const uint8_t ALT_MEDIAN_WINDOW = 5;

// Fenster fuer die Geschwindigkeitsschaetzung, in Samples.
// Bei 100 Hz entsprechen 20 Samples 200 ms: lang genug, dass das
// Sensorrauschen (ca. +/- 0,5 m) die Schaetzung nicht dominiert, kurz genug,
// um den Uebergang zum Sinkflug nicht zu verschlafen.
static const uint8_t VSPEED_WINDOW = 20;

struct BaroReading {
    float temperatureC;
    float pressureHpa;
    float altitudeM;        // absolute Hoehe ueber Meeresspiegel (wetterabhaengig ungenau)
    float altitudeAglRawM;  // Hoehe ueber Startplatz, UNGEFILTERT (roher Sensorwert)
    float altitudeAglM;     // Hoehe ueber Startplatz, medianfiltriert -
                            // hierauf arbeitet die Flugerkennung
    float verticalSpeedMs;  // positiv = steigend, negativ = sinkend
};

class BaroSensor {
public:
    // powerPin: Pin, der den BMP280 mit Strom versorgt (VCC des Moduls haengt
    // direkt daran). Damit laesst sich der Sensor per Software komplett
    // aus- und wieder einschalten - ein Reset des nRF52 allein erreicht ihn
    // nicht, weil er sonst dauerhaft an 3,3 V haengt.
    BaroSensor(uint8_t i2cAddress, float seaLevelHpa,
               uint8_t sdaPin, uint8_t sclPin, uint8_t powerPin);

    // Schaltet den Sensor aus und wieder ein und initialisiert ihn.
    // Blockiert ca. 70 ms - nur in setup() verwenden.
    // Rueckgabe: true, wenn der Sensor antwortet und eingerichtet ist.
    bool begin();

    // Misst den aktuellen Standort als Null-Referenz ein (Rakete muss dabei
    // ruhig auf der Startrampe stehen). Mittelt ueber mehrere Messungen, um
    // Sensorrauschen herauszufiltern.
    // Damit wird die wetterbedingte Ungenauigkeit des absoluten Hoehenwerts
    // irrelevant: es zaehlt nur noch die Aenderung gegenueber dem Startplatz.
    // Rueckgabe: false, wenn keine einzige Messung geklappt hat.
    bool calibrateGround(uint16_t samples = 50, uint16_t delayMs = 20);

    // Absolute Hoehe des Startplatzes, wie bei calibrateGround() gemessen
    float getGroundAltitude() const;

    // Liest den Sensor aus und aktualisiert dabei Medianfilter und
    // Geschwindigkeitsschaetzung. Sollte in gleichmaessigen Abstaenden
    // aufgerufen werden.
    //
    // Blockiert NIE laenger als wenige Millisekunden. Antwortet der Sensor
    // nicht, wird der letzte gueltige Wert zurueckgegeben (die Hoehe
    // "friert ein") und der Sensor im Hintergrund neu gestartet.
    BaroReading read();

    // false, solange der Sensor gerade nicht antwortet oder neu startet.
    bool isHealthy() const;

    // Zaehler fuer die Diagnose (Befehl 's' im Monitor).
    uint32_t getErrorCount() const;
    uint16_t getRestartCount() const;

private:
    SoftI2C bus;
    uint8_t address;
    float seaLevelHpa;
    uint8_t powerPin;
    float groundAltitudeM = 0.0f;

    // --- Kalibrierwerte aus dem Sensor (Datenblatt Kap. 3.11.2) ---
    uint16_t digT1 = 0;
    int16_t digT2 = 0, digT3 = 0;
    uint16_t digP1 = 0;
    int16_t digP2 = 0, digP3 = 0, digP4 = 0, digP5 = 0,
            digP6 = 0, digP7 = 0, digP8 = 0, digP9 = 0;

    // --- Zustand fuer den Neustart im laufenden Betrieb ---
    enum class Health : uint8_t { OK, POWERED_OFF, SETTLING };
    Health health = Health::OK;
    uint32_t healthSince = 0;
    uint8_t consecutiveErrors = 0;
    uint32_t errorCount = 0;
    uint16_t restartCount = 0;
    BaroReading lastReading = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    void powerOff();
    void powerOn();

    // Chip-ID pruefen, Soft-Reset, Kalibrierwerte lesen, Messmodus setzen.
    bool configure();

    // Einen Satz Rohwerte lesen und in Temperatur (Grad C) und Druck (Pa)
    // umrechnen. false, wenn der Sensor nicht antwortet oder noch keine
    // gueltige Messung hat.
    bool measure(float& temperatureC, float& pressurePa);

    // Leitet bei Fehlern den Neustart ein bzw. fuehrt ihn Schritt fuer
    // Schritt weiter, ohne die Hauptschleife aufzuhalten.
    void handleError();
    void serviceRestart();

public:
    // Umrechnung nach Bosch-Datenblatt. Oeffentlich und statisch, damit sie
    // sich am PC mit den Beispielwerten aus dem Datenblatt pruefen laesst.
    static int32_t compensateTemperature(int32_t adcT, uint16_t t1, int16_t t2,
                                         int16_t t3, int32_t& tFine);
    static uint32_t compensatePressure(int32_t adcP, int32_t tFine,
                                       uint16_t p1, int16_t p2, int16_t p3,
                                       int16_t p4, int16_t p5, int16_t p6,
                                       int16_t p7, int16_t p8, int16_t p9);

private:
    // Rechnet Druck in Hoehe um (internationale Hoehenformel).
    float pressureToAltitude(float pressureHpa) const;

    // --- Medianfilter auf der Hoehe ---
    float medianBuf[ALT_MEDIAN_WINDOW];
    uint8_t medianCount = 0;   // wie viele Werte bereits gueltig sind
    uint8_t medianIndex = 0;   // naechste Schreibposition (Ringpuffer)
    float medianOf(float newValue);

    // --- Geschwindigkeitsschaetzung ---
    // Ringpuffer aus gefilterten Hoehen samt Zeitstempel. Die Geschwindigkeit
    // ist die Steigung zwischen dem aeltesten und dem neuesten Eintrag.
    float speedAlt[VSPEED_WINDOW];
    uint32_t speedTime[VSPEED_WINDOW];
    uint8_t speedCount = 0;
    uint8_t speedIndex = 0;
    float updateVerticalSpeed(float filteredAlt, uint32_t nowMs);
};