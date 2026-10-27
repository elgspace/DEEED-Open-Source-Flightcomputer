#pragma once
#include <Arduino.h>
#include "ImuSensor.h"
#include "BaroSensor.h"

// Ein einzelner Messpunkt, bewusst kompakt gehalten.
//
// Warum nicht einfach float fuer alles? Der nRF52840 hat 237 KB RAM. Waehrend
// des Fluges wird nicht auf die SD-Karte geschrieben (siehe DataLogger unten),
// sondern in den RAM gepuffert - also entscheidet die Groesse dieses Structs
// direkt darueber, wie viele Sekunden Flug aufgezeichnet werden koennen.
// Beschleunigung/Drehrate werden als int16 mit festem Faktor gespeichert
// (1/1000 g bzw. 1/10 dps) statt als float: halbiert den Platzbedarf, die
// Aufloesung ist immer noch feiner als das Sensorrauschen.
struct LogRecord {
    // Zeit RELATIV zur Starterkennung, in Einheiten von 10 ms.
    // Negativ = vor dem Start (aus dem Vorlaufpuffer).
    // int16 statt int32 spart 2 Byte pro Record und deckt +/- 327 Sekunden
    // ab - mehr, als der Puffer selbst bei der langsamsten Rate fassen kann.
    // Die 10-ms-Aufloesung ist feiner als das kuerzeste Logging-Intervall.
    int16_t timestampTicks;
    int16_t accelX;           // in 1/1000 g
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;            // in 1/10 dps
    int16_t gyroY;
    int16_t gyroZ;
    int16_t altitudeAglRawDm; // Hoehe ueber Startplatz, UNGEFILTERT, in Dezimetern
    int16_t altitudeAglDm;    // Hoehe ueber Startplatz, medianfiltriert, in Dezimetern
    int16_t verticalSpeedCms; // Vertikalgeschwindigkeit in cm/s (+/- 327 m/s Bereich)
    uint16_t pressureHpaX10;  // Luftdruck in 1/10 hPa
    int8_t temperatureC;      // Temperatur in ganzen Grad C. Die Temperatur
                              // dient nur der Plausibilitaet; ein Grad
                              // Aufloesung wirkt sich auf die barometrische
                              // Hoehe mit unter einem Meter aus.
    uint8_t state;            // FlightState als Zahl
};

// Wie viele Messpunkte in den RAM passen sollen.
//
// Auslegung mit den gestaffelten Logging-Raten (50 Hz bis zum Apogaeum,
// 5 Hz im Sinkflug): 150 Records Vorlauf + 10 s Aufstieg (500) laesst rund
// 1350 Records fuer den Abstieg - bei 5 Hz also 270 Sekunden. Der
// Sinkflug-Timeout in StateMachine.cpp beendet den Flug ohnehin nach
// 240 s, der Puffer reicht also fuer den kompletten Flug.
//
// 2000 Records * 24 Byte = 47 KB - derselbe Wert wie beim zweiten Flug.
// 4000 ist zurueckgenommen, weil der Flight Computer damit beim Bodentest
// nicht mehr zuverlaessig gestartet ist.
static const uint16_t LOG_BUFFER_CAPACITY = 2000;

// Vorlaufpuffer ("pre-trigger"): Die Starterkennung braucht einige Samples,
// um sicher auszuloesen - dieser Moment waere sonst fuer immer verloren.
// Deshalb wird auch auf der Rampe schon durchgehend in einen kleinen
// Ringpuffer geschrieben, der beim erkannten Start vorne an die Aufzeichnung
// angehaengt wird. So ist der komplette Schubaufbau in den Daten.
// 150 Records bei 50 Hz = 3 Sekunden Vorlauf.
static const uint16_t PRE_BUFFER_CAPACITY = 150;

// Kennzahlen, die nach dem Flug aus dem Puffer berechnet werden.
struct FlightSummary {
    bool valid = false;
    uint16_t records = 0;

    float maxAltitudeM = 0.0f;
    int32_t maxAltitudeTimeMs = 0;

    float maxAccelG = 0.0f;       // groesster Betrag entlang der Laengsachse
    int32_t maxAccelTimeMs = 0;

    float maxSpeedUpMs = 0.0f;    // hoechste gemessene Steiggeschwindigkeit

    int32_t burnoutTimeMs = -1;   // erster Record in COAST (-1 = nie erreicht)
    int32_t apogeeTimeMs = -1;    // erster Record in APOGEE/DESCENT
    int32_t lastTimeMs = 0;

    float meanDescentMs = 0.0f;   // mittlere Sinkrate ab Apogaeum
};

class DataLogger {
public:
    explicit DataLogger(uint8_t chipSelectPin);

    // SD-Karte initialisieren. Rueckgabe: true bei Erfolg.
    // Schlaegt das fehl, laeuft der Flug trotzdem weiter - Logging ist
    // wuenschenswert, aber nicht flugkritisch.
    bool begin();

    bool isCardAvailable() const;

    // Starterkennung melden: ab hier ist t=0. Der Vorlaufpuffer wird
    // uebernommen, seine Eintraege bekommen negative Zeitstempel.
    void markLaunch(uint32_t launchMillis);

    // Aufzeichnung beenden (z.B. bei der Landung).
    void stopRecording();

    bool hasLaunched() const;

    // Einen Messpunkt in den RAM-Puffer schreiben.
    // Bewusst NICHT auf die SD-Karte: ein SD-Schreibvorgang kann
    // gelegentlich ueber 100 ms blockieren (interne Verwaltung der Karte).
    // Waehrend Aufstieg und Apogaeum darf die Hauptschleife aber nicht
    // stehenbleiben, sonst verpasst die State Machine den Umkehrpunkt.
    void record(uint32_t timestampMs,
                const ImuReading& imu,
                const BaroReading& baro,
                uint8_t state);

    // Den kompletten Puffer als CSV-Datei auf die Karte schreiben.
    // Wird nach der Landung aufgerufen, wenn Zeit unkritisch ist.
    // Legt fortlaufend nummerierte Dateien an (flight001.csv, flight002.csv, ...),
    // damit ein neuer Flug einen alten nicht ueberschreibt.
    // Schreibt zusaetzlich flightNNN.txt mit den Flugkennzahlen.
    bool flushToCard();

    // Kennzahlen aus dem Puffer berechnen (ohne SD-Karte moeglich).
    FlightSummary computeSummary() const;

    // Kennzahlen ueber Serial ausgeben.
    void printSummary(const FlightSummary& s) const;

    // Gibt die zuletzt geschriebene Datei ueber Serial aus - zum Auslesen
    // der Flugdaten, ohne die Karte aus dem Geraet nehmen zu muessen.
    bool dumpFileToSerial(const char* filename);

    // Listet alle Flugdateien auf der Karte auf.
    void listFiles();

    uint16_t getRecordCount() const;
    bool hasOverflowed() const;
    const char* getLastFilename() const;

private:
    uint8_t csPin;
    bool cardAvailable = false;

    // Gibt Dateisystemtyp und Kartengroesse aus (Diagnose).
    void printCardInfo();

    LogRecord buffer[LOG_BUFFER_CAPACITY];
    uint16_t recordCount = 0;
    bool overflowed = false;

    // Ringpuffer fuer die Zeit vor der Starterkennung
    LogRecord preBuffer[PRE_BUFFER_CAPACITY];
    uint16_t preIndex = 0;     // naechste Schreibposition
    uint16_t preCount = 0;     // wie viele Eintraege gueltig sind

    // Absolute millis()-Werte der Vorlauf-Records. Getrennt gehalten, weil
    // der Startzeitpunkt beim Schreiben noch nicht bekannt ist und ein
    // absoluter Zeitstempel nicht in das int16-Feld des Records passt.
    // Kostet nur 150 * 4 Byte und macht die Umrechnung eindeutig.
    uint32_t preAbsTimeMs[PRE_BUFFER_CAPACITY];

    bool launched = false;
    bool recordingStopped = false;
    uint32_t launchMillis = 0;

    // Fuellt einen Record aus den Sensordaten (ohne Zeitstempel).
    void fillRecord(LogRecord& r,
                    const ImuReading& imu,
                    const BaroReading& baro,
                    uint8_t state);

    // Zeitstempel eines Records in Millisekunden relativ zum Start.
    int32_t recordTime(uint16_t index) const;

    // Rechnet eine Millisekunden-Differenz in das int16-Tickformat um und
    // begrenzt sie auf den darstellbaren Bereich.
    static int16_t toTicks(int32_t deltaMs);

    // Schreibt die Kennzahlen in eine eigene .txt-Datei neben der CSV.
    bool writeSummaryFile(const char* csvName, const FlightSummary& s);

    char lastFilename[16] = {0};

    bool makeNextFilename(char* out, size_t outSize);
};