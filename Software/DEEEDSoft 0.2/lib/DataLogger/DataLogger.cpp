#include "DataLogger.h"
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SleepyDog.h>

// SdFs statt SdFat: unterstuetzt FAT16, FAT32 UND exFAT.
// Karten ab 64 GB sind ab Werk exFAT formatiert und wuerden von der
// reinen SdFat-Klasse gar nicht erkannt - ein sehr haeufiger Stolperstein.
static SdFs sd;

DataLogger::DataLogger(uint8_t chipSelectPin)
    : csPin(chipSelectPin) {}

bool DataLogger::begin() {
    // Mehrere SPI-Geschwindigkeiten durchprobieren, von schnell nach langsam.
    // Bei fliegender Verdrahtung, laengeren Leitungen oder manchen Karten
    // funktioniert nur eine niedrige Taktrate. Fuer das Schreiben nach der
    // Landung ist Tempo ohnehin unkritisch - Hauptsache, es klappt ueberhaupt.
    const uint8_t speedsMhz[] = {16, 8, 4, 1};

    for (uint8_t i = 0; i < sizeof(speedsMhz); i++) {
        // Watchdog vor JEDEM Versuch fuettern: ein fehlschlagender
        // Initialisierungsversuch kann mehrere Sekunden dauern, und ohne
        // dieses Fuettern wuerde der Watchdog mitten in der Suche neu starten.
        Watchdog.reset();

        Serial.print("[SD] Versuche Initialisierung mit ");
        Serial.print(speedsMhz[i]);
        Serial.println(" MHz...");

        if (sd.begin(SdSpiConfig(csPin, SHARED_SPI, SD_SCK_MHZ(speedsMhz[i])))) {
            cardAvailable = true;
            Serial.print("[SD] Karte erfolgreich initialisiert bei ");
            Serial.print(speedsMhz[i]);
            Serial.println(" MHz.");
            printCardInfo();
            return true;
        }
    }

    cardAvailable = false;
    Serial.println("[SD] Karte nicht gefunden - Logging deaktiviert, Flug laeuft weiter.");
    Serial.println("[SD] Fehlerdetails von SdFat:");
    sd.initErrorPrint(&Serial);
    Serial.println("[SD] Pruefen: Verkabelung (CS=D7, SCK=D8, MISO=D9, MOSI=D10),");
    Serial.println("[SD]          Karte eingesteckt, 3V und GND verbunden,");
    Serial.println("[SD]          Karte als FAT32 formatiert.");
    return false;
}

void DataLogger::printCardInfo() {
    // Kartentyp und Dateisystem ausgeben - hilft beim Einordnen, ob z.B.
    // eine exFAT-Karte im Spiel ist oder die Karte sehr gross/langsam ist.
    Serial.print("[SD] Dateisystem: ");
    switch (sd.fatType()) {
        case FAT_TYPE_FAT16: Serial.println("FAT16"); break;
        case FAT_TYPE_FAT32: Serial.println("FAT32"); break;
        case FAT_TYPE_EXFAT: Serial.println("exFAT"); break;
        default:             Serial.println("unbekannt"); break;
    }

    // Gesamtkapazitaet statt freiem Speicher anzeigen: freeClusterCount()
    // durchsucht bei FAT32 die komplette Zuordnungstabelle und kann auf
    // grossen Karten viele Sekunden dauern - lange genug, dass der Watchdog
    // zuschlaegt. card()->sectorCount() liest dagegen nur ein Register.
    uint32_t sizeMb = (uint32_t)(sd.card()->sectorCount() / 2048UL);
    Serial.print("[SD] Kartengroesse: ");
    Serial.print(sizeMb);
    Serial.println(" MB");
}

bool DataLogger::isCardAvailable() const {
    return cardAvailable;
}

void DataLogger::fillRecord(LogRecord& r,
                            const ImuReading& imu,
                            const BaroReading& baro,
                            uint8_t state) {
    r.accelX = (int16_t)(imu.accelX * 1000.0f);
    r.accelY = (int16_t)(imu.accelY * 1000.0f);
    r.accelZ = (int16_t)(imu.accelZ * 1000.0f);
    r.gyroX = (int16_t)(imu.gyroX * 10.0f);
    r.gyroY = (int16_t)(imu.gyroY * 10.0f);
    r.gyroZ = (int16_t)(imu.gyroZ * 10.0f);
    r.altitudeAglRawDm = (int16_t)(baro.altitudeAglRawM * 10.0f);
    r.altitudeAglDm = (int16_t)(baro.altitudeAglM * 10.0f);
    r.verticalSpeedCms = (int16_t)(baro.verticalSpeedMs * 100.0f);
    r.pressureHpaX10 = (uint16_t)(baro.pressureHpa * 10.0f);
    r.temperatureC = (int8_t)(baro.temperatureC + (baro.temperatureC >= 0 ? 0.5f : -0.5f));
    r.state = state;
}

int16_t DataLogger::toTicks(int32_t deltaMs) {
    int32_t ticks = deltaMs / 10;
    // Begrenzen, damit ein sehr langer Flug keinen stillen Ueberlauf
    // erzeugt. Der Puffer laeuft vorher voll, aber sicher ist sicher.
    if (ticks > 32767)  ticks = 32767;
    if (ticks < -32768) ticks = -32768;
    return (int16_t)ticks;
}

int32_t DataLogger::recordTime(uint16_t index) const {
    // Alle Records im Hauptpuffer tragen bereits die Zeit relativ zum Start -
    // die Vorlauf-Records werden beim Uebernehmen in markLaunch() umgerechnet.
    return (int32_t)buffer[index].timestampTicks * 10;
}

void DataLogger::markLaunch(uint32_t launchMs) {
    if (launched) {
        return;
    }
    launched = true;
    launchMillis = launchMs;

    // Vorlaufpuffer in chronologischer Reihenfolge in den Hauptpuffer
    // uebernehmen. Der Ringpuffer ist eventuell umgebrochen, deshalb wird
    // ab der aeltesten Position gelesen, nicht ab Index 0.
    uint16_t start = (preCount < PRE_BUFFER_CAPACITY) ? 0 : preIndex;

    for (uint16_t i = 0; i < preCount && recordCount < LOG_BUFFER_CAPACITY; i++) {
        uint16_t src = (start + i) % PRE_BUFFER_CAPACITY;
        buffer[recordCount] = preBuffer[src];

        // Jetzt ist der Startzeitpunkt bekannt - aus dem absoluten
        // millis()-Wert wird die (negative) Zeit relativ zum Start.
        buffer[recordCount].timestampTicks =
            toTicks((int32_t)preAbsTimeMs[src] - (int32_t)launchMs);

        recordCount++;
    }

    Serial.print("[LOG] Start erkannt - ");
    Serial.print(preCount);
    Serial.println(" Messpunkte aus dem Vorlaufpuffer uebernommen.");
}

void DataLogger::stopRecording() {
    recordingStopped = true;
}

bool DataLogger::hasLaunched() const {
    return launched;
}

void DataLogger::record(uint32_t timestampMs,
                        const ImuReading& imu,
                        const BaroReading& baro,
                        uint8_t state) {
    if (recordingStopped) {
        return;
    }

    // Vor der Starterkennung: in den Ringpuffer schreiben. Ist er voll,
    // werden die aeltesten Eintraege ueberschrieben - so steht immer der
    // juengste Vorlauf bereit, egal wie lange die Rakete auf der Rampe stand.
    if (!launched) {
        LogRecord& r = preBuffer[preIndex];
        fillRecord(r, imu, baro, state);
        // Zeitstempel vorerst absolut und separat merken - beim Start wird
        // in markLaunch() daraus die relative Zeit.
        r.timestampTicks = 0;
        preAbsTimeMs[preIndex] = timestampMs;

        preIndex = (preIndex + 1) % PRE_BUFFER_CAPACITY;
        if (preCount < PRE_BUFFER_CAPACITY) {
            preCount++;
        }
        return;
    }

    if (recordCount >= LOG_BUFFER_CAPACITY) {
        if (!overflowed) {
            overflowed = true;
            Serial.println("[LOG] Puffer voll - weitere Messpunkte werden verworfen.");
        }
        // Bewusst die AELTESTEN Daten behalten statt einen Ringpuffer zu
        // benutzen: Start, Brennphase und Apogaeum liegen am Anfang des Fluges
        // und sind fuer die Auswertung am interessantesten. Eine lange
        // Fallschirmphase am Ende darf abgeschnitten werden.
        return;
    }

    LogRecord& r = buffer[recordCount];
    fillRecord(r, imu, baro, state);
    r.timestampTicks = toTicks((int32_t)timestampMs - (int32_t)launchMillis);
    recordCount++;
}

bool DataLogger::makeNextFilename(char* out, size_t outSize) {
    for (uint16_t i = 1; i < 1000; i++) {
        Watchdog.reset(); // viele Dateien auf der Karte = viele Abfragen
        snprintf(out, outSize, "flight%03u.csv", i);
        if (!sd.exists(out)) {
            return true;
        }
    }
    return false;
}

bool DataLogger::flushToCard() {
    if (!cardAvailable) {
        Serial.println("[SD] Keine Karte - kann nicht schreiben.");
        return false;
    }

    if (recordCount == 0) {
        Serial.println("[SD] Keine Daten zum Schreiben vorhanden.");
        return false;
    }

    char filename[16];
    if (!makeNextFilename(filename, sizeof(filename))) {
        Serial.println("[SD] Kein freier Dateiname gefunden.");
        return false;
    }

    FsFile file;
    if (!file.open(filename, O_WRONLY | O_CREAT | O_TRUNC)) {
        Serial.print("[SD] Datei konnte nicht angelegt werden: ");
        Serial.println(filename);
        return false;
    }

    Serial.print("[SD] Schreibe ");
    Serial.print(recordCount);
    Serial.print(" Messpunkte nach ");
    Serial.println(filename);

    // alt_raw_m ist der ungefilterte Sensorwert, alt_m der medianfiltrierte -
    // auf letzterem arbeitet die Flugerkennung. Beide stehen in der Datei,
    // damit sich beim Auswerten nachvollziehen laesst, was der Filter
    // entfernt hat.
    file.println(F("time_ms,state,accelX_g,accelY_g,accelZ_g,gyroX_dps,gyroY_dps,gyroZ_dps,alt_raw_m,alt_m,vspeed_ms,temp_C,press_hPa"));

    for (uint16_t i = 0; i < recordCount; i++) {
        const LogRecord& r = buffer[i];

        // Watchdog regelmaessig fuettern. Beim nRF52 laesst sich die
        // Watchdog-Zeit nach dem Start NICHT mehr aendern (siehe setup() in
        // main.cpp) - es bleibt also bei 4 s. 4000 Zeilen koennen laenger
        // dauern, und ein Reset mitten im Schreiben wuerde die Flugdaten im
        // RAM loeschen. Alle 64 Zeilen zuruecksetzen kostet praktisch nichts.
        if ((i & 0x3F) == 0) {
            Watchdog.reset();
        }

        // Zurueckrechnen in lesbare Einheiten, damit die CSV direkt in
        // Excel/Python ausgewertet werden kann, ohne Umrechnungsfaktoren.
        file.print(recordTime(i));        file.print(',');
        file.print(r.state);              file.print(',');
        file.print(r.accelX / 1000.0f, 3); file.print(',');
        file.print(r.accelY / 1000.0f, 3); file.print(',');
        file.print(r.accelZ / 1000.0f, 3); file.print(',');
        file.print(r.gyroX / 10.0f, 1);    file.print(',');
        file.print(r.gyroY / 10.0f, 1);    file.print(',');
        file.print(r.gyroZ / 10.0f, 1);    file.print(',');
        file.print(r.altitudeAglRawDm / 10.0f, 1); file.print(',');
        file.print(r.altitudeAglDm / 10.0f, 1);    file.print(',');
        file.print(r.verticalSpeedCms / 100.0f, 2); file.print(',');
        file.print((int)r.temperatureC);          file.print(',');
        file.println(r.pressureHpaX10 / 10.0f, 1);
    }

    file.sync();
    file.close();

    strncpy(lastFilename, filename, sizeof(lastFilename) - 1);
    lastFilename[sizeof(lastFilename) - 1] = '\0';

    Serial.print("[SD] Fertig geschrieben: ");
    Serial.println(filename);

    // Kennzahlen berechnen, ausgeben und in eine eigene Datei schreiben.
    // Bewusst NICHT als Kommentarzeilen in die CSV: dort wuerden sie beim
    // Import in Excel oder Pandas den Header verschieben.
    FlightSummary s = computeSummary();
    printSummary(s);
    writeSummaryFile(filename, s);

    return true;
}

FlightSummary DataLogger::computeSummary() const {
    FlightSummary s;

    if (recordCount == 0) {
        return s;
    }

    s.valid = true;
    s.records = recordCount;
    s.lastTimeMs = recordTime(recordCount - 1);

    int32_t apogeeTime = -1;
    float apogeeAlt = 0.0f;

    for (uint16_t i = 0; i < recordCount; i++) {
        const LogRecord& r = buffer[i];
        int32_t t = recordTime(i);

        float alt = r.altitudeAglDm / 10.0f;
        if (alt > s.maxAltitudeM) {
            s.maxAltitudeM = alt;
            s.maxAltitudeTimeMs = t;
        }

        float az = r.accelZ / 1000.0f;
        if (fabsf(az) > fabsf(s.maxAccelG)) {
            s.maxAccelG = az;
            s.maxAccelTimeMs = t;
        }

        float v = r.verticalSpeedCms / 100.0f;
        if (v > s.maxSpeedUpMs) {
            s.maxSpeedUpMs = v;
        }

        // Erster Record in COAST (Zustand 2) = Brennschluss
        if (s.burnoutTimeMs < 0 && r.state == 2) {
            s.burnoutTimeMs = t;
        }
        // Erster Record ab APOGEE (3) bzw. DESCENT (4). APOGEE dauert nur
        // einen Schleifendurchlauf und taucht in der Aufzeichnung oft gar
        // nicht auf, deshalb wird auch DESCENT akzeptiert.
        if (apogeeTime < 0 && r.state >= 3) {
            apogeeTime = t;
            apogeeAlt = alt;
        }
    }

    s.apogeeTimeMs = apogeeTime;

    // Mittlere Sinkrate ab dem Apogaeum bis zum letzten Messpunkt
    if (apogeeTime >= 0 && s.lastTimeMs > apogeeTime) {
        float endAlt = buffer[recordCount - 1].altitudeAglDm / 10.0f;
        float dt = (s.lastTimeMs - apogeeTime) / 1000.0f;
        s.meanDescentMs = (endAlt - apogeeAlt) / dt;
    }

    return s;
}

void DataLogger::printSummary(const FlightSummary& s) const {
    if (!s.valid) {
        Serial.println("[FLUG] Keine Daten fuer eine Zusammenfassung.");
        return;
    }

    Serial.println("===== FLUGZUSAMMENFASSUNG =====");
    Serial.print("Messpunkte:        "); Serial.println(s.records);

    Serial.print("Max. Hoehe:        ");
    Serial.print(s.maxAltitudeM, 1);
    Serial.print(" m  bei t = ");
    Serial.print(s.maxAltitudeTimeMs / 1000.0f, 2);
    Serial.println(" s");

    Serial.print("Max. Laengsbeschl.:");
    Serial.print(s.maxAccelG, 2);
    Serial.print(" g  bei t = ");
    Serial.print(s.maxAccelTimeMs / 1000.0f, 2);
    Serial.println(" s");

    Serial.print("Max. Steigrate:    ");
    Serial.print(s.maxSpeedUpMs, 1);
    Serial.println(" m/s");

    if (s.burnoutTimeMs >= 0) {
        Serial.print("Brennzeit:         ");
        Serial.print(s.burnoutTimeMs / 1000.0f, 2);
        Serial.println(" s");
    } else {
        Serial.println("Brennzeit:         nicht erkannt");
    }

    if (s.apogeeTimeMs >= 0) {
        Serial.print("Apogaeum bei:      ");
        Serial.print(s.apogeeTimeMs / 1000.0f, 2);
        Serial.println(" s");
    } else {
        Serial.println("Apogaeum:          nicht erreicht");
    }

    Serial.print("Mittl. Sinkrate:   ");
    Serial.print(s.meanDescentMs, 1);
    Serial.println(" m/s");

    Serial.print("Flugdauer:         ");
    Serial.print(s.lastTimeMs / 1000.0f, 2);
    Serial.println(" s");
    Serial.println("===============================");
}

bool DataLogger::writeSummaryFile(const char* csvName, const FlightSummary& s) {
    if (!cardAvailable || !s.valid) {
        return false;
    }

    // flight001.csv -> flight001.txt
    char txtName[16];
    strncpy(txtName, csvName, sizeof(txtName) - 1);
    txtName[sizeof(txtName) - 1] = '\0';
    size_t len = strlen(txtName);
    if (len >= 4) {
        strcpy(txtName + len - 4, ".txt");
    }

    FsFile f;
    if (!f.open(txtName, O_WRONLY | O_CREAT | O_TRUNC)) {
        Serial.print("[SD] Zusammenfassung konnte nicht geschrieben werden: ");
        Serial.println(txtName);
        return false;
    }

    f.print(F("Flugzusammenfassung zu ")); f.println(csvName);
    f.println();
    f.print(F("Messpunkte:           ")); f.println(s.records);
    f.print(F("Max. Hoehe [m]:       ")); f.print(s.maxAltitudeM, 1);
    f.print(F("   bei t [s]: "));         f.println(s.maxAltitudeTimeMs / 1000.0f, 2);
    f.print(F("Max. Laengsbeschl [g]:")); f.print(s.maxAccelG, 2);
    f.print(F("   bei t [s]: "));         f.println(s.maxAccelTimeMs / 1000.0f, 2);
    f.print(F("Max. Steigrate [m/s]: ")); f.println(s.maxSpeedUpMs, 1);
    f.print(F("Brennzeit [s]:        ")); f.println(s.burnoutTimeMs / 1000.0f, 2);
    f.print(F("Apogaeum bei [s]:     ")); f.println(s.apogeeTimeMs / 1000.0f, 2);
    f.print(F("Mittl. Sinkrate [m/s]:")); f.println(s.meanDescentMs, 1);
    f.print(F("Flugdauer [s]:        ")); f.println(s.lastTimeMs / 1000.0f, 2);

    f.sync();
    f.close();

    Serial.print("[SD] Zusammenfassung geschrieben: ");
    Serial.println(txtName);
    return true;
}

bool DataLogger::dumpFileToSerial(const char* filename) {
    if (!cardAvailable) {
        Serial.println("[SD] Keine Karte verfuegbar.");
        return false;
    }

    FsFile file;
    if (!file.open(filename, O_RDONLY)) {
        Serial.print("[SD] Datei nicht gefunden: ");
        Serial.println(filename);
        return false;
    }

    Serial.print("----- BEGINN ");
    Serial.print(filename);
    Serial.println(" -----");

    // Blockweise statt Zeichen fuer Zeichen: deutlich schneller, und nach
    // jedem Block wird der Watchdog gefuettert - eine volle Flugdatei ist
    // mehrere hundert KB gross, die Ausgabe dauert laenger als 4 s.
    uint8_t chunk[512];
    int n;
    while ((n = file.read(chunk, sizeof(chunk))) > 0) {
        Serial.write(chunk, n);
        Watchdog.reset();
    }

    file.close();
    Serial.print("----- ENDE ");
    Serial.print(filename);
    Serial.println(" -----");
    return true;
}

void DataLogger::listFiles() {
    if (!cardAvailable) {
        Serial.println("[SD] Keine Karte verfuegbar.");
        return;
    }

    Serial.println("[SD] Dateien auf der Karte:");
    sd.ls(LS_SIZE);
}

uint16_t DataLogger::getRecordCount() const {
    return recordCount;
}

bool DataLogger::hasOverflowed() const {
    return overflowed;
}

const char* DataLogger::getLastFilename() const {
    return lastFilename;
}