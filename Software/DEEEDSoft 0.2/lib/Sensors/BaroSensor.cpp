#include "BaroSensor.h"

// --- BMP280-Register (Datenblatt Kap. 4.2) ---
static const uint8_t REG_CALIB     = 0x88;  // 24 Byte Kalibrierwerte
static const uint8_t REG_CHIP_ID   = 0xD0;
static const uint8_t REG_RESET     = 0xE0;
static const uint8_t REG_STATUS    = 0xF3;
static const uint8_t REG_CTRL_MEAS = 0xF4;
static const uint8_t REG_CONFIG    = 0xF5;
static const uint8_t REG_DATA      = 0xF7;  // 6 Byte: Druck, dann Temperatur

// Einstellungen wie bisher mit der Adafruit-Bibliothek: auf Tempo optimiert.
// ctrl_meas: Temperatur x1, Druck x1, Normalmodus (dauernd messen)
// config:    0,5 ms Pause zwischen Messungen, Hardwarefilter aus -
//            das Glaetten uebernimmt der Medianfilter in dieser Klasse.
static const uint8_t CTRL_MEAS_VALUE = (0x1 << 5) | (0x1 << 2) | 0x3;  // 0x27
static const uint8_t CONFIG_VALUE    = 0x00;

// Nach so vielen Fehlversuchen in Folge wird der Sensor neu gestartet.
// Einzelne Aussetzer (z.B. eine Stoerung beim Zuenden) loesen noch nichts aus.
static const uint8_t ERRORS_BEFORE_RESTART = 3;

// Neustart: so lange bleibt der Sensor stromlos, dann so lange Zeit zum
// Hochfahren und fuer die erste Messung.
static const uint32_t POWER_OFF_MS = 50;
static const uint32_t SETTLE_MS    = 15;

BaroSensor::BaroSensor(uint8_t i2cAddress, float seaLevelHpa,
                       uint8_t sdaPin, uint8_t sclPin, uint8_t powerPin)
    : bus(sdaPin, sclPin), address(i2cAddress),
      seaLevelHpa(seaLevelHpa), powerPin(powerPin) {}

void BaroSensor::powerOff() {
    // Datenleitungen mit auf LOW, sonst versorgt sich der Sensor ueber die
    // Pull-Ups an SDA/SCL weiter und geht nicht richtig aus.
    bus.driveLow();
    digitalWrite(powerPin, LOW);
#ifdef OUTPUT_H0H1
    pinMode(powerPin, OUTPUT_H0H1);
#else
    pinMode(powerPin, OUTPUT);
#endif
}

void BaroSensor::powerOn() {
    // "High Drive": der Pin liefert dann bis zu 10 mA statt 2 mA, die
    // Spannung am Sensor bricht unter Last kaum ein.
#ifdef OUTPUT_H0H1
    pinMode(powerPin, OUTPUT_H0H1);
#else
    pinMode(powerPin, OUTPUT);
#endif
    digitalWrite(powerPin, HIGH);
    bus.begin();
}

bool BaroSensor::begin() {
    powerOff();
    delay(POWER_OFF_MS);
    powerOn();
    delay(SETTLE_MS);

    if (!bus.busIdle()) {
        bus.recover();
    }
    if (!configure()) {
        return false;
    }
    delay(SETTLE_MS); // erste Messung abwarten

    float t, p;
    if (!measure(t, p)) {
        return false;
    }
    health = Health::OK;
    consecutiveErrors = 0;
    return true;
}

bool BaroSensor::configure() {
    uint8_t id = 0;
    if (!bus.readRegs(address, REG_CHIP_ID, &id, 1)) {
        return false;
    }
    // 0x58 = BMP280. 0x56/0x57 = Vorserien-BMP280, 0x60 = BME280
    // (gleiche Register fuer Druck/Temperatur) - alle funktionieren hier.
    if (id != 0x58 && id != 0x56 && id != 0x57 && id != 0x60) {
        return false;
    }

    // Soft-Reset: bringt den Sensor in einen definierten Zustand, egal was
    // er vorher gemacht hat. Danach kopiert er ca. 2 ms lang seine
    // Kalibrierwerte intern um.
    if (!bus.writeReg(address, REG_RESET, 0xB6)) {
        return false;
    }
    delay(3);
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t status = 0;
        if (!bus.readRegs(address, REG_STATUS, &status, 1)) {
            return false;
        }
        if ((status & 0x01) == 0) {
            break; // im_update fertig
        }
        delay(1);
    }

    uint8_t c[24];
    if (!bus.readRegs(address, REG_CALIB, c, sizeof(c))) {
        return false;
    }
    digT1 = (uint16_t)(c[0] | (c[1] << 8));
    digT2 = (int16_t)(c[2] | (c[3] << 8));
    digT3 = (int16_t)(c[4] | (c[5] << 8));
    digP1 = (uint16_t)(c[6] | (c[7] << 8));
    digP2 = (int16_t)(c[8] | (c[9] << 8));
    digP3 = (int16_t)(c[10] | (c[11] << 8));
    digP4 = (int16_t)(c[12] | (c[13] << 8));
    digP5 = (int16_t)(c[14] | (c[15] << 8));
    digP6 = (int16_t)(c[16] | (c[17] << 8));
    digP7 = (int16_t)(c[18] | (c[19] << 8));
    digP8 = (int16_t)(c[20] | (c[21] << 8));
    digP9 = (int16_t)(c[22] | (c[23] << 8));

    if (digT1 == 0 || digP1 == 0) {
        return false; // Kalibrierwerte unbrauchbar (Lesefehler)
    }

    return bus.writeReg(address, REG_CONFIG, CONFIG_VALUE) &&
           bus.writeReg(address, REG_CTRL_MEAS, CTRL_MEAS_VALUE);
}

int32_t BaroSensor::compensateTemperature(int32_t adcT, uint16_t t1, int16_t t2,
                                          int16_t t3, int32_t& tFine) {
    // Bosch-Referenzformel, Ergebnis in 1/100 Grad C
    int32_t var1 = ((((adcT >> 3) - ((int32_t)t1 << 1))) * ((int32_t)t2)) >> 11;
    int32_t var2 = (((((adcT >> 4) - ((int32_t)t1)) *
                      ((adcT >> 4) - ((int32_t)t1))) >> 12) *
                    ((int32_t)t3)) >> 14;
    tFine = var1 + var2;
    return (tFine * 5 + 128) >> 8;
}

uint32_t BaroSensor::compensatePressure(int32_t adcP, int32_t tFine,
                                        uint16_t p1, int16_t p2, int16_t p3,
                                        int16_t p4, int16_t p5, int16_t p6,
                                        int16_t p7, int16_t p8, int16_t p9) {
    // Bosch-Referenzformel (64 Bit), Ergebnis in Pa * 256 -
    // dieselbe Rechnung, die auch die Adafruit-Bibliothek benutzt.
    int64_t var1 = ((int64_t)tFine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)p6;
    var2 = var2 + ((var1 * (int64_t)p5) << 17);
    var2 = var2 + (((int64_t)p4) << 35);
    var1 = ((var1 * var1 * (int64_t)p3) >> 8) + ((var1 * (int64_t)p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)p1) >> 33;
    if (var1 == 0) {
        return 0; // Division durch 0 vermeiden
    }
    int64_t p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)p7) << 4);
    return (uint32_t)p;
}

bool BaroSensor::measure(float& temperatureC, float& pressurePa) {
    // Druck und Temperatur in EINEM Lesevorgang: beide stammen garantiert
    // aus derselben Messung (siehe Kommentar in read()).
    uint8_t d[6];
    if (!bus.readRegs(address, REG_DATA, d, sizeof(d))) {
        return false;
    }
    int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);

    // 0x80000 ist der Wert nach einem Reset, bevor die erste Messung fertig
    // ist. Alles 0 oder alles 1 deutet auf einen gestoerten Lesevorgang.
    if (adcP == 0x80000 || adcT == 0x80000 ||
        adcP == 0 || adcP == 0xFFFFF) {
        return false;
    }

    int32_t tFine;
    int32_t t100 = compensateTemperature(adcT, digT1, digT2, digT3, tFine);
    uint32_t p256 = compensatePressure(adcP, tFine, digP1, digP2, digP3, digP4,
                                       digP5, digP6, digP7, digP8, digP9);
    if (p256 == 0) {
        return false;
    }

    temperatureC = t100 / 100.0f;
    pressurePa = p256 / 256.0f;

    // Plausibilitaet: ausserhalb dieser Grenzen ist es ein Lesefehler,
    // kein echter Messwert (Messbereich des Sensors: 300-1100 hPa).
    if (pressurePa < 30000.0f || pressurePa > 110000.0f ||
        temperatureC < -40.0f || temperatureC > 85.0f) {
        return false;
    }
    return true;
}

void BaroSensor::handleError() {
    errorCount++;
    if (health != Health::OK) {
        return; // Neustart laeuft bereits
    }
    consecutiveErrors++;
    if (consecutiveErrors >= ERRORS_BEFORE_RESTART) {
        Serial.println("[BARO] Sensor antwortet nicht - schalte ihn aus und wieder ein.");
        powerOff();
        health = Health::POWERED_OFF;
        healthSince = millis();
        restartCount++;
    }
}

void BaroSensor::serviceRestart() {
    uint32_t now = millis();
    if (health == Health::POWERED_OFF && now - healthSince >= POWER_OFF_MS) {
        powerOn();
        health = Health::SETTLING;
        healthSince = now;
    } else if (health == Health::SETTLING && now - healthSince >= SETTLE_MS) {
        if (!bus.busIdle()) {
            bus.recover();
        }
        // configure() blockiert einmalig ca. 3-5 ms (Soft-Reset) - das ist
        // ein einzelner ausgelassener Messzyklus, kein Problem.
        if (configure()) {
            Serial.println("[BARO] Sensor neu gestartet, misst wieder.");
            health = Health::OK;
            consecutiveErrors = 0;
            // Nicht sofort lesen: die erste Messung braucht noch ein paar ms.
            // Die naechsten Fehlversuche zaehlen deshalb nicht gleich wieder
            // fuer einen neuen Neustart.
        } else {
            // Klappt nicht - noch einmal von vorn.
            powerOff();
            health = Health::POWERED_OFF;
            healthSince = now;
            restartCount++;
        }
    }
}

bool BaroSensor::isHealthy() const {
    return health == Health::OK && consecutiveErrors == 0;
}

uint32_t BaroSensor::getErrorCount() const {
    return errorCount;
}

uint16_t BaroSensor::getRestartCount() const {
    return restartCount;
}

float BaroSensor::pressureToAltitude(float pressureHpa) const {
    // Internationale Hoehenformel (Standardatmosphaere)
    return 44330.0f * (1.0f - powf(pressureHpa / seaLevelHpa, 0.1903f));
}

bool BaroSensor::calibrateGround(uint16_t samples, uint16_t delayMs) {
    double sum = 0.0;
    uint16_t good = 0;
    for (uint16_t i = 0; i < samples; i++) {
        float t, p;
        if (measure(t, p)) {
            sum += pressureToAltitude(p / 100.0f);
            good++;
        }
        delay(delayMs);
    }
    if (good == 0) {
        return false;
    }
    groundAltitudeM = (float)(sum / good);

    // Filterpuffer leeren: die Kalibrierwerte sollen die erste
    // Geschwindigkeitsschaetzung nach dem Start nicht verfaelschen.
    medianCount = 0;
    medianIndex = 0;
    speedCount = 0;
    speedIndex = 0;
    return true;
}

float BaroSensor::getGroundAltitude() const {
    return groundAltitudeM;
}

float BaroSensor::medianOf(float newValue) {
    // Neuen Wert in den Ringpuffer schreiben
    medianBuf[medianIndex] = newValue;
    medianIndex = (medianIndex + 1) % ALT_MEDIAN_WINDOW;
    if (medianCount < ALT_MEDIAN_WINDOW) {
        medianCount++;
    }

    // Kopie sortieren. Bei fuenf Elementen ist Insertion Sort das Schnellste,
    // was man hier sinnvoll tun kann - kein Grund fuer etwas Komplizierteres.
    float tmp[ALT_MEDIAN_WINDOW];
    for (uint8_t i = 0; i < medianCount; i++) {
        tmp[i] = medianBuf[i];
    }
    for (uint8_t i = 1; i < medianCount; i++) {
        float key = tmp[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    return tmp[medianCount / 2];
}

float BaroSensor::updateVerticalSpeed(float filteredAlt, uint32_t nowMs) {
    speedAlt[speedIndex] = filteredAlt;
    speedTime[speedIndex] = nowMs;
    speedIndex = (speedIndex + 1) % VSPEED_WINDOW;
    if (speedCount < VSPEED_WINDOW) {
        speedCount++;
    }

    // Solange nur wenige Werte vorliegen, waere die Schaetzung ueber ein sehr
    // kurzes Fenster viel zu verrauscht - dann lieber 0 melden.
    if (speedCount < 4) {
        return 0.0f;
    }

    // Aeltesten gueltigen Eintrag finden: ist der Puffer voll, steht an der
    // aktuellen Schreibposition der naechste zu ueberschreibende und damit
    // aelteste Wert; sonst beginnt der Puffer bei Index 0.
    uint8_t oldest = (speedCount < VSPEED_WINDOW) ? 0 : speedIndex;
    uint8_t newest = (speedIndex + VSPEED_WINDOW - 1) % VSPEED_WINDOW;

    uint32_t dtMs = speedTime[newest] - speedTime[oldest];
    if (dtMs == 0) {
        return 0.0f;
    }

    return (speedAlt[newest] - speedAlt[oldest]) / (dtMs / 1000.0f);
}

BaroReading BaroSensor::read() {
    // Laeuft gerade ein Neustart, nur den naechsten Schritt erledigen und
    // den letzten gueltigen Wert zurueckgeben.
    if (health != Health::OK) {
        serviceRestart();
        return lastReading;
    }

    // EINE Messung als Grundlage fuer Druck, Temperatur UND Hoehe.
    // (Frueher wurden Druck und Hoehe getrennt gelesen; waehrend schneller
    // Druckaenderungen passten die Zahlen dann nicht zusammen - im
    // Flugdatensatz vom 25.09. standen 995,5 hPa neben 124,8 m.)
    float temperatureC, pressurePa;
    if (!measure(temperatureC, pressurePa)) {
        // Letzten gueltigen Wert halten: die Hoehe friert ein, statt auf 0
        // zu springen. Fuer die Flugerkennung ist das die sichere Variante -
        // ein Sprung auf 0 saehe aus wie ein Absturz, ein eingefrorener Wert
        // loest nichts aus. Das Apogaeum kommt dann ueber den Backup-Timer.
        handleError();
        return lastReading;
    }
    consecutiveErrors = 0;

    BaroReading r;
    r.temperatureC = temperatureC;
    r.pressureHpa = pressurePa / 100.0f;
    r.altitudeM = pressureToAltitude(r.pressureHpa);

    // Hoehe ueber Startplatz: der wetterbedingte Offset steckt in beiden
    // Werten gleichermassen drin und kuerzt sich bei der Differenz heraus.
    r.altitudeAglRawM = r.altitudeM - groundAltitudeM;

    // Medianfilter gegen Einzelausreisser. Die Flugerkennung arbeitet auf
    // diesem Wert, die CSV bekommt beide.
    r.altitudeAglM = medianOf(r.altitudeAglRawM);

    r.verticalSpeedMs = updateVerticalSpeed(r.altitudeAglM, millis());

    lastReading = r;
    return r;
}