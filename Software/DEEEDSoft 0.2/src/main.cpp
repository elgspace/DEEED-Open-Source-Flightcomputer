#include <Arduino.h>
#include <Wire.h>
#include <malloc.h>
#include <Adafruit_SleepyDog.h>
#include <bluefruit.h>
#include "ImuSensor.h"
#include "BaroSensor.h"
#include "StateMachine.h"
#include "Actuators.h"
#include "DataLogger.h"

ImuSensor imu(0x6A);
// BMP280 an D4 (SDA) / D5 (SCL), Versorgung (VCC) ueber D6.
// D6 ist eigentlich der TX-Pin der seriellen Schnittstelle Serial1 - die
// wird nicht benutzt (der Monitor laeuft ueber USB), der Pin ist also frei.
// Ueber ihn kann der Sensor komplett aus- und wieder eingeschaltet werden.
const uint8_t BMP_POWER_PIN = 6;
BaroSensor baro(0x76, 1013.25f, SDA, SCL, BMP_POWER_PIN);
StateMachine flightState;

// --- Aktoren (MOSFET-Ausgaenge) ---
const uint8_t IGNITER_PIN  = 2; // D2 - Zuenddraht (Bergungssystem)
const uint8_t BUZZER_PIN = 1; // D1 - Signalgeber zum Wiederfinden

Actuators actuators(IGNITER_PIN, BUZZER_PIN);

// Wie lange der Zuenddraht beim Apogaeum bestromt wird.
// 2000 ms sind fuer einen Zuenddraht ein sicherer Startwert: er muss sich
// erst aufheizen, und die Batteriespannung sackt beim Einschalten kurz ab.
// Zu kurz ist das eigentliche Risiko (Draht wird nicht heiss genug), zu lang
// praktisch keins - nach dem Durchbrennen ist der Stromkreis ohnehin offen.
// Am besten am Boden mit dem echten Draht + Flugakku ausmessen, wie lange es
// tatsaechlich bis zum Durchbrennen dauert, und dann etwas Reserve draufgeben.
const uint32_t IGNITER_FIRE_DURATION_MS = 2000;

// Buzzer-Rhythmus nach der Landung: kurzer Piep, laengere Pause.
// Spart Strom und ist trotzdem gut ortbar.
const uint32_t BUZZER_ON_MS  = 300;
const uint32_t BUZZER_OFF_MS = 700;

// --- SD-Karte (SPI) ---
// Der XIAO nRF52840 nutzt fest D8=SCK, D9=MISO, D10=MOSI.
// CS liegt auf D7 (P1.12). Achtung: D7 ist gleichzeitig UART_RX - solange
// keine serielle Schnittstelle ueber die Pins genutzt wird (der Monitor
// laeuft ueber USB, nicht ueber UART), ist das unproblematisch.
const uint8_t SD_CS_PIN = 7;
DataLogger logger(SD_CS_PIN);

// Logging-Rate, gestaffelt nach Flugphase.
//
// Bis zum Apogaeum wird mit 50 Hz aufgezeichnet - dort passiert alles
// Interessante (Schubaufbau, Brennschluss, Umkehrpunkt) und dort zaehlt jede
// Millisekunde bei der Auswertung.
//
// Im Sinkflug reichen 5 Hz: die Sinkrate ist konstant, es gibt nichts, was
// eine hoehere Aufloesung braucht. Das ist entscheidend fuer den Puffer -
// beim Flug am 29.09. dauerte der Abstieg aus 386 m bei 2,3 m/s Sinkrate
// rund 170 Sekunden; bei 50 Hz waeren das allein 8500 Records gewesen.
// Bei 5 Hz liefert die Aufzeichnung immer noch alle 46 cm einen Messpunkt.
const uint32_t LOG_INTERVAL_ASCENT_MS  = 20;   // 50 Hz bis zum Apogaeum
const uint32_t LOG_INTERVAL_DESCENT_MS = 200;  // 5 Hz ab dem Sinkflug
uint32_t lastLogTime = 0;

// Welches Logging-Intervall fuer den aktuellen Zustand gilt.
uint32_t logIntervalFor(FlightState s) {
    return (s == FlightState::DESCENT || s == FlightState::LANDED)
             ? LOG_INTERVAL_DESCENT_MS
             : LOG_INTERVAL_ASCENT_MS;
}

// Es wird durchgehend aufgezeichnet - vor dem Start in einen kleinen
// Ringpuffer im DataLogger, der beim erkannten Start uebernommen wird.
// So geht der Schubaufbau nicht verloren, und der Hauptpuffer ist trotzdem
// nicht schon voll, wenn die Rakete lange auf der Rampe steht.

// Merkt sich den Zustand des letzten Durchlaufs, um Uebergaenge zu erkennen
// (ein Uebergang ist ein einmaliges Ereignis, kein Dauerzustand).
FlightState previousState = FlightState::PAD;

// Zeitstempel des laufenden Schleifendurchlaufs, damit ein Zustandswechsel
// und der Messpunkt derselben Runde exakt dieselbe Zeitbasis benutzen.
uint32_t launchTimestamp = 0;

// BLE-Beacon laeuft erst ab der Landung, um vorher Strom zu sparen
// und den Funkbetrieb waehrend des Fluges nicht mitlaufen zu lassen.
bool beaconStarted = false;

void startBLEBeacon() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4); // maximale Sendeleistung fuer grosse Reichweite
    Bluefruit.setName("Rocket-Recovery");

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0); // dauerhaft werben, kein Timeout

    Serial.println("[BLE] Beacon 'Rocket-Recovery' gestartet.");
}

// Freien Heap ausgeben.
//
// Warum das ueberhaupt noetig ist: Das Build-Log zeigt nur den STATISCHEN
// Verbrauch - globale Variablen und Puffer wie den Log-Puffer. Was dort
// NICHT auftaucht, sind Task-Stacks, der BLE-Stack und die Puffer, die
// SdFat zur Laufzeit anlegt. Ein RAM-Ueberlauf meldet sich deshalb nicht
// beim Kompilieren, sondern als Absturz im Flug.
//
// Der Heap reicht vom Ende der globalen Variablen bis kurz vor das obere
// RAM-Ende; die Grenzen legt das Linker-Skript des Cores fest
// (__HeapBase/__HeapLimit). mallinfo().uordblks ist der davon aktuell
// belegte Teil (Task-Stacks, SdFat, BLE ...). Frei = Gesamt - belegt.
// (mallinfo().fordblks allein zeigt nur Luecken im schon angeforderten
// Bereich und steht deshalb meist auf 0 - das ist KEIN voller Speicher.)
extern "C" unsigned char __HeapBase[];
extern "C" unsigned char __HeapLimit[];

void printMemInfo(const char* wann) {
    struct mallinfo mi = mallinfo();
    uint32_t heapTotal = (uint32_t)(__HeapLimit - __HeapBase);
    uint32_t heapUsed  = (uint32_t)mi.uordblks;

    Serial.print("[RAM] ");
    Serial.print(wann);
    Serial.print(": Heap belegt ");
    Serial.print(heapUsed / 1024);
    Serial.print(" KB, frei ");
    Serial.print((heapTotal - heapUsed) / 1024);
    Serial.print(" KB von ");
    Serial.print(heapTotal / 1024);
    Serial.print(" KB | Log-Puffer statisch ");
    Serial.print((uint32_t)sizeof(DataLogger) / 1024);
    Serial.println(" KB");
}

// --- Status-LED (RGB-LED auf dem XIAO nRF52840) ---
// ACHTUNG: Die LED ist invertiert verdrahtet - LOW schaltet EIN, HIGH AUS.
// Deshalb wird ueberall !on geschrieben.
void setStatusLed(bool red, bool green, bool blue) {
    digitalWrite(LED_RED,   !red);
    digitalWrite(LED_GREEN, !green);
    digitalWrite(LED_BLUE,  !blue);
}

void statusLedOff() {
    setStatusLed(false, false, false);
}

// Anzeigemodus der Status-LED. Das Blinken laeuft nicht-blockierend ueber
// updateStatusLed(), damit die Hauptschleife nirgends warten muss.
enum class LedMode {
    BOOTING,        // blau, dauerhaft
    READY,          // gruen, dauerhaft
    RECORDING,      // gruen, blinkend - Flug wird aufgezeichnet
    ERROR           // rot, dauerhaft
};

LedMode ledMode = LedMode::BOOTING;
const uint32_t LED_BLINK_INTERVAL_MS = 250;
uint32_t ledLastToggle = 0;
bool ledBlinkOn = true;

void setLedMode(LedMode mode) {
    ledMode = mode;
    ledLastToggle = millis();
    ledBlinkOn = true;

    switch (mode) {
        case LedMode::BOOTING:   setStatusLed(false, false, true); break;
        case LedMode::READY:     setStatusLed(false, true, false); break;
        case LedMode::RECORDING: setStatusLed(false, true, false); break;
        case LedMode::ERROR:     setStatusLed(true, false, false); break;
    }
}

// Bei jedem Schleifendurchlauf aufrufen - kuemmert sich um das Blinken.
void updateStatusLed() {
    if (ledMode != LedMode::RECORDING) {
        return; // alle anderen Modi leuchten konstant, nichts zu tun
    }

    uint32_t now = millis();
    if (now - ledLastToggle >= LED_BLINK_INTERVAL_MS) {
        ledLastToggle = now;
        ledBlinkOn = !ledBlinkOn;
        setStatusLed(false, ledBlinkOn, false);
    }
}

// Reagiert einmalig auf einen Zustandswechsel.
void onStateEntered(FlightState newState) {
    switch (newState) {
        case FlightState::BOOST:
            // Ab hier ist t=0. Der Vorlaufpuffer wird uebernommen, damit
            // auch die Millisekunden vor der Erkennung erhalten bleiben.
            logger.markLaunch(launchTimestamp);
            // Gruen blinkend: Flug wird aufgezeichnet
            setLedMode(LedMode::RECORDING);
            break;

        case FlightState::APOGEE:
            // Zuenddraht bestromen - Bergungssystem freigeben
            Serial.print("[STATE] Apogaeum-Ausloesung: ");
            Serial.println(StateMachine::apogeeTriggerName(flightState.getApogeeTrigger()));
            actuators.fireIgniter(IGNITER_FIRE_DURATION_MS);
            break;

        case FlightState::LANDED:
            // Bergungshilfen aktivieren: Piepen + Funkbake
            actuators.startBuzzer(BUZZER_ON_MS, BUZZER_OFF_MS);
            if (!beaconStarted) {
                startBLEBeacon();
                beaconStarted = true;
            }

            // Jetzt erst auf die Karte schreiben. Die Rakete liegt still,
            // eine kurze Blockade der Hauptschleife ist unkritisch.
            //
            // Das Schreiben mehrerer tausend CSV-Zeilen dauert laenger als
            // die 4 s Watchdog-Zeit. flushToCard() fuettert den Watchdog
            // deshalb selbst waehrend des Schreibens (Hochsetzen geht beim
            // nRF52 nicht, siehe setup()).
            logger.stopRecording();
            // Aufzeichnung beendet - wieder Dauerlicht statt Blinken
            setLedMode(LedMode::READY);
            Watchdog.reset();

            if (logger.isCardAvailable()) {
                // flushToCard() gibt die Flugkennzahlen selbst mit aus und
                // legt sie als flightNNN.txt neben der CSV ab.
                logger.flushToCard();
            } else {
                // Ohne Karte sind die Daten zwar verloren, die Kennzahlen
                // lassen sich aber wenigstens noch anzeigen.
                logger.printSummary(logger.computeSummary());
            }

            // Nach dem Schreiben messen: hier ist der Verbrauch am hoechsten,
            // weil BLE und SdFat beide aktiv sind.
            printMemInfo("nach Landung");

            Watchdog.reset();
            break;

        case FlightState::PAD:
            // Zurueck auf PAD passiert nur beim manuellen Reset im Bodentest.
            // Buzzer abschalten und die Einmal-Sperre loesen, damit der
            // naechste Testlauf sauber startet. Im echten Flug wird dieser
            // Zweig nie erreicht - dort geht es nur vorwaerts bis LANDED.
            actuators.stopBuzzer();
            actuators.resetIgniterLock();
            setLedMode(LedMode::READY);
            break;

        default:
            break;
    }
}

// Resetgrund aus dem Arduino-Core (dort beim Start ausgelesen). Schwach
// deklariert: falls eine Core-Version die Funktion nicht hat, baut das
// Projekt trotzdem, es fehlt dann nur diese eine Angabe.
extern "C" uint32_t readResetReason(void) __attribute__((weak));

// Fortschrittsmeldung beim Start. flush() sorgt dafuer, dass die Zeile
// wirklich beim PC ankommt, BEVOR der naechste Schritt beginnt - haengt
// der, ist die letzte sichtbare Zeile also genau der Uebeltaeter.
void bootLog(const char* step) {
    Watchdog.reset();
    Serial.print("[BOOT] ");
    Serial.print(step);
    Serial.println(" ...");
    Serial.flush();
}

void printResetReason() {
    Serial.print("[DIAG] Resetgrund: ");
    if (!readResetReason) {
        Serial.println("(vom Core nicht verfuegbar)");
        return;
    }
    uint32_t r = readResetReason();
    if (r == 0)            Serial.print("Einschalten/Stromversorgung ");
    if (r & (1UL << 0))    Serial.print("Reset-Taster ");
    if (r & (1UL << 1))    Serial.print("WATCHDOG ");
    if (r & (1UL << 2))    Serial.print("Software-Reset ");
    if (r & (1UL << 3))    Serial.print("CPU-Lockup ");
    if (r & (1UL << 16))   Serial.print("Aufwachen ");
    Serial.print("(0x");
    Serial.print((unsigned long)r, HEX);
    Serial.println(")");
}

bool lastResetWasWatchdog() {
    return readResetReason && (readResetReason() & (1UL << 1));
}

// Ob der Barometer in diesem Bootzyklus verfuegbar ist (false, wenn er
// sich nicht gemeldet hat oder der I2C-Bus blockiert war).
bool baroAvailable = false;

// Sensor-Sampling schnell, Serial-Ausgabe bewusst langsamer
const uint32_t SAMPLE_INTERVAL_MS = 10;   // 100 Hz Sensor-Update
const uint32_t PRINT_INTERVAL_MS  = 200;  // 5 Hz Terminal-Ausgabe

uint32_t lastSampleTime = 0;
uint32_t lastPrintTime = 0;

// --- Manueller Override-Pin fuer Bodentests ---
// D0 / P0.02 auf dem XIAO nRF52840 Sense, direkt neben dem Reset-Taster.
// Pin auf GND ziehen (z.B. per Taster oder Jumperkabel) -> naechster Zustand.
const uint8_t MANUAL_OVERRIDE_PIN = 0; // entspricht D0 / P0.02
const uint32_t OVERRIDE_DEBOUNCE_MS = 50;

bool lastOverridePinState = HIGH;
uint32_t lastOverrideChangeTime = 0;

void checkManualOverride() {
    bool pinState = digitalRead(MANUAL_OVERRIDE_PIN);
    uint32_t now = millis();

    if (pinState != lastOverridePinState && (now - lastOverrideChangeTime) > OVERRIDE_DEBOUNCE_MS) {
        lastOverrideChangeTime = now;
        lastOverridePinState = pinState;

        // Pin ist per INPUT_PULLUP im Ruhezustand HIGH - LOW bedeutet: auf GND gezogen
        if (pinState == LOW) {
            Serial.println("[MANUAL] Erzwinge naechsten Zustand");
            flightState.forceNextState();
        }
    }
}

void setup() {
    // Watchdog SOFORT fuettern. Beim nRF52 laeuft er nach einem Soft-Reset
    // weiter - z.B. nach jedem Hochladen, denn der Bootloader startet das
    // neue Programm per Soft-Reset. Dann bleiben ab hier nur 4 s bis zum
    // naechsten Fuettern, sonst folgt ein Neustart mitten im Setup.
    // Laeuft der Watchdog nicht, passiert beim Fuettern nichts.
    Watchdog.reset();

    // Kurze Wartezeit, damit sich die Versorgungsspannung nach dem
    // Einschalten stabilisiert, bevor Pins und Sensoren angefasst werden.
    delay(300);
    Watchdog.reset();

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    statusLedOff();

    // Aktoren SOFORT auf einen sicheren Zustand setzen (beide MOSFETs aus),
    // bevor irgendetwas anderes passiert. Ein undefinierter Pin-Zustand
    // beim Booten koennte sonst den Trennzylinder ausloesen.
    actuators.begin();

    // Blau = Initialisierung laeuft
    setLedMode(LedMode::BOOTING);

    pinMode(MANUAL_OVERRIDE_PIN, INPUT_PULLUP);

    // Kam der letzte Neustart vom Watchdog? Dann ist der vorherige Start
    // irgendwo haengen geblieben.
    bool lastBootFailed = lastResetWasWatchdog();

    // Haengt USB am PC? (Spannung an VBUS). Nur dann lohnt es sich, laenger
    // auf den Monitor zu warten - im Flug gibt es kein USB.
    bool usbPowered = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;

    Serial.begin(115200);
    // Normal 3 s auf den Monitor warten. Kam der Neustart vom Watchdog und
    // haengt der PC dran, 10 s - damit die Meldungen sicher lesbar sind,
    // auch wenn der Monitor nach dem Neustart erst neu verbinden muss.
    uint32_t serialTimeout = millis() + ((lastBootFailed && usbPowered) ? 10000 : 3000);
    while (!Serial && millis() < serialTimeout) {
        // Falls der Watchdog aus der vorherigen Sitzung noch laeuft (nach
        // einem Soft-Reset moeglich), hier fuettern. Laeuft er nicht,
        // passiert dabei nichts.
        Watchdog.reset();
        delay(10);
    }

    Serial.println();
    Serial.println("=== Flight Computer startet ===");
    printResetReason();
    if (lastBootFailed) {
        Serial.println("[DIAG] Der vorherige Start ist haengen geblieben oder abgestuerzt.");
    }

    // Kurze Wartezeit NACH einem echten Kaltstart (USB-Stromversorgung
    // startet dann bei 0V), damit sich die 3,3V-Versorgung des BMP280 sauber
    // stabilisiert, bevor wir ihn ansprechen. Bei einem reinen MCU-Reset
    // (Taster) ist die Versorgung schon laenger stabil - hier schadet die
    // Wartezeit aber nicht, sie ist einfach kurz genug, um nicht aufzufallen.
    delay(100);
    Watchdog.reset();

    // WICHTIG: Wire.begin() wird bewusst NICHT aufgerufen. Der BMP280 an
    // D4/D5 laeuft ueber eine eigene Software-I2C mit Zeitlimit (SoftI2C),
    // die Wire-Hardware darf diese Pins nicht belegen. Die IMU haengt am
    // internen Bus Wire1 und startet den selbst.

    bootLog("IMU starten");
    bool imuOk = imu.begin();
    Watchdog.reset();

    // Watchdog scharf schalten: Falls JETZT etwas haengt (z.B. der BMP280),
    // wird der Mikrocontroller nach spaetestens 4 Sekunden automatisch neu
    // gestartet, statt fuer immer stehenzubleiben.
    //
    // WICHTIG: Beim nRF52 ist die Watchdog-Zeit nach dem Start fest - sie
    // laesst sich bis zum naechsten Reset weder aendern noch abschalten
    // (Hardware-Eigenschaft, die Bibliothek ignoriert jedes weitere
    // enable() stillschweigend). Alles, was laenger als 4 s dauern kann
    // (SD-Suche, Schreiben nach der Landung, Datei-Ausgabe), muss den
    // Watchdog deshalb zwischendurch selbst mit Watchdog.reset() fuettern.
    Watchdog.enable(4000);

    // Schaltet den BMP280 einmal aus und wieder ein - er startet dadurch
    // bei jedem Einschalten frisch, egal in welchem Zustand er vorher war.
    bootLog("BMP280 aus- und einschalten und starten");
    bool baroOk = baro.begin();

    Watchdog.reset();

    if (!imuOk) {
        Serial.println("Fehler: IMU konnte nicht initialisiert werden!");
    } else {
        Serial.println("IMU erfolgreich initialisiert.");
    }

    if (!baroOk) {
        Serial.println("BMP280 nicht verfuegbar - Flug laeuft nur mit IMU-Daten weiter.");
    } else {
        Serial.println("BMP280 erfolgreich initialisiert.");

        // Startplatz als Null-Referenz einmessen. Die Rakete muss dabei ruhig
        // stehen (dauert ca. 1 Sekunde). Danach zeigt altitudeAglM die Hoehe
        // ueber dem Startplatz an - unabhaengig vom aktuellen Wetterdruck.
        Serial.println("Kalibriere Startplatz-Hoehe, bitte ruhig halten...");
        Watchdog.reset();
        baroOk = baro.calibrateGround(50, 20);
        Watchdog.reset();

        if (baroOk) {
            Serial.print("Startplatz eingemessen. Absolute Referenzhoehe: ");
            Serial.print(baro.getGroundAltitude(), 2);
            Serial.println(" m (Anzeige laeuft ab jetzt relativ dazu)");
        } else {
            Serial.println("Kalibrierung fehlgeschlagen - BMP280 lieferte keine Messwerte.");
        }
    }
    baroAvailable = baroOk;

    // SD-Suche: es werden mehrere SPI-Geschwindigkeiten durchprobiert.
    // logger.begin() fuettert den Watchdog vor jedem Versuch selbst.
    bootLog("SD-Karte suchen");
    logger.begin();
    Watchdog.reset();

    Serial.print("Startzustand: ");
    Serial.println(StateMachine::stateName(flightState.getState()));
    previousState = flightState.getState();

    // Status-LED: Gruen nur, wenn WIRKLICH alles bereit ist - beide Sensoren
    // laufen und die SD-Karte nimmt Daten an. Rot heisst: geflogen werden
    // kann zwar trotzdem, aber irgendetwas fehlt (z.B. kein Logging).
    // So sieht man den Zustand auf der Rampe auf einen Blick, ohne Laptop.
    bool allReady = imuOk && baroOk && logger.isCardAvailable();

    printMemInfo("nach Setup");

    if (allReady) {
        setLedMode(LedMode::READY);
        Serial.println("Status: BEREIT (LED gruen)");
    } else {
        setLedMode(LedMode::ERROR);
        Serial.print("Status: EINGESCHRAENKT (LED rot) -");
        if (!imuOk)  Serial.print(" IMU fehlt");
        if (!baroOk) Serial.print(" Barometer fehlt");
        if (!logger.isCardAvailable()) Serial.print(" SD-Karte fehlt");
        Serial.println();
    }
}

// Einfache Serial-Konsole zum Auslesen der Flugdaten nach dem Flug,
// ohne die Karte ausbauen zu muessen. Befehle im Monitor eintippen:
//   l  = Dateien auf der Karte auflisten
//   d  = zuletzt geschriebene Datei ausgeben
//   w  = Puffer jetzt sofort schreiben (falls die Landung nicht erkannt wurde)
//   r  = SD-Karte erneut initialisieren
//   f  = Flugkennzahlen anzeigen
//   m  = RAM-/Heap-Nutzung anzeigen
//   s  = Statusinfos
void handleSerialCommands() {
    if (!Serial.available()) {
        return;
    }

    char cmd = Serial.read();
    switch (cmd) {
        case 'l':
            logger.listFiles();
            break;

        case 'd': {
            const char* fn = logger.getLastFilename();
            if (fn[0] == '\0') {
                Serial.println("[SD] Noch keine Datei in dieser Sitzung geschrieben. Mit 'l' auflisten.");
            } else {
                logger.dumpFileToSerial(fn); // fuettert den Watchdog selbst
            }
            break;
        }

        case 'w':
            // Notfall-Rettung: falls die Landungserkennung nicht ausgeloest
            // hat, laesst sich der Puffer hier von Hand sichern.
            logger.flushToCard(); // fuettert den Watchdog selbst
            break;

        case 'r':
            // SD-Init erneut versuchen - praktisch beim Debuggen der
            // Verkabelung, ohne jedes Mal neu flashen zu muessen.
            Serial.println("[SD] Erneuter Initialisierungsversuch...");
            logger.begin(); // fuettert den Watchdog selbst
            break;

        case 'f':
            // Flugkennzahlen jederzeit abrufen, auch ohne SD-Karte.
            logger.printSummary(logger.computeSummary());
            break;

        case 'm':
            printMemInfo("aktuell");
            break;

        case 's':
            Serial.print("[STATUS] Zustand: ");
            Serial.print(StateMachine::stateName(flightState.getState()));
            Serial.print(" | Messpunkte: ");
            Serial.print(logger.getRecordCount());
            Serial.print("/");
            Serial.print(LOG_BUFFER_CAPACITY);
            Serial.print(logger.hasOverflowed() ? " (uebergelaufen)" : "");
            Serial.print(" | SD: ");
            Serial.print(logger.isCardAvailable() ? "ok" : "fehlt");
            Serial.print(" | Start erkannt: ");
            Serial.print(logger.hasLaunched() ? "ja" : "nein");
            Serial.print(" | Apogaeum-Ausloesung: ");
            Serial.println(StateMachine::apogeeTriggerName(flightState.getApogeeTrigger()));
            Serial.print("[STATUS] BMP280: ");
            if (!baroAvailable) {
                Serial.print("nicht verfuegbar");
            } else {
                Serial.print(baro.isHealthy() ? "ok" : "STOERUNG");
            }
            Serial.print(" | Lesefehler: ");
            Serial.print(baro.getErrorCount());
            Serial.print(" | Neustarts: ");
            Serial.println(baro.getRestartCount());
            break;

        default:
            break;
    }
}

void loop() {
    // Watchdog "fuettern" - sonst wuerde er nach 4s ohne Reset den MCU neu starten,
    // auch wenn alles normal laeuft.
    Watchdog.reset();

    // Aktoren bei JEDEM Durchlauf bedienen (nicht an das Sample-Intervall
    // gekoppelt), damit das Abschalten des Valves zeitlich praezise ist.
    actuators.update();

    // Status-LED bedienen (Blinken laeuft nicht-blockierend)
    updateStatusLed();

    // Manuellen Override-Pin bei JEDEM Loop-Durchlauf pruefen (nicht an das
    // Sample-Intervall gekoppelt), damit ein kurzer Tastendruck nicht verpasst wird.
    checkManualOverride();

    handleSerialCommands();

    uint32_t now = millis();
    if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
        return;
    }
    lastSampleTime = now;

    ImuReading imuData = imu.read();

    // Baro-Werte nur lesen, wenn der Sensor beim Start erfolgreich initialisiert wurde
    BaroReading baroData = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    if (baroAvailable) {
        baroData = baro.read();
    }

    flightState.update(imuData, baroData);

    // Zustandswechsel erkennen und die zugehoerige Aktion EINMALIG ausloesen.
    // Das funktioniert auch beim manuellen Override, weil der ueber dieselbe
    // State Machine laeuft - Bodentests loesen also dieselben Aktionen aus.
    FlightState currentState = flightState.getState();
    if (currentState != previousState) {
        // 'now' statt millis() weitergeben: der Messpunkt dieser Runde nutzt
        // denselben Zeitstempel, sonst liegt der erste BOOST-Record ein paar
        // Millisekunden vor dem Nullpunkt.
        launchTimestamp = now;
        onStateEntered(currentState);
        previousState = currentState;
    }

    // Messpunkt aufzeichnen (eigenes, langsameres Intervall - im Sinkflug
    // nochmal langsamer, siehe logIntervalFor()).
    // Vor der Starterkennung landet das im Vorlauf-Ringpuffer.
    if (now - lastLogTime >= logIntervalFor(currentState)) {
        lastLogTime = now;
        logger.record(now, imuData, baroData, (uint8_t)currentState);
    }

    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;

        Serial.print("[");
        Serial.print(StateMachine::stateName(flightState.getState()));
        Serial.print("] Accel X:");
        Serial.print(imuData.accelX, 3);
        Serial.print(" Y:");
        Serial.print(imuData.accelY, 3);
        Serial.print(" Z:");
        Serial.print(imuData.accelZ, 3);

        Serial.print(" | Gyro X:");
        Serial.print(imuData.gyroX, 3);
        Serial.print(" Y:");
        Serial.print(imuData.gyroY, 3);
        Serial.print(" Z:");
        Serial.print(imuData.gyroZ, 3);

        Serial.print(" | ");
        if (baroAvailable) {
            Serial.print("Temp:");
            Serial.print(baroData.temperatureC, 2);
            Serial.print(" Druck:");
            Serial.print(baroData.pressureHpa, 2);
            Serial.print(" Hoehe(AGL):");
            Serial.print(baroData.altitudeAglM, 2);
            Serial.print(" (roh:");
            Serial.print(baroData.altitudeAglRawM, 2);
            Serial.print(") vSpeed:");
            Serial.print(baroData.verticalSpeedMs, 1);
            Serial.print(" m/s");
            if (!baro.isHealthy()) {
                Serial.print(" [BARO-STOERUNG, Wert eingefroren]");
            }
            Serial.println();
        } else {
            Serial.println("Baro: nicht verfuegbar");
        }
    }
}