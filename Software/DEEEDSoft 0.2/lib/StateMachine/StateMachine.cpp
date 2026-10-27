#include "StateMachine.h"

// ---- Schwellenwerte fuer die automatische Erkennung ----
// Angepasst nach dem Flug vom 25.09.2026. Vor jedem scharfen Flug mit
// geaenderter Motorkonfiguration erneut pruefen!

// --- Start ---
static const float LIFTOFF_ACCEL_G = 2.0f;            // Beschleunigung entlang der Laengsachse (Z)
static const uint32_t LIFTOFF_CONFIRM_SAMPLES = 5;    // so viele Samples in Folge noetig (Entprellung)

// --- Brennschluss ---
static const float BURNOUT_ACCEL_G = 0.5f;            // darunter = Motor aus / Freiflug
static const uint32_t BURNOUT_CONFIRM_SAMPLES = 5;

// --- Apogaeum ---
static const float APOGEE_DROP_M = 1.0f;              // Hoehenabfall gegenueber Maximum
static const uint32_t APOGEE_CONFIRM_SAMPLES = 5;

// Coast-Lockout: so lange nach dem Brennschluss wird KEIN Apogaeum erkannt.
//
// AKTUELL DEAKTIVIERT (0 = aus). Beim Flug vom 25.09. meldete der Barometer
// unmittelbar nach dem Brennschluss einen Hoehensturz von 143 m auf 37 m
// innerhalb von 120 ms, waehrend die Rakete real mit 93 m/s stieg - Ursache
// war der Drucksprung beim Wegfall des Abgasstrahls. Ein Lockout von 2000 ms
// haette dieses Fenster vollstaendig ausgeblendet.
//
// Ohne Lockout haengt der Schutz allein an der Geschwindigkeitspruefung
// unten. Die haette denselben Fall ebenfalls abgefangen (880 m/s scheinbare
// Sinkrate), arbeitet aber auf denselben Barometerdaten, die die Stoerung
// verursacht haben. Zum Aktivieren einfach auf 2000 setzen.
static const uint32_t COAST_LOCKOUT_MS = 0;

// Plausibilitaetsgrenze fuer die Sinkrate bei der Apogaeum-Erkennung.
// Am echten Apogaeum ist die Vertikalgeschwindigkeit per Definition nahe
// null; selbst wenige hundert Millisekunden danach liegt sie erst bei
// wenigen m/s. Alles jenseits dieser Grenze ist kein Apogaeum, sondern ein
// Messfehler - beim Flug vom 25.09. waren es 880 m/s.
// 30 m/s laesst reichlich Luft fuer Sensorrauschen (bei 200 ms Fenster und
// +/-1 m Rauschen sind gut 10 m/s Streuung moeglich) und liegt trotzdem
// weit unter jedem realen Stoerungswert.
static const float APOGEE_MAX_SPEED_MS = 30.0f;

// Sicherheitsnetz: Spaetestens so lange nach dem Brennschluss wird das
// Apogaeum ausgeloest, auch wenn die Erkennung nicht gegriffen hat.
//
// Auslegung nach dem Flug vom 29.09.: Das Apogaeum wurde dort 7,16 s nach
// dem Brennschluss erreicht. Mit den urspruenglichen 10 s blieben nur
// 2,84 s Reserve - bei etwas mehr Schub oder weniger Masse haette der
// Timer zuerst ausgeloest, und zwar bei noch steigender Rakete.
// 15 s lassen rund den doppelten Abstand und liegen weiterhin deutlich vor
// einem moeglichen Aufschlag (ballistisch waeren das etwa 15 s nach
// Brennschluss). Bei staerkerem Motor nach oben anpassen.
static const uint32_t APOGEE_BACKUP_MS = 15000;

// --- Landung ---
static const float LANDING_STABLE_RANGE_M = 1.0f;     // max. erlaubte Hoehenschwankung
static const uint32_t LANDING_STABLE_MS = 3000;       // ... ueber diese Zeitdauer

// IMU-Zusatzbedingung: Am Boden misst der Beschleunigungssensor nur noch die
// Schwerkraft, und das Gyroskop steht still. Am Fallschirm ist beides nicht
// der Fall - die Rakete pendelt und dreht sich.
static const float LANDING_ACCEL_MIN_G = 0.85f;
static const float LANDING_ACCEL_MAX_G = 1.15f;
static const float LANDING_GYRO_MAX_DPS = 30.0f;

// Absolutes Zeitlimit fuer den Sinkflug. Greift, wenn weder Barometer noch
// IMU eine Landung melden - ohne das wuerde LANDED nie erreicht, und damit
// wuerden auch die Flugdaten nie auf die SD-Karte geschrieben.
//
// Auslegung nach dem Flug vom 29.09.: Dort brach die Aufzeichnung bei 104 m
// Hoehe ab, weil die urspruenglichen 120 s abgelaufen waren - die Sinkrate
// lag bei nur 2,3 m/s, der Abstieg aus 386 m haette rund 170 s gedauert.
// 240 s decken das mit gut 40 % Reserve ab.
//
// Nicht beliebig hoch setzen: Solange dieser Timer laeuft, bleibt die
// Rakete im Zustand DESCENT - Buzzer und Funkbake starten erst danach,
// und erst dann werden die Daten geschrieben. Wer deutlich hoeher fliegt,
// erhoeht den Wert UND prueft, ob der Log-Puffer noch reicht.
static const uint32_t DESCENT_TIMEOUT_MS = 240000;


StateMachine::StateMachine()
    : currentState(FlightState::PAD), stateEnteredAt(0) {}

FlightState StateMachine::getState() const {
    return currentState;
}

ApogeeTrigger StateMachine::getApogeeTrigger() const {
    return apogeeTrigger;
}

void StateMachine::transitionTo(FlightState newState) {
    if (newState == currentState) {
        return;
    }

    Serial.print("[STATE] ");
    Serial.print(stateName(currentState));
    Serial.print(" -> ");
    Serial.println(stateName(newState));

    currentState = newState;
    stateEnteredAt = millis();

    // Zaehler/Fenster aus dem vorherigen Zustand zuruecksetzen
    liftoffCounter = 0;
    burnoutCounter = 0;
    apogeeCounter = 0;
    landingWindowStarted = false;
    imuCalmWindowStarted = false;

    if (newState == FlightState::COAST) {
        maxAltitudeSeen = -10000.0f; // Hoehen-Tracking fuer Apogaeum neu beginnen
    }
}

uint32_t StateMachine::timeInState() const {
    return millis() - stateEnteredAt;
}

const char* StateMachine::stateName(FlightState state) {
    switch (state) {
        case FlightState::PAD:     return "PAD";
        case FlightState::BOOST:   return "BOOST";
        case FlightState::COAST:   return "COAST";
        case FlightState::APOGEE:  return "APOGEE";
        case FlightState::DESCENT: return "DESCENT";
        case FlightState::LANDED:  return "LANDED";
        default:                   return "UNKNOWN";
    }
}

const char* StateMachine::apogeeTriggerName(ApogeeTrigger trigger) {
    switch (trigger) {
        case ApogeeTrigger::BAROMETRIC:   return "barometrisch";
        case ApogeeTrigger::BACKUP_TIMER: return "Backup-Timer";
        case ApogeeTrigger::MANUAL:       return "manuell";
        default:                          return "keiner";
    }
}

void StateMachine::forceNextState() {
    if (currentState == FlightState::COAST) {
        apogeeTrigger = ApogeeTrigger::MANUAL;
    }

    switch (currentState) {
        case FlightState::PAD:     transitionTo(FlightState::BOOST);   break;
        case FlightState::BOOST:   transitionTo(FlightState::COAST);   break;
        case FlightState::COAST:   transitionTo(FlightState::APOGEE);  break;
        case FlightState::APOGEE:  transitionTo(FlightState::DESCENT); break;
        case FlightState::DESCENT: transitionTo(FlightState::LANDED);  break;
        case FlightState::LANDED:
            apogeeTrigger = ApogeeTrigger::NONE;
            transitionTo(FlightState::PAD);
            break;
    }
}

void StateMachine::update(const ImuReading& imuData, const BaroReading& baroData) {
    switch (currentState) {

        case FlightState::PAD: {
            // Start-Erkennung: anhaltend hohe Beschleunigung entlang der Laengsachse
            if (imuData.accelZ > LIFTOFF_ACCEL_G) {
                liftoffCounter++;
            } else {
                liftoffCounter = 0;
            }

            if (liftoffCounter >= LIFTOFF_CONFIRM_SAMPLES) {
                transitionTo(FlightState::BOOST);
            }
            break;
        }

        case FlightState::BOOST: {
            // Brennschluss-Erkennung: Beschleunigung faellt deutlich ab
            if (imuData.accelZ < BURNOUT_ACCEL_G) {
                burnoutCounter++;
            } else {
                burnoutCounter = 0;
            }

            if (burnoutCounter >= BURNOUT_CONFIRM_SAMPLES) {
                transitionTo(FlightState::COAST);
            }
            break;
        }

        case FlightState::COAST: {
            uint32_t inCoast = timeInState();

            // Maximalhoehe durchgehend mitverfolgen - auch waehrend eines
            // eventuellen Lockouts, damit danach der richtige Bezugswert
            // bereitsteht.
            if (baroData.altitudeAglM > maxAltitudeSeen) {
                maxAltitudeSeen = baroData.altitudeAglM;
                apogeeCounter = 0;
            } else if (baroData.altitudeAglM < maxAltitudeSeen - APOGEE_DROP_M) {
                // Zaehler nur hochzaehlen, wenn die Sinkrate ueberhaupt
                // physikalisch moeglich ist. Genau hier haette der Flug vom
                // 25.09. gestoppt werden muessen.
                if (fabsf(baroData.verticalSpeedMs) < APOGEE_MAX_SPEED_MS) {
                    apogeeCounter++;
                } else {
                    apogeeCounter = 0;
                }
            }

            bool lockoutOver = (inCoast >= COAST_LOCKOUT_MS);

            if (lockoutOver && apogeeCounter >= APOGEE_CONFIRM_SAMPLES) {
                apogeeTrigger = ApogeeTrigger::BAROMETRIC;
                transitionTo(FlightState::APOGEE);
                break;
            }

            // Sicherheitsnetz: Erkennung hat nicht gegriffen
            if (inCoast >= APOGEE_BACKUP_MS) {
                Serial.println("[STATE] Apogaeum nicht erkannt - Backup-Timer loest aus.");
                apogeeTrigger = ApogeeTrigger::BACKUP_TIMER;
                transitionTo(FlightState::APOGEE);
            }
            break;
        }

        case FlightState::APOGEE: {
            // Momentzustand: hier loest main.cpp den Zuenddraht aus,
            // danach direkt weiter zu DESCENT.
            transitionTo(FlightState::DESCENT);
            break;
        }

        case FlightState::DESCENT: {
            uint32_t now = millis();

            // --- Bedingung 1: Hoehe bleibt ueber einen Zeitraum stabil ---
            if (!landingWindowStarted) {
                lastAltitudeForLanding = baroData.altitudeAglM;
                landingStableSince = now;
                landingWindowStarted = true;
            }

            float altitudeDelta = fabsf(baroData.altitudeAglM - lastAltitudeForLanding);
            if (altitudeDelta > LANDING_STABLE_RANGE_M) {
                lastAltitudeForLanding = baroData.altitudeAglM;
                landingStableSince = now;
            }
            bool baroSaysLanded = (now - landingStableSince >= LANDING_STABLE_MS);

            // --- Bedingung 2: die IMU sieht Ruhe ---
            // Am Boden misst der Sensor nur die Schwerkraft und das Gyroskop
            // steht still. Am Schirm pendelt und dreht sich die Rakete, dort
            // ist diese Bedingung nicht dauerhaft erfuellt.
            float accelMag = sqrtf(imuData.accelX * imuData.accelX +
                                   imuData.accelY * imuData.accelY +
                                   imuData.accelZ * imuData.accelZ);
            float gyroMag = sqrtf(imuData.gyroX * imuData.gyroX +
                                  imuData.gyroY * imuData.gyroY +
                                  imuData.gyroZ * imuData.gyroZ);

            bool imuCalm = (accelMag > LANDING_ACCEL_MIN_G) &&
                           (accelMag < LANDING_ACCEL_MAX_G) &&
                           (gyroMag < LANDING_GYRO_MAX_DPS);

            if (!imuCalm) {
                imuCalmWindowStarted = false;
            } else if (!imuCalmWindowStarted) {
                imuCalmSince = now;
                imuCalmWindowStarted = true;
            }
            bool imuSaysLanded = imuCalmWindowStarted &&
                                 (now - imuCalmSince >= LANDING_STABLE_MS);

            // Beide Sensoren muessen zustimmen - das verhindert, dass eine
            // ruhige Phase am Schirm faelschlich als Landung zaehlt.
            if (baroSaysLanded && imuSaysLanded) {
                transitionTo(FlightState::LANDED);
                break;
            }

            // Notausgang: Wenn keiner der beiden Sensoren je zustimmt, wuerde
            // LANDED nie erreicht - und damit landeten auch die Flugdaten nie
            // auf der Karte. Nach dem Zeitlimit wird deshalb umgeschaltet.
            if (timeInState() >= DESCENT_TIMEOUT_MS) {
                Serial.println("[STATE] Landung nicht erkannt - Zeitlimit abgelaufen, sichere Daten.");
                transitionTo(FlightState::LANDED);
            }
            break;
        }

        case FlightState::LANDED: {
            // Endzustand - nichts zu tun, bis manueller Reset/Neustart
            break;
        }
    }
}