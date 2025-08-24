/*
  Project: Smart Irrigation System
  Features:
  - Soil moisture analog sensing with moving average
  - Finite State Machine: IDLE -> WATERING -> COOLDOWN
  - Relay control with min/max run times and cooldown
  - Optional tank level (float switch) interlock
  - Optional manual override via Serial: START, STOP
  - Telemetry and parameter tuning via Serial
*/

#include <EEPROM.h>

// ------------------ USER CONFIG ------------------
const uint8_t PIN_SOIL = A0;
const uint8_t PIN_RELAY_PUMP = 7;
const bool RELAY_ACTIVE_HIGH = true;

// Optional water tank level switch (NC/NO set by logic)
const bool USE_TANK_SWITCH = true;
const uint8_t PIN_TANK_SWITCH = 4; // HIGH = water present (wire accordingly)
const bool TANK_HIGH_MEANS_WATER = true;

// Soil thresholds (raw 0..1023): lower = wetter
uint16_t SOIL_DRY  = 450; // start watering when avg <= SOIL_DRY ? (choose by sensor)
uint16_t SOIL_WET  = 520; // stop watering when avg >= SOIL_WET (hysteresis inverted because some sensors reverse)
                          // Adjust after checking your sensor orientation!

// Timing
const uint32_t SAMPLE_MS = 200;
const uint32_t TELEMETRY_MS = 2000;
const uint32_t MIN_RUN_MS = 20UL * 1000UL;     // ensure at least 20s watering
const uint32_t MAX_RUN_MS = 120UL * 1000UL;    // safety limit 2 minutes
const uint32_t COOLDOWN_MS = 60UL * 1000UL;    // wait 1 min before next cycle

// Moving average window
const uint8_t AVG_N = 15;

// EEPROM layout
const uint16_t MAGIC = 0xBEEF;
const int EE_MAGIC = 0;
const int EE_DRY = 2;
const int EE_WET = 4;
// -------------------------------------------------

enum State { ST_IDLE, ST_WATERING, ST_COOLDOWN };
State state = ST_IDLE;

uint16_t buf[30]; uint8_t head=0, cnt=0;
uint32_t tSample=0, tTele=0, tStateStart=0;

void setPump(bool on) {
  digitalWrite(PIN_RELAY_PUMP, (RELAY_ACTIVE_HIGH ? on : !on) ? HIGH : LOW);
}

bool tankHasWater() {
  if (!USE_TANK_SWITCH) return true;
  int r = digitalRead(PIN_TANK_SWITCH);
  return TANK_HIGH_MEANS_WATER ? (r == HIGH) : (r == LOW);
}

uint16_t readSoil() { return analogRead(PIN_SOIL); }
uint16_t smooth(uint16_t v) {
  buf[head] = v;
  head = (head + 1) % AVG_N;
  if (cnt < AVG_N) cnt++;
  uint32_t s = 0; for (uint8_t i=0;i<cnt;i++) s += buf[i];
  return s / cnt;
}

void eeload() {
  uint16_t m; EEPROM.get(EE_MAGIC, m);
  if (m == MAGIC) {
    EEPROM.get(EE_DRY, SOIL_DRY);
    EEPROM.get(EE_WET, SOIL_WET);
  }
}
void eesave() {
  EEPROM.put(EE_MAGIC, MAGIC);
  EEPROM.put(EE_DRY, SOIL_DRY);
  EEPROM.put(EE_WET, SOIL_WET);
}

void printStatus(uint16_t raw, uint16_t avg) {
  Serial.print(F("STATE="));
  Serial.print(state==ST_IDLE?"IDLE":state==ST_WATERING?"WATERING":"COOLDOWN");
  Serial.print(F(" RAW=")); Serial.print(raw);
  Serial.print(F(" AVG=")); Serial.print(avg);
  Serial.print(F(" DRY=")); Serial.print(SOIL_DRY);
  Serial.print(F(" WET=")); Serial.print(SOIL_WET);
  Serial.print(F(" TANK=")); Serial.print(tankHasWater()?"OK":"EMPTY");
  Serial.print(F(" ELAPSE=")); Serial.print(millis()-tStateStart);
  Serial.println();
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim(); s.toUpperCase();
  if (s=="HELP") {
    Serial.println(F("Commands: STATUS, START, STOP, SET DRY x, SET WET x, SAVE, HELP"));
  } else if (s=="STATUS") {
    printStatus(0,0);
  } else if (s=="START") {
    if (state==ST_IDLE && tankHasWater()) { state=ST_WATERING; tStateStart=millis(); setPump(true); Serial.println(F("Watering...")); }
    else Serial.println(F("Cannot START"));
  } else if (s=="STOP") {
    setPump(false); state = ST_COOLDOWN; tStateStart=millis(); Serial.println(F("Stopped -> Cooldown"));
  } else if (s.startsWith("SET DRY ")) {
    SOIL_DRY = constrain(s.substring(8).toInt(), 0, 1023); Serial.println(F("OK"));
  } else if (s.startsWith("SET WET ")) {
    SOIL_WET = constrain(s.substring(8).toInt(), 0, 1023); Serial.println(F("OK"));
  } else if (s=="SAVE") {
    eesave(); Serial.println(F("Saved"));
  } else {
    Serial.println(F("Unknown. Type HELP"));
  }
}

void setup() {
  pinMode(PIN_RELAY_PUMP, OUTPUT); setPump(false);
  if (USE_TANK_SWITCH) pinMode(PIN_TANK_SWITCH, INPUT_PULLUP);
  Serial.begin(9600);
  eeload();
  tSample = tTele = tStateStart = millis();
  Serial.println(F("Irrigation boot. Type HELP."));
}

void loop() {
  handleSerial();

  uint32_t now = millis();
  static uint16_t lastRaw=0, lastAvg=0;

  if (now - tSample >= SAMPLE_MS) {
    tSample = now;
    lastRaw = readSoil();
    lastAvg = smooth(lastRaw);

    switch (state) {
      case ST_IDLE:
        if (lastAvg <= SOIL_DRY && tankHasWater()) {
          state = ST_WATERING; tStateStart = now; setPump(true);
        }
        break;

      case ST_WATERING: {
        uint32_t run = now - tStateStart;
        bool wetEnough = lastAvg >= SOIL_WET;
        bool hitMin = run >= MIN_RUN_MS;
        bool hitMax = run >= MAX_RUN_MS;

        if (!tankHasWater() || hitMax || (wetEnough && hitMin)) {
          setPump(false);
          state = ST_COOLDOWN; tStateStart = now;
        }
      } break;

      case ST_COOLDOWN:
        if (now - tStateStart >= COOLDOWN_MS) {
          state = ST_IDLE; tStateStart = now;
        }
        break;
    }
  }

  if (now - tTele >= TELEMETRY_MS) {
    tTele = now;
    printStatus(lastRaw, lastAvg);
  }
}
