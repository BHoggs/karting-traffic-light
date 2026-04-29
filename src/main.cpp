// ============================================================
//  Traffic Light Controller v3
//  Front-bay sensor: WAITING → COUNTDOWN → GO state machine.
// ============================================================

#include <Arduino.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>
#include <PinChangeInterrupt.h>
#include <math.h>   // ceil()

// ============================================================
//  Compile-time constants
// ============================================================

// Sensor / noise filter
constexpr unsigned long SENSOR_PING_INTERVAL    =   20UL;  // ms — minimum time between pings per sensor (cross-talk prevention)
constexpr uint8_t       SENSOR_TRIGGER_COUNT    =    3;    // consecutive hits required to transition out of WAITING

// State-machine timings
constexpr unsigned long GREEN_CLEAR_DEBOUNCE    =  500UL;  // ms — kart must be clear before returning to WAITING
constexpr unsigned long COUNTDOWN_CLEAR_TIMEOUT = 1000UL;  // ms — kart must be clear during stop to abort

// User-adjustable settings — range and defaults
constexpr int           DISTANCE_STEP       =    5;    // cm
constexpr int           DISTANCE_MAX        =  150;    // cm
constexpr int           DEFAULT_DISTANCE    =  100;    // cm — initial trigger distance
constexpr unsigned long TIME_STEP           = 1000UL;  // ms
constexpr unsigned long RED_DURATION_MAX    = 120000UL;// ms (120 s)
constexpr unsigned long DEFAULT_DURATION    = 10000UL; // ms — initial timer duration

// Timing
constexpr unsigned long DEBOUNCE_DELAY      =  200UL;  // ms
constexpr unsigned long LCD_UPDATE_INTERVAL =  500UL;  // ms
constexpr unsigned long LED_TOGGLE_INTERVAL =  500UL;  // ms

// Analog-keypad upper thresholds (one resistor ladder on A0)
constexpr int BTN_RIGHT_MAX  =  60;
constexpr int BTN_UP_MAX     = 200;
constexpr int BTN_DOWN_MAX   = 400;
constexpr int BTN_LEFT_MAX   = 600;
constexpr int BTN_SELECT_MAX = 800;

// Pin assignments
constexpr int BUTTON_PIN = A0;
constexpr int LCD_RS     =  8;
constexpr int LCD_EN     =  9;
constexpr int LCD_DB4    =  4;
constexpr int LCD_DB5    =  5;
constexpr int LCD_DB6    =  6;
constexpr int LCD_DB7    =  7;

// ============================================================
//  Types
// ============================================================

// User-adjustable settings (read and written only in loop())
struct Settings {
    int           triggerDistance; // cm
    unsigned long timerDuration;   // ms — stop/countdown timer
};

// Cached copies of displayed settings — avoids redundant LCD writes
struct DisplayCache {
    int           triggerDistance;
    unsigned long timerDuration;
};

enum LightState : uint8_t { WAITING, COUNTDOWN, GO };

// All data for one traffic-light channel
struct TrafficLight {
    int           trigPin;
    int           echoPin;
    int           greenPin;
    int           redPin;
    char          label;          // 'A' or 'B' — used for the LCD prefix
    uint8_t       lcdRow;         // 0 or 1
    float         distance;       // Latest sensor reading in cm; -1 = no echo
    LightState    state;
    unsigned long countdownStart; // millis() when COUNTDOWN was entered
    unsigned long clearSince;     // millis() when sensor first became clear; 0 while tripped
    unsigned long lastFlashToggle;// millis() of last red-LED flash toggle
    bool          redFlashOn;     // current flash output level during COUNTDOWN
    uint8_t       triggerCount;   // consecutive sensor hits in WAITING (noise filter)
};

// ============================================================
//  Global instances
// ============================================================

hd44780_pinIO lcd(LCD_RS, LCD_EN, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

//                       trig  echo  green  red   lbl  row  dist     state   cntStart  clearSince  flashToggle  flashOn  trgCnt
TrafficLight lightA = {   11,  12,    A5,   A4,  'A',  0, 0.0f, WAITING,       0,          0,          0,      false,     0 };
TrafficLight lightB = {    2,   3,    A3,   A2,  'B',  1, 0.0f, WAITING,       0,          0,          0,      false,     0 };

Settings       settings     = { DEFAULT_DISTANCE, DEFAULT_DURATION };
DisplayCache   displayCache = { -1, 0UL }; // -1 forces the first draw

unsigned long  lastLcdUpdate     = 0;
unsigned long  lastLedToggle     = 0;
unsigned long  lastButtonTime    = 0; // debounce: single timer for the shared pin
unsigned long  nextPingAt        = 0; // millis() when the next sensor ping is due
uint8_t        nextPingLight     = 0; // 0 = ping lightA next, 1 = ping lightB next
bool           ledState          = false;
volatile bool  buttonPressedFlag = false; // set by ISR, processed in loop()

// ============================================================
//  Function prototypes
// ============================================================

void  initTrafficLight(const TrafficLight& light);
void  setLightState(TrafficLight& light, LightState newState);
void  updateTrafficLight(TrafficLight& light, unsigned long now, bool distanceUpdated);
void  updateLCDRow(const TrafficLight& light, unsigned long now);
void  updateLCDDisplay(unsigned long now);
void  buttonSharedISR();
float readUltrasonicDistance(int trigPin, int echoPin);
void  handleButtonPress(int buttonPin, Settings& settings);

// ============================================================
//  Setup
// ============================================================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    initTrafficLight(lightA);
    initTrafficLight(lightB);

    pinMode(BUTTON_PIN, INPUT_PULLUP); // float-guard: holds A0 high (~1023) when keypad is disconnected
    attachPCINT(digitalPinToPCINT(BUTTON_PIN), buttonSharedISR, FALLING);

    lcd.init();
    lcd.backlight();

    setLightState(lightA, WAITING);
    setLightState(lightB, WAITING);

    unsigned long now = millis();
    lastLcdUpdate = now;
    lastLedToggle = now;
    nextPingAt    = now; // first ping fires immediately on the first loop iteration

    updateLCDDisplay(now); // draw initial screen
}

// ============================================================
//  Main loop
// ============================================================

void loop() {
    unsigned long now = millis();

    // Fire at most one sensor per slot; schedule the next slot after the blocking
    // pulseIn call completes so the gap is measured in real elapsed time.
    bool freshA = false, freshB = false;
    if (now >= nextPingAt) {
        if (nextPingLight == 0) {
            lightA.distance = readUltrasonicDistance(lightA.trigPin, lightA.echoPin);
            freshA = true;
        } else {
            lightB.distance = readUltrasonicDistance(lightB.trigPin, lightB.echoPin);
            freshB = true;
        }
        nextPingLight = !nextPingLight;
        nextPingAt    = millis() + SENSOR_PING_INTERVAL; // millis() — not now — accounts for pulseIn block time
    }

    now = millis(); // refresh after potential pulseIn block
    updateTrafficLight(lightA, now, freshA);
    updateTrafficLight(lightB, now, freshB);

    // Process button press here — analogRead (~104 µs) and settings writes
    // are unsafe inside an ISR on AVR, so the ISR only sets a flag.
    // One debounce timer suffices because the resistor ladder can only
    // register one button at a time.
    if (buttonPressedFlag) {
        buttonPressedFlag = false;
        if (now - lastButtonTime > DEBOUNCE_DELAY) {
            lastButtonTime = now;
            handleButtonPress(BUTTON_PIN, settings);
            // Redraw immediately so the new value is visible without delay
            updateLCDDisplay(now);
            lastLcdUpdate = now;
        }
    }

    if (now - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        updateLCDDisplay(now);
        lastLcdUpdate = now;
    }

    if (now - lastLedToggle >= LED_TOGGLE_INTERVAL) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        lastLedToggle = now;
    }
}

// ============================================================
//  Traffic light helpers
// ============================================================

// Configures the GPIO pins for one traffic-light channel
void initTrafficLight(const TrafficLight& light) {
    pinMode(light.trigPin,  OUTPUT);
    pinMode(light.echoPin,  INPUT);
    pinMode(light.greenPin, OUTPUT);
    pinMode(light.redPin,   OUTPUT);
}

// Drives the LEDs and updates the state field
void setLightState(TrafficLight& light, LightState newState) {
    light.state = newState;
    switch (newState) {
        case WAITING:
            digitalWrite(light.greenPin, LOW);
            digitalWrite(light.redPin,   HIGH);
            break;
        case COUNTDOWN:
            // Red starts OFF immediately (visual cue of detection); flash loop takes over
            digitalWrite(light.greenPin, LOW);
            digitalWrite(light.redPin,   LOW);
            light.redFlashOn      = false;
            light.lastFlashToggle = millis();
            break;
        case GO:
            digitalWrite(light.greenPin, HIGH);
            digitalWrite(light.redPin,   LOW);
            break;
    }
}

// ============================================================
//  State machine — called every loop iteration for each light
// ============================================================

void updateTrafficLight(TrafficLight& light, unsigned long now, bool distanceUpdated) {
    const int  trigDist = settings.triggerDistance;
    // A reading of -1 means no echo: treat as clear (no kart present)
    const bool present  = (light.distance > 0.0f && light.distance < trigDist);

    switch (light.state) {

        case WAITING:
            if (present) {
                if (distanceUpdated && ++light.triggerCount >= SENSOR_TRIGGER_COUNT) {
                    setLightState(light, COUNTDOWN);
                    light.countdownStart = now;
                    light.clearSince     = 0;
                    light.triggerCount   = 0;
                }
            } else if (distanceUpdated) {
                light.triggerCount = 0;
            }
            break;

        case COUNTDOWN:
            // Flash the red LED at LED_TOGGLE_INTERVAL
            if (now - light.lastFlashToggle >= LED_TOGGLE_INTERVAL) {
                light.redFlashOn = !light.redFlashOn;
                digitalWrite(light.redPin, light.redFlashOn ? HIGH : LOW);
                light.lastFlashToggle = now;
            }
            // Track how long the sensor has been clear
            if (present) {
                light.clearSince = 0;
            } else if (light.clearSince == 0) {
                light.clearSince = now;
            }
            // Kart left during stop — reset to WAITING
            if (light.clearSince != 0 &&
                    (now - light.clearSince >= COUNTDOWN_CLEAR_TIMEOUT)) {
                setLightState(light, WAITING);
                light.clearSince = 0;
            }
            // Countdown complete — signal GO
            else if (now - light.countdownStart >= settings.timerDuration) {
                setLightState(light, GO);
                light.clearSince = 0;
            }
            break;

        case GO:
            // Track how long the sensor has been clear
            if (present) {
                light.clearSince = 0;
            } else if (light.clearSince == 0) {
                light.clearSince = now;
            }
            // Kart has cleared — return to WAITING
            if (light.clearSince != 0 &&
                    (now - light.clearSince >= GREEN_CLEAR_DEBOUNCE)) {
                setLightState(light, WAITING);
                light.clearSince = 0;
            }
            break;
    }
}

// ============================================================
//  LCD display
// ============================================================

// Renders the left half of one LCD row for a traffic-light channel:
//   WAITING   state: "SA:123  "  (live distance in cm, or "---" on no echo)
//   COUNTDOWN state: "SA:ST-3 "  (seconds remaining before GO)
//   GO        state: "SA:GO   "
void updateLCDRow(const TrafficLight& light, unsigned long now) {
    char buf[9];
    lcd.setCursor(0, light.lcdRow);
    lcd.print('S');
    lcd.print(light.label);
    lcd.print(':');

    if (light.state == WAITING) {
        if (light.distance < 0.0f) {
            lcd.print("---   ");
        } else {
            snprintf(buf, sizeof(buf), "%-6d", (int)ceil(light.distance));
            lcd.print(buf);
        }
    } else if (light.state == COUNTDOWN) {
        unsigned long elapsed   = now - light.countdownStart;
        unsigned long remaining = (elapsed < settings.timerDuration)
                                  ? settings.timerDuration - elapsed : 0UL;
        snprintf(buf, sizeof(buf), "ST-%-3d", (int)ceil(remaining / 1000.0f));
        lcd.print(buf);
    } else {
        lcd.print("GO    ");
    }
}

// Redraws the full LCD; settings columns are only rewritten when changed
void updateLCDDisplay(unsigned long now) {
    if (settings.triggerDistance != displayCache.triggerDistance) {
        displayCache.triggerDistance = settings.triggerDistance;
        char buf[7];
        snprintf(buf, sizeof(buf), "D:%-3d ", settings.triggerDistance);
        lcd.setCursor(10, 0);
        lcd.print(buf);
    }
    if (settings.timerDuration != displayCache.timerDuration) {
        displayCache.timerDuration = settings.timerDuration;
        char buf[7];
        snprintf(buf, sizeof(buf), "T:%-3lu ", settings.timerDuration / 1000UL);
        lcd.setCursor(10, 1);
        lcd.print(buf);
    }
    updateLCDRow(lightA, now);
    updateLCDRow(lightB, now);
}

// ============================================================
//  Button ISR  (FALLING edge on BUTTON_PIN)
// ============================================================

// Kept as short as possible — no ADC reads, no millis(), no settings writes.
void buttonSharedISR() {
    buttonPressedFlag = true;
}

// ============================================================
//  Sensor and button helpers
// ============================================================

// Returns distance in cm, or -1.0 on timeout (no echo within 30 ms ~ 5 m)
float readUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000UL);
    if (duration == 0) return -1.0f;
    return duration * 0.034f / 2.0f;
}

void handleButtonPress(int buttonPin, Settings& settings) {
    int adcValue = analogRead(buttonPin);
    if (adcValue < BTN_RIGHT_MAX) {
        if (settings.triggerDistance + DISTANCE_STEP <= DISTANCE_MAX)
            settings.triggerDistance += DISTANCE_STEP;
    } else if (adcValue < BTN_UP_MAX) {
        if (settings.timerDuration + TIME_STEP <= RED_DURATION_MAX)
            settings.timerDuration += TIME_STEP;
    } else if (adcValue < BTN_DOWN_MAX) {
        settings.timerDuration = (settings.timerDuration > TIME_STEP)
                                  ? settings.timerDuration - TIME_STEP : 0UL;
    } else if (adcValue < BTN_LEFT_MAX) {
        settings.triggerDistance = (settings.triggerDistance > DISTANCE_STEP)
                                    ? settings.triggerDistance - DISTANCE_STEP : 0;
    }
}
