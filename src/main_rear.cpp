// ============================================================
//  Traffic Light Controller v3
//  Refactored with structs and a generic state-machine pattern.
// ============================================================

#include <Arduino.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>
#include <PinChangeInterrupt.h>
#include <math.h>   // ceil()
#include "pit_bay_shared.h"

// ============================================================
//  Compile-time constants
// ============================================================

// Rear-specific constants (shared constants are in pit_bay_shared.h)
constexpr unsigned long GREEN_COOLDOWN = 1500UL;  // ms

// ============================================================
//  Types
// ============================================================

enum LightState : uint8_t { GREEN, RED };

// All data for one traffic-light channel
struct TrafficLight {
    int           trigPin;
    int           echoPin;
    int           greenPin;
    int           redPin;
    char          label;    // 'A' or 'B' — used for the LCD prefix
    uint8_t       lcdRow;   // 0 or 1
    float         distance; // Latest sensor reading in cm; -1 = no echo
    LightState    state;
    unsigned long redTimer; // millis() when RED was entered / last extended
    unsigned long greenCooldownUntil; // millis() until GREEN can retrigger
    uint8_t       triggerCount;       // consecutive sensor hits in GREEN (noise filter)
};

// ============================================================
//  Global instances
// ============================================================

hd44780_pinIO lcd(LCD_RS, LCD_EN, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

//                      trig  echo  green  red   lbl  row  dist   state   redTimer cooldown  trgCnt
TrafficLight lightA = {  11,  12,    A5,   A4,  'A',  0, 0.0f, GREEN,      0,        0,       0 };
TrafficLight lightB = {   2,   3,    A3,   A2,  'B',  1, 0.0f, GREEN,      0,        0,       0 };

Settings       settings     = { 50, 5000UL };
DisplayCache   displayCache = { -1, 0UL }; // -1 forces the first draw

unsigned long  lastLcdUpdate     = 0;
unsigned long  lastLedToggle     = 0;
unsigned long  lastButtonTime    = 0; // debounce: single timer for the shared pin
bool           ledState          = false;
volatile bool  buttonPressedFlag = false; // set by ISR, processed in loop()

// ============================================================
//  Function prototypes
// ============================================================

void  initTrafficLight(const TrafficLight& light);
void  setLightState(TrafficLight& light, LightState newState);
void  updateTrafficLight(TrafficLight& light, unsigned long now);
void  updateLCDRow(const TrafficLight& light, unsigned long now);
void  updateLCDDisplay(unsigned long now);
void  buttonSharedISR();

// ============================================================
//  Setup
// ============================================================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    initTrafficLight(lightA);
    initTrafficLight(lightB);

    attachPCINT(digitalPinToPCINT(BUTTON_PIN), buttonSharedISR, FALLING);

    lcd.init();
    lcd.backlight();

    setLightState(lightA, GREEN);
    setLightState(lightB, GREEN);

    unsigned long now = millis();
    lastLcdUpdate = now;
    lastLedToggle = now;

    updateLCDDisplay(now); // draw initial screen
}

// ============================================================
//  Main loop
// ============================================================

void loop() {
    unsigned long now = millis();

    lightA.distance = (lightA.state == GREEN && now < lightA.greenCooldownUntil)
                      ? -1.0f
                      : readUltrasonicDistance(lightA.trigPin, lightA.echoPin);
    lightB.distance = (lightB.state == GREEN && now < lightB.greenCooldownUntil)
                      ? -1.0f
                      : readUltrasonicDistance(lightB.trigPin, lightB.echoPin);

    updateTrafficLight(lightA, now);
    updateTrafficLight(lightB, now);

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

    delay(10);
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
    digitalWrite(light.greenPin, newState == GREEN ? HIGH : LOW);
    digitalWrite(light.redPin,   newState == RED   ? HIGH : LOW);
}

// ============================================================
//  State machine — called every loop iteration for each light
// ============================================================

void updateTrafficLight(TrafficLight& light, unsigned long now) {
    const int           trigDist  = settings.triggerDistance;
    const unsigned long redDur    = settings.timerDuration;
    const bool          greenLocked = (light.state == GREEN && now < light.greenCooldownUntil);
    // A reading of -1 means no echo: don't treat as a trigger
    const bool          triggered = (light.distance > 0.0f && light.distance < trigDist);

    switch (light.state) {

        case GREEN:
            if (!greenLocked && triggered) {
                if (++light.triggerCount >= 3) {
                    setLightState(light, RED);
                    light.redTimer     = now;
                    light.triggerCount = 0;
                }
            } else {
                light.triggerCount = 0;
            }
            break;

        case RED:
            if (triggered) {
                light.redTimer = now; // extend while object is still present
            }
            else if (now - light.redTimer >= redDur) {
                setLightState(light, GREEN);
                light.greenCooldownUntil = now + GREEN_COOLDOWN;
            }
            break;
    }
}

// ============================================================
//  LCD display
// ============================================================

// Renders the left half of one LCD row for a traffic-light channel:
//   GREEN state:  "SA:123  "  (distance in cm, or "---" on timeout / cooldown)
//   RED   state:  "SA:ST-3 "  (seconds remaining before turning green)
void updateLCDRow(const TrafficLight& light, unsigned long now) {
    char buf[9];
    lcd.setCursor(0, light.lcdRow);
    lcd.print('S');
    lcd.print(light.label);
    lcd.print(':');

    if (light.state == GREEN) {
        if (light.distance < 0.0f) {
            lcd.print("--- ");
        } else {
            snprintf(buf, sizeof(buf), "%-4d", (int)ceil(light.distance));
            lcd.print(buf);
        }
    } else {
        unsigned long elapsed   = now - light.redTimer;
        unsigned long remaining = (elapsed < settings.timerDuration)
                                  ? settings.timerDuration - elapsed : 0UL;
        snprintf(buf, sizeof(buf), "ST-%-2d", (int)ceil(remaining / 1000.0f));
        lcd.print(buf);
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
