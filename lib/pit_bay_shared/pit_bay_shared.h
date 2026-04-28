// ============================================================
//  Shared constants, types, and utilities
//  Common to both the front-bay and rear-bay controllers.
// ============================================================

#pragma once
#include <Arduino.h>

// ============================================================
//  Compile-time constants
// ============================================================

constexpr int           DISTANCE_STEP       =    5;    // cm
constexpr int           DISTANCE_MAX        =  150;    // cm
constexpr int           DEFAULT_DISTANCE    =  100;    // cm — initial trigger distance
constexpr unsigned long TIME_STEP           = 1000UL;  // ms
constexpr unsigned long RED_DURATION_MAX    = 120000UL;// ms (120 s)
constexpr unsigned long DEFAULT_DURATION    = 10000UL; // ms — initial timer duration
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
//  Shared types
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

// ============================================================
//  Shared functions
// ============================================================

// Returns distance in cm, or -1.0 on timeout (no echo within 30 ms ~ 5 m)
float readUltrasonicDistance(int trigPin, int echoPin);

// Reads the analog keypad on buttonPin and applies the result to settings.
void handleButtonPress(int buttonPin, Settings& settings);
