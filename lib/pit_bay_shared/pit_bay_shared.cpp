#include "pit_bay_shared.h"

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

