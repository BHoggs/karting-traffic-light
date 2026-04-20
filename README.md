# Arduino Traffic Light Controller

A PlatformIO project for an Arduino Uno that powers the traffic light system used at my local go-kart weekly league.

## Overview

Two independent traffic light channels (A and B) each use an ultrasonic sensor to detect an approaching kart. When a kart comes within the configured trigger distance, the corresponding light switches from green to red via a relay, holding it red for a configurable duration before automatically resetting.

Settings (trigger distance and red-light hold time) are adjustable at runtime using an analog keypad, with live feedback shown on a 16×2 LCD display.

## Hardware

- **Microcontroller:** Arduino Uno
- **Sensors:** 2× HC-SR04 ultrasonic distance sensors (one per channel)
- **Output:** Relays controlling the red and green lights for each channel
- **Display:** 16×2 LCD (direct pin wiring via hd44780)
- **Input:** Analog keypad (single resistor-ladder on A0)

## Dependencies

Managed via PlatformIO (`platformio.ini`):

| Library | Purpose |
|---|---|
| `hd44780` | LCD driver |
| `PinChangeInterrupt` | Pin-change ISR support |
| `LiquidCrystal` / `LiquidCrystal_I2C` | LCD compatibility |

## Building & Flashing

1. Open the project in VS Code with the PlatformIO extension installed.
2. Connect the Arduino Uno via USB.
3. Run **PlatformIO: Upload** (or `pio run -t upload`).
