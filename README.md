# Arduino Traffic Light Controller

A PlatformIO project for an Arduino Uno that powers the traffic light system used at my local go-kart weekly league.

## Overview

Two independent traffic light channels (A and B) each use an HC-SR04 ultrasonic sensor to detect a kart. There are two firmware variants depending on where the sensor is mounted in the pit bay:

| Firmware | Sensor position | Build environment |
|---|---|---|
| `main_front.cpp` | Front of the bay — detects the kart arriving | `uno_front` |
| `main_rear.cpp` | Rear of the bay — detects the kart already stopped | `uno_rear` |

Both variants share pin assignments, timing constants, sensor reading, and button handling via the `lib/pit_bay_shared` library. Each maintains two independent channels (A and B) with settings adjustable at runtime via an analog keypad, shown on a 16×2 LCD.

To reject spurious reflections, both variants require **3 consecutive sensor hits** within the trigger distance before acting on a detection.

---

## Firmware: Front (`uno_front`)

The sensor is at the **entrance** of the bay. The kart trips it on the way in, triggering a stop countdown. When the countdown expires the light goes green, authorising the kart to leave. The system returns to red once the kart has fully cleared the sensor.

### State machine

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                                                         │
                    ▼                                                         │
          ┌─────────────────┐                                                 │
          │                 │  3 consecutive hits                             │
          │  WAITING        ├────────────────────►  ┌─────────────────┐      │
          │  (solid red)    │                        │                 │      │
          └─────────────────┘                        │  COUNTDOWN      │      │
                    ▲                                │  (flashing red) │      │
                    │                                └────────┬────────┘      │
                    │                                         │               │
                    │  sensor clear ≥ 0.5 s                  │  sensor clear  │
                    │                                         │  ≥ 1 s        │
                    │                          timer expires  │  (abort)      │
                    │                                         │               │
                    │                  ┌──────────────────────┘               │
                    │                  │                                      │
                    │                  ▼                                      │
                    │        ┌─────────────────┐                              │
                    └────────┤                 │                              │
                             │  GO             ├──────────────────────────────┘
                             │  (solid green)  │  sensor re-tripped
                             └─────────────────┘  (clearSince reset; stays green)
```

| State | Light | Condition to leave |
|---|---|---|
| **WAITING** | Solid red | 3 consecutive hits within trigger distance → **COUNTDOWN** |
| **COUNTDOWN** | Flashing red (1 Hz) | Timer expires → **GO**; or sensor clear ≥ 1 s → **WAITING** (abort) |
| **GO** | Solid green | Sensor clear ≥ 0.5 s → **WAITING** |

---

## Firmware: Rear (`uno_rear`)

The sensor is at the **back** of the bay. The kart is already stopped when it trips the sensor. The light holds red for a configurable duration then goes green, signalling the kart to leave. A post-green cooldown prevents the sensor from immediately re-triggering.

### State machine

```
          ┌─────────────────┐
          │                 │  3 consecutive hits
          │  GREEN          ├────────────────────►  ┌─────────────────┐
          │  (solid green)  │                        │                 │
          └────────▲────────┘                        │  RED            │
                   │                                 │  (solid red)    │
                   │  timer expires                  └────────┬────────┘
                   │  + 1.5 s cooldown                        │
                   │                                          │  still present:
                   └──────────────────────────────────────────┘  reset timer
```

| State | Light | Condition to leave |
|---|---|---|
| **GREEN** | Solid green | 3 consecutive hits → **RED**; 1.5 s cooldown after returning prevents immediate re-trigger |
| **RED** | Solid red | Timer expires (no kart present) → **GREEN**; kart still present resets the timer |

---

## Hardware

- **Microcontroller:** Arduino Uno
- **Sensors:** 2× HC-SR04 ultrasonic distance sensors (one per channel)
- **Output:** Relays controlling the red and green lights for each channel
- **Display:** 16×2 LCD (direct pin wiring via hd44780)
- **Input:** Analog keypad (single resistor-ladder on A0)

### LCD layout

```
SA:<status>  D:<cm>
SB:<status>  T:<s>
```

`<status>` is the live sensor distance (cm) when idle, `ST-N` (seconds remaining) during a timed phase, or `GO` when the kart is cleared to leave. `D:` and `T:` show the current trigger distance and timer duration, adjustable via the keypad.

### Keypad buttons

| Button | Action |
|---|---|
| RIGHT | Increase trigger distance |
| LEFT | Decrease trigger distance |
| UP | Increase timer duration |
| DOWN | Decrease timer duration |

---

## Dependencies

Managed via PlatformIO (`platformio.ini`):

| Library | Purpose |
|---|---|
| `hd44780` | LCD driver |
| `PinChangeInterrupt` | Pin-change ISR for keypad |
| `LiquidCrystal` / `LiquidCrystal_I2C` | LCD compatibility |

## Building & Flashing

```bash
# Front-bay controller
pio run -e uno_front -t upload

# Rear-bay controller
pio run -e uno_rear -t upload
```
