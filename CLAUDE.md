# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Coupe de France de Robotique Junior 2026** project. It consists of multiple independent PlatformIO subprojects, each flashed to a separate microcontroller. There is no top-level build system — each subdirectory is its own self-contained PlatformIO project.

## Build & Flash Commands

Run from within a subproject directory (e.g. `cd base_roulante`):

```bash
pio run                        # compile
pio run --target upload        # compile and flash
pio device monitor             # open serial monitor at 115200 baud
pio run --target upload && pio device monitor  # flash then monitor
```

## Architecture: Data Flow

```
telecommande (Arduino Uno)
  → SerialTransfer (UART, 115200)
    → emetteur (ESP32)
      → ESP-NOW (WiFi)
        → recepteur (ESP32-C3)  → SerialTransfer → base_roulante (ESP32)
                                                 → carte_actionneurs (ESP32)

pami (ESP32-C3)  [fully autonomous — tirette-triggered, no radio]
```

## Subprojects Summary

| Directory | Board | Role |
|---|---|---|
| `telecommande` | Arduino Uno (AVR) | Reads joysticks (A0–A3) + buttons, sends `STRUCT` via SerialTransfer over USB Serial |
| `emetteur` | ESP32 (`esp32dev`) | Bridges SerialTransfer UART → ESP-NOW, transmits to `recepteur` |
| `recepteur` | ESP32-C3 | Receives ESP-NOW, forwards raw bytes over Serial1 (UART) to downstream boards |
| `base_roulante` | ESP32 (`esp32dev`) | Receives `STRUCT`, drives 4 stepper motors (mecanum/holonomic drive) via FastAccelStepper |
| `carte_actionneurs` | ESP32 (`esp32doit-devkit-v1`) | Receives `STRUCT`, drives 32 servos via two PCA9685 PWM drivers (I2C 0x40 and 0x41) |
| `pami` | ESP32-C3 | Autonomous robot: waits for tirette, counts down, runs a pre-programmed movement sequence, then oscillates a servo |

## Shared Packet Structure

The `STRUCT` is **copy-pasted into every module** — there is no shared header. Any change must be made in all affected files:

```cpp
struct __attribute__((packed)) STRUCT {
  int16_t x;           // joystick X axis (0–1023 raw from telecommande)
  int16_t y;           // joystick Y axis
  int16_t z;           // joystick Z axis (rotation)
  uint16_t compteur[32]; // PWM values for 32 servos (used by carte_actionneurs)
} message;
```

## Hardware Notes

- All serial communication runs at **115200 baud**
- `emetteur` MAC address is hardcoded as `broadcastAddress[]`; `recepteur` filters by `emetteurAddress[]` — these must match
