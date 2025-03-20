# CG2271 RTOS-Based Robotic Car 🚗💨

## Project Overview
This project is developed for **CG2271: Real-Time Operating Systems** and focuses on designing an **RTOS-based robotic car** controlled via an Android app or an alternative controller. The car is capable of movement, LED signaling, and audio output.

## Features
- ** Wireless Control:** The car connects to an PS4 controller to ESP32 via Bluetooth.
- ** Remote Navigation:** Supports **forward, backward, left, and right movements**, including curved turns.
- ** LED Indications:**
  - Green LEDs animate while moving.
  - Red LEDs flash at different rates for movement vs. stationary mode.
- ** Audio Output:** 
  - Continuous background music during the run.
  - A unique tone plays when the challenge is completed.

## RTOS Architecture
The project follows **CMSIS-RTOS2 (Keil RTX or FreeRTOS)** with the following tasks:

| Task Name       | Type          | Function |
|----------------|--------------|----------|
| `Serial_ISR`   | Interrupt    | Captures UART data from ESP32. |
| `tBrain`       | RTOS Task    | Decodes commands and dispatches them to other tasks. |
| `tMotorControl`| RTOS Task    | Controls PWM signals to move the robot. |
| `tLED`         | RTOS Task    | Manages LED animations and states. |
| `tAudio`       | RTOS Task    | Plays continuous music and challenge completion tones. |

