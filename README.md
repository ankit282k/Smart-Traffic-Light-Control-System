# 🚦 Smart Traffic Light System using STM32

A Smart Traffic Light Controller designed to improve traffic flow efficiency and safety using real-time vehicle detection and user input. Built with STM32 microcontroller, ultrasonic sensors, 4-digit 7-segment LED display, and a 4x4 matrix keypad.

---

## 📌 Features

- **Real-time vehicle detection** using ultrasonic sensors
- **Adaptive signal control** based on vehicle presence
- **Manual override/input** via 4x4 Matrix Keypad
- **Countdown timer display** on 4-digit 7-segment LCD
- Supports multiple traffic lanes and light states (Red, Yellow, Green)

---

## 🧰 Components Used

| Component                     | Quantity |
|------------------------------|----------|
| STM32F405RGT6 Microcontroller| 1        |
| Ultrasonic Sensor (HC-SR04)  | 2        |
| Traffic Light Module         | 2        |
| 4-digit 7-segment TM1637 LCD | 2        |
| 4x4 Matrix Keypad            | 1        |
| Jumper Wires & Breadboard    | -        |
| 5V Power Supply              | 1        |

---

## 🛠️ Functional Overview

- **Ultrasonic Sensors** detect the presence and density of vehicles at each signal.
- **STM32** processes sensor input and dynamically adjusts traffic light timing.
- **4x4 Keypad** can be used to:
  - Manually set signal durations
  - Switch modes (Auto/Manual)
  - Trigger pedestrian crossing
- **7-Segment Display** shows the countdown timer for current signal phase.

---

