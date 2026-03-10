# STM32 Mini Mecanum Wheel Robot
<img src="mini-mechanum-wheel-robot.jpg" alt="Mecanum Wheel Robot" width="500"/>
A mini mecanum wheel robot built around the **STM32F407VGT6**, featuring closed-loop motor speed PID control and orientation PID for robust omnidirectional movement — including a full **Field-Oriented Control (FOC)** drive mode.

---

## ✨ Features

- **Omnidirectional Drive** — Full mecanum wheel holonomic movement (forward, lateral, diagonal, rotation)
- **Motor Speed PID** — Per-wheel encoder feedback ensures balanced wheel speeds for smooth, accurate motion
- **Orientation PID (Yaw correction)** — Overcomes the skidding issue where the robot body drifts or slips in an unintended direction, using the IMU heading to actively correct course
- **Absolute Field-Oriented Control (FOC) Mode** — The robot moves in the direction you command relative to the *field/world frame*, regardless of the robot's current heading
- **On-board Display** — Real-time telemetry and PID menu on a 1.3" ST7789 LCD
- **Dual MCU Architecture** — STM32 handles motor control and kinematics; ESP32 handles wireless communication

---

## 🎬 Demo Videos

> click the number to watch the demo video

| Video | Description |
|---|---|
| [`1`](https://youtu.be/DZKMX0H58ZI?si=J2i-f-OPjciT31iD) | Yaw PID orientation correction in action |
| [`2`](https://youtu.be/A75sVBpRvhQ?si=4BdPLSrhaBGgYdHK) | Absolute field-oriented control demo |
| [`3`](https://youtu.be/tIPLqOVqF0E?si=LInQARhww9oBqlkp) | Circular movement without PID (drift visible)|
| [`4`](https://youtu.be/zL731xuKqPI?si=oEz85-FffTZKwK0l) | Circular movement without PID |
| [`5`](https://youtu.be/0gELg2SBPxo?si=uxx6zX-W0JFL4s1f) | Lateral movement without PID (drift visible) |
| [`6`](https://youtu.be/wuhcH3w_rd0?si=oFXOrNfrPluLR-e1) | Lateral movement with orientation PID active |
| [`7`](https://youtu.be/lcQUk2T1LM4?si=rOg1BIaivM7QJVvr) | On-board PID tuning menu on LCD |

---

## 🔧 Hardware

| Component | Details |
|---|---|
| **Microcontroller** | STM32F407VGT6 |
| **Wireless Module** | ESP32 |
| **IMU** | BNO085 (rotation vector / absolute orientation) |
| **Motor Driver** | TB6612FNG module |
| **Motors** | 4× TT Motor with encoder |
| **Wheels** | 68mm plastic mecanum wheels |
| **Display** | 1.3" LCD (ST7789 driver) |
| **Battery** | 2S 18650 Li-ion |

---

## 🧠 Control Architecture

```
         ┌─────────────────────────────────────────────┐
         │                STM32F407VGT6                │
         │                                             │
 ESP32 ──►  Joystick Input / Drive Mode Select        │
 BNO085 ──►  Yaw / Orientation Feedback               │
         │           │                                 │
         │    ┌──────▼──────┐                          │
         │    │ Orientation │  Yaw PID correction      │
         │    │    PID      │                          │
         │    └──────┬──────┘                          │
         │           │                                 │
         │    ┌──────▼──────┐                          │
         │    │  Mecanum    │  Inverse kinematics      │
         │    │  Kinematics │  (Normal / FOC mode)     │
         │    └──────┬──────┘                          │
         │           │  ω target per wheel             │
         │    ┌──────▼──────┐                          │
         │    │  Motor PID  │  Speed PID × 4 wheels    │
         │    └──────┬──────┘                          │
         └───────────┼─────────────────────────────────┘
                     │
              TB6612 Motor Driver
                     │
          ┌──┬───────┴───────┬──┐
         M1  M2             M3  M4
```

### Drive Modes

**Normal Omnidirectional Mode**
Robot translates and rotates based on raw joystick input in the robot's own body frame.

**Absolute Field-Oriented Control (FOC) Mode**
Using the BNO085's absolute heading, joystick commands are transformed into the world/field frame. The robot always moves in the intended real-world direction regardless of where its nose is pointing — ideal for remote control without needing to track robot heading manually.
