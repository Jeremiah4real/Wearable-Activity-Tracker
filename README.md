# Wearable Activity Tracker — Efinix Ti60 FPGA

A wearable health monitoring system implemented on the **Efinix Titanium Ti60F225C-DK** FPGA development board. The system integrates three concurrent health monitoring functions: posture analysis, step counting, and fall detection with AI-based visual verification.

This repository contains the firmware source code that runs on the **Sapphire RISC-V soft-core processor** instantiated on the FPGA fabric.

---

## Features

- **Posture Analysis** — Estimates the Craniovertebral Angle (CVA) from a cervical-spine-mounted MPU-6050 IMU. Detects Forward Head Posture (FHP) and drives an LED alert after sustained bad posture.
- **Step Counting** — Uses the Analog Devices V2.1.0 pedometer algorithm adapted for the MPU-6050, with the sensor mounted at the lower back. Step count is displayed in real time over UART (PuTTY terminal).
- **Fall Detection** — A four-state IMU threshold FSM (free-fall → impact → lying-down confirmation) combined with a YOLO-pico person detection model as a vision-based verification layer. A buzzer sounds on confirmed fall events.

---

## Hardware

| Component | Details |
|---|---|
| FPGA Board | Efinix Titanium Ti60F225C-DK |
| IMU (x2) | MPU-6050 (I²C-1: cervical spine, I²C-2: lower back) |
| Camera | Raspberry Pi v2 (8 MP, MIPI CSI-2) |
| Display | Mini-DSI LCD (480×272) |
| Alerts | GPIO LED (posture), GPIO buzzer (fall) |
| Debug | UART → PuTTY serial terminal |

---

## Repository Structure

| File | Description |
|---|---|
| `main.cc` | Top-level firmware loop — orchestrates all subsystems |
| `posture_analysis.c/h` | CVA computation, FHP detection, correction feedback (Algorithm 1 & 2) |
| `fall_detection_imu.c/h` | Fall FSM, complementary filter attitude estimation, accel magnitude |
| `ADI_pedometer_algorithm.c/h` | Analog Devices V2.1.0 step counting library |
| `i2c_two.h` | Custom single-byte-address I²C read function for MPU-6050 |
| `userDef.h` | System peripheral address definitions (I²C, GPIO, SPI, CLINT) |

---

## Dependencies & Tools

### FPGA Toolchain
The HDL design (top module and peripheral configuration) was built using the **Efinity FPGA Toolchain** by Efinix, starting from the **Edge Vision SoC** reference design.

- 📘 [Efinix Edge Vision SoC User Guide (v6.0)](https://www.efinixinc.com/docs/edge-vision-soc-ug-v6.0.pdf)
- 🌐 [Efinix Edge Vision SoC Framework](https://www.efinixinc.com/edge-vision-soc.html)

### TinyML — Person Detection Model
The YOLO-pico person detection model used for fall verification is sourced from the **Efinix TinyML platform**.

- 🤖 [Efinix TinyML Model Zoo — YOLO Person Detection](https://github.com/Efinix-Inc/tinyml/blob/main/model_zoo/README.md#yolo-person-detection)
- 🌐 [Efinix TinyML GitHub Repository](https://github.com/Efinix-Inc/tinyml)

### Firmware
- **RISC-V C/C++ Toolchain** (provided with the Efinity SDK)
- **TensorFlow Lite Micro** (bundled in the Edge Vision SoC framework)

---

## How It Works

The RISC-V firmware runs a continuous main loop:
1. Triggers the hardware accelerator to scale the camera frame (540×540 → 96×96) for TinyML input
2. Runs YOLO-pico inference via TF Lite Micro
3. Polls IMU on I²C-2 (lower back) → step counting
4. Polls IMU on I²C-1 (cervical spine) → attitude estimation, fall FSM, posture state update
5. Drives LED and buzzer outputs; prints step count over UART

The FPGA fabric runs the camera pipeline, DMA controller, hardware accelerator, and display annotator concurrently — independent of the RISC-V core's software execution.

---

## Author

**Jeremiah K*
[github.com/Jeremiah4real](https://github.com/Jeremiah4real)

---

## License

This project is for academic and educational purposes. The ADI pedometer library is subject to Analog Devices' license terms. The Efinix Edge Vision SoC framework and TinyML model zoo are subject to their respective licenses — see the linked repositories above.
