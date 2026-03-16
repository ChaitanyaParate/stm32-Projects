# STM32 Projects

A collection of embedded systems projects built around the STM32 microcontroller family,
spanning custom bootloaders, real-time multi-threaded systems, motor control, ADC
interfacing, and on-device AI inference.

All firmware is written in C and built with STM32CubeIDE / STM32CubeMX unless noted.

---

## Projects

### BT_program_upload — Bluetooth OTA Bootloader

A custom bootloader for the **STM32G4** series that receives a compiled `.bin` binary
wirelessly over USART2 via an **HC-05 Bluetooth module**.

**How it works:**
1. On reset, the bootloader runs and waits for an incoming `.bin` over USART2.
2. The received binary is buffered in RAM.
3. The bootloader erases and re-flashes the application region starting at `0x08008000`.
4. Execution jumps to the application's reset handler.

**Key peripherals:** USART2, HC-05 (UART-Bluetooth bridge), Flash HAL

---

### BMS_VCU / freeRTOS_DMA_try — Multi-Threaded BMS/VCU Braking System

A **FreeRTOS**-based multi-threaded firmware for a Battery Management System and Vehicle
Control Unit. Designed for a high-reliability braking system with multiple concurrent
sensor and communication tasks.

**Peripherals used:**
- UART4, USART2 (serial comms)
- ADC1, ADC2 (pressure sensing)
- FDCAN3 (CAN bus communication)
- USB (data logging or host interface)
- GPIO (actuator control, status signaling)

**Architecture:** Each subsystem (pressure monitoring, CAN TX/RX, UART logging) runs as
an independent FreeRTOS task with defined priorities.

---

### F1_Inverter_Final — Inverter Health Monitoring & Control (Hyperloop)

STM32-based firmware for driving and monitoring a 3-phase inverter in a Hyperloop pod
drivetrain.

**Features:**
- Generates **3 PWM signals + 3 complementary (inverted) PWM signals** via **TIM1**
  for 3-phase inverter gate control
- PWM frequency controls motor speed; duty cycle controls amplitude (torque/force)
- Inverter health monitoring with fault detection
- Status reporting and fault signaling via GPIO and UART

---

### Aeron Polux 2.1

STM32 firmware developed for the **Aeron Polux** Hyperloop pod (version 2.1). Exact
subsystem details are folder-level; see source files inside `Aeron Polux 2.1/`.

---

### multichannel_adc — Multi-Channel ADC Sampling

Demonstrates simultaneous multi-channel ADC acquisition on STM32, likely using DMA for
continuous non-blocking reads.

**Use case:** Sensor fusion, analog data aggregation.

---

### Soil_irrigation_prediction / first_AI — On-Device AI Inference

An early attempt at running an **AI/ML inference model** on STM32 for soil moisture-based
irrigation prediction. Uses sensor inputs to decide irrigation triggers without a host
processor.

---

## Hardware Requirements

| Project               | MCU            | Key Modules            |
|-----------------------|----------------|------------------------|
| BT_program_upload     | STM32G4        | HC-05 Bluetooth        |
| BMS_VCU               | STM32 (F/G4)   | CAN transceiver, USB   |
| F1_Inverter_Final     | STM32          | SD card, motor driver  |
| Aeron Polux 2.1       | STM32          | Pod-specific hardware  |
| multichannel_adc      | STM32          | Analog sensors         |
| Soil_irrigation_pred  | STM32          | Soil moisture sensor   |

---

## Toolchain

- **IDE:** STM32CubeIDE
- **HAL:** STM32 HAL / LL drivers
- **RTOS:** FreeRTOS (BMS_VCU project)
- **Language:** C (C99)
- **Programmer:** ST-LINK / custom BT bootloader

---

## Getting Started

1. Clone the repo:
```bash
   git clone https://github.com/ChaitanyaParate/stm32-Projects.git
```
2. Open the desired project folder in **STM32CubeIDE** (each folder is a self-contained
   CubeIDE project).
3. Build via `Project > Build All` or `Ctrl+B`.
4. Flash using ST-LINK or, for `BT_program_upload`, use the Bluetooth bootloader flow.

---

## Author

**Chaitanya Parate**  
B.Tech Computer Science, MIT World Peace University, Pune  
[GitHub](https://github.com/ChaitanyaParate)