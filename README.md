# stm32-dht22-input-capture

STM32F446ZE firmware project demonstrating two implementations of the DHT22 (AM2302) single-wire protocol:
- **v1.0**: GPIO-driven “bit-banging” using a **free-running 1 MHz timer** for microsecond timing
- **v2.0**: Hardware-assisted decoding using **TIM9 Input Capture** (edge timestamping) with an ISR-driven FSM

---

## Features

- Periodic measurement scheduling using **TIM6** (5 s interval)
- DHT22 protocol handling using:
  - **v1.0**: polling + microsecond timebase (TIM9 free-running counter)
  - **v2.0**: **TIM9 input capture** with polarity toggling and an interrupt-driven finite state machine
- UART output over **USART3** (ST-Link Virtual COM Port)

---

## Hardware / Board

- MCU: **STM32F446ZE** (NUCLEO-F446ZE)
- Sensor: **DHT22 (AM2302)**

### Pin mapping
- **PE5**: DHT22 data line (bidirectional single-wire)
- **USART3**: UART log/output  
  - PD8: TX  
  - PD9: RX

> Note: DHT22 data line should be used with a pull-up resistor (typical single-wire / open-drain behavior).


### Timer configuration
- **TIM6**: periodic trigger (5 s)
- **TIM9**: protocol timing (v1: timebase, v2: input capture)

---

## Architecture (v2.0 input capture)

### Design goals
- Deterministic pulse measurement (microsecond resolution)
- Minimal CPU load (no busy-loop edge timing)
- Clear separation of responsibilities:
  - **main.c**: scheduling and reporting
  - **dht22.c/.h**: sensor driver and protocol decode
  - **clock.c/.h**: clock-query helpers for correct prescaler/baud calculations

### High-level flow
1. **TIM6 interrupt** sets a request flag every 5 s
2. `main()` calls `dht22_start_read()` when requested
3. Start phase:
   - PE5 configured as GPIO output, drives low (~1 ms)
   - PE5 switched to AF3 for TIM9 input capture
4. **TIM9 ISR** timestamps edges and reconstructs the 40-bit frame
5. `main()` validates checksum, decodes to °C / %RH, prints over UART

---

## Releases / Milestones

- **v1.0-bitbang** – Bit-banging with free-running 1 MHz timer  
- **v2.0-input-capture** – TIM9 Input Capture (recommended “current” version)

See the **Releases** page for tagged milestones and notes.

---

## Build & Run

1. Open the project in **STM32CubeIDE**
2. Build (Debug or Release)
3. Flash to the NUCLEO board
4. Open a serial terminal:
   - Baud: **115200**
   - 8N1

---

## What this project demonstrates

- STM32 timer fundamentals: prescaler, counter, capture/compare, interrupts
- ISR design: short deterministic handlers + FSM approach
- Protocol decoding from pulse widths
- Practical driver layering and error reporting

