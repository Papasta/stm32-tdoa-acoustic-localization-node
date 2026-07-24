# STM32 TDOA Acoustic Localization Node

Autonomous node firmware for a distributed acoustic monitoring and sound source localization system based on the Time Difference of Arrival (TDOA) method.

Developed as a graduation thesis at **Moscow State Technical University named after N.E. Bauman (MSTU)**.

---

## Overview

This project implements a low-cost, wireless sensor network for detecting impulsive acoustic events (e.g., gunshots, pyrotechnics) and estimating their 3D position. The system consists of **four slave measurement nodes** and **one master aggregation node**. Each slave node continuously acquires audio from a digital MEMS microphone, detects the first arrival of an acoustic event, timestamps it, and transmits the data to the master. The master collects timestamps, groups them by event ID, and forwards the aggregated data to a localization server via Ethernet.

**Key design constraints:**
- Pure integer arithmetic (no floating-point operations on the MCU)
- Real-time streaming audio processing
- Minimal radio payload (9 bytes per event)
- Sub-millisecond time synchronization between nodes

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Localization Server                   │
│                    (TDOA solver, coordinates)                │
└──────────────────────────┬──────────────────────────────────┘
                           │ Ethernet (TCP)
┌──────────────────────────▼──────────────────────────────────┐
│                    MASTER NODE (1x)                          │
│  STM32F411CEU6 + W5500 + 2× nRF24L01+                        │
│  • Time synchronization broadcaster                          │
│  • Event timestamp collector & grouper                       │
│  • Ethernet gateway to server                                │
└──────┬──────────────────────┬──────────────────────────────┘
       │ Radio Ctrl (100 ms)  │ Radio Events
┌──────▼──────┐  ┌───────────▼────────┐  ┌──────────────────┐
│  SLAVE #1   │  │     SLAVE #2       │  │  SLAVE #3, #4   │
│ INMP441     │  │     INMP441        │  │    INMP441        │
│ STM32F411   │  │     STM32F411      │  │    STM32F411      │
└─────────────┘  └────────────────────┘  └───────────────────┘
```

### Node Roles

| Role | Hardware | Responsibilities |
|------|----------|------------------|
| **Slave** | STM32F411CEU6, INMP441, 2× nRF24L01+ | Audio acquisition, DC removal, envelope extraction, event detection, timestamping, radio TX |
| **Master** | STM32F411CEU6, 2× nRF24L01+, W5500 | Sync broadcast, health polling, event RX, grouping by event ID, Ethernet TX |

---

## Hardware

| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| MCU | STM32F411CEU6 | — | Core processing (ARM Cortex-M4 @ 100 MHz, 512 KB Flash, 128 KB SRAM) |
| Microphone | INMP441 | I2S + DMA | Digital MEMS microphone, 16 kHz, 24-bit samples |
| Radio (Control) | nRF24L01+ | SPI | Master→Slave sync & health check |
| Radio (Events) | nRF24L01+ | SPI | Slave→Master timestamp transmission |
| Ethernet | W5500 | SPI | Hardware TCP/IP stack, 8 sockets, 32 KB buffer |

---

## Signal Processing & Detection Algorithms

The slave node runs a two-stage streaming detector entirely in fixed-point arithmetic:

1. **Pre-processing**
   - 24-bit PCM reconstruction from I2S DMA buffer
   - DC offset removal (recursive IIR filter, shift-based)
   - Scaling to 16-bit and full-wave rectification
   - Exponential moving average envelope (shift-based smoothing)

2. **Event Detection**
   - **Adaptive Threshold**: `Threshold = μ + 4σ`, where μ and σ are recursively estimated background mean and deviation
   - **STA/LTA Ratio**: Short-Term Average / Long-Term Average in **Q8 format** (threshold = 768, i.e., ratio > 3.0)
   - **CUSUM (Cumulative Sum)**: Confirms statistical change; leak factor = 96, threshold = 24000
   - **State Machine**: `QUIET` → `CANDIDATE` → `LOCKOUT` (50 ms lockout prevents re-triggering on reflections)

3. **First Arrival Refinement**
   - **Pre-trigger ring buffer** (256 samples) stores recent envelope history
   - After event confirmation, the timestamp is moved back to the earliest stable onset (4 consecutive samples above soft threshold `μ + 2σ`)

**Timestamp formula:**
```
t_event [μs] = t_start + n_event × (1_000_000 / 16_000)
```

---

## Communication Protocols

### Radio Control Channel (Master → Slaves)
| Packet | Code | Length | Payload |
|--------|------|--------|---------|
| Time Reset | `0xA0` | 9 bytes | zeros |
| Time Sync | `0xA1` | 9 bytes | 8-byte master timestamp (μs, little-endian) |
| Health Check | `0xA2` | 9 bytes | zeros (individual address per slave) |

- **Sync period**: 100 ms (10 Hz)
- **Health check period**: 60 s

### Radio Event Channel (Slave → Master)
| Field | Size | Description |
|-------|------|-------------|
| Timestamp | 8 bytes | Synchronized event time in microseconds |
| Event ID | 1 byte | 8-bit rolling counter |

### Ethernet Frame (Master → Server)
Fixed-length frame containing:
- 4× 8-byte timestamps (one per slave)
- Status flags (0xFF = healthy, 0x00 = missing/unavailable)

---

## Repository Structure

```
├── SLAVE_MODULE/          # Slave node firmware (STM32CubeIDE project)
│   ├── Core/
│   ├── Drivers/
│   └── ...
├── MASTER_MODULE/         # Master node firmware (STM32CubeIDE project)
│   ├── Core/
│   ├── Drivers/
│   └── ...
├── docs/                  # Thesis (RPN), schematics, test reports
└── README.md
```

---

## Build & Flash

Both projects are built with **STM32CubeIDE** (or any GCC ARM toolchain).

1. Open `SLAVE_MODULE` or `MASTER_MODULE` in STM32CubeIDE.
2. Build (`Ctrl+B`).
3. Flash via ST-Link (`F11`).

**Important:** Ensure each slave node is assigned a unique radio address before flashing (see `radio_config.h`).

---

## Field Test Results

The system was validated outdoors using pyrotechnic sound sources ("Korsar K0401") placed at control points on circles of **40 m, 70 m, and 100 m** radius around a tetrahedral sensor base.

| Distance | Avg. Relative Error | Max Absolute Error |
|----------|---------------------|--------------------|
| 40 m     | ~7.95 %             | 3.57 m             |
| 70 m     | ~8.24 %             | 6.28 m             |
| 100 m    | ~8.72 %             | 9.48 m             |

**Requirement met:** Relative localization error ≤ 10 % in the 40–100 m range.

---

## Documentation

- [Thesis (Russian)](docs/) — Full calculation and explanatory note (95 pages)

---

## Author

**Igor E. Leontev**  
Group IU3-81B  
Faculty of Informatics and Control Systems  
Moscow State Technical University named after N.E. Bauman (MSTU)

Supervisor: S.V. Fedorov

---

## License

This project is provided for academic and educational purposes. Commercial use requires permission.
