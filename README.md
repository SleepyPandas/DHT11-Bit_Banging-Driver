# STM32 DHT11 Bit-Banging Driver

[![Platform](https://img.shields.io/badge/Platform-STM32H5-003399)](https://www.st.com/en/microcontrollers-microprocessors/stm32h5-series.html)
[![Language](https://img.shields.io/badge/Language-C-A8B9CC)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Category](https://img.shields.io/badge/Category-Bare_Metal_Driver-green)]()

## Project Overview
This project implements a **custom bit-banging driver** for the DHT11 temperature and humidity sensor on the STM32H503RB Nucleo development board.

Unlike typical library-based implementations, this driver manually handles the **single-wire proprietary protocol** at the microsecond level. It demonstrates precise timing control using hardware timers (TIM1), direct GPIO manipulation, and rigorous signal validation (checksum verification). This project serves as a demonstration of **low-level embedded C programming** and **protocol validation** techniques.

## Hardware & Technical Stack
*   **MCU**: STM32H503RB (Nucleo-64)
*   **Sensor**: DHT11 (Temperature & Humidity)
*   **Interface**: Single-Wire Serial Communication (Bit-Banging)
*   **Timing Reference**: TIM1 (Hardware Timer) for µs-precision delays
*   **Validation Output**: UART (115200 baud) for sensor data and error logging

---

## Technical Deep Dive

### 1. Protocol Implementation (State Machine)
The DHT11 uses a strict timing-based protocol. This driver implements the half-duplex communication sequence manually:

1.  **Start Signal**: The MCU pulls the data line LOW for exactly 18ms to wake up the sensor.
2.  **Handshake**: The MCU releases the line and waits for the Sensor to pull it LOW (80µs) then HIGH (80µs).
3.  **Data Transmission**: The sensor sends 40 bits of data.
    *   **Logic 0**: 50µs LOW -> ~26-28µs HIGH
    *   **Logic 1**: 50µs LOW -> ~70µs HIGH
4.  **Stop Signal**: Release bus.

### 2. Precise Timing Control
Standard `HAL_Delay()` only provides millisecond resolution, which is insufficient for the DHT11 protocol (20µs - 80µs tolerances).
I addressed this by configuring **TIM1** as a microsecond counter:
```c
void delay_microseconds(uint16_t microseconds) {
    htim1.Instance->CNT = 0;
    while (htim1.Instance->CNT < microseconds) {
        // Precise blocking delay based on hardware timer
    };
}
```

### 3. Bit Logic & Signal capture
The driver reads the length of the voltage HIGH pulse to distinguish between '0' and '1'.
```c
// Shift left to prepare for new LSB
current_byte <<= 1;

// Wait 40us (safely past the '0' threshold of 28us)
delay_microseconds(40);

// Sample the pin
if (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin) == GPIO_PIN_SET) {
    current_byte |= 1; // Pulse is still HIGH, so it must be a '1' (70us)
}
// Else: Pulse would be LOW if it was a '0' (28us), leaving the LSB as 0
```

### 4. Data Validation (Post-Silicon Methodology)
To ensure data integrity, the driver implements the checksum verification native to the DHT11. This prevents erroneous readings due to noise or timing violations.
*   **Formula**: `(Integral_RH + Decimal_RH + Integral_T + Decimal_T) == Checksum`

---




## 🚀 Getting Started

### Hardware Connections
| STM32 Pin | DHT11 Pin | Description |
| :--- | :--- | :--- |
| **PC8** | Data | Data Signal (Requires Pull-up if module has none) |
| **5V/3.3V** | VCC | Power |
| **GND** | GND | Ground |

### Build Instructions
1.  Open project in **STM32CubeIDE**.
2.  Ensure **TIM1** is configured with Prescaler `249` (assuming 250MHz clock → 1MHz tick) or adjusted for your clock tree.
3.  Build and Flash to Nucleo-H503RB.
4.  Open Serial Terminal:
    *   **Baud**: 115200
    *   **Parity**: None, **Stop Bits**: 1

## 📂 File Structure
*   `Core/Src/main.c`: Contains the `DHT11_Read_Data` driver and main application loop.
*   `Core/Inc/main.h`: Pin definitions and macros.

---
*Anthony*
