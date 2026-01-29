# STM32 DHT11 Bit-Banging Driver

[![Platform](https://img.shields.io/badge/Platform-STM32H5-003399)](https://www.st.com/en/microcontrollers-microprocessors/stm32h5-series.html)
[![Language](https://img.shields.io/badge/Language-C-A8B9CC)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Category](https://img.shields.io/badge/Category-Bare_Metal_Driver-green)]()

## Project Overview
This project implements a **custom bit-banging driver** for the DHT11 temperature and humidity sensor on the STM32H503RB Nucleo-64 development board. The driver is split into a dedicated module (`DHT11_Driver.c/.h`) and is called from `main.c` to read the sensor and log values over UART.

Unlike typical library-based implementations, this driver manually handles the **single-wire proprietary protocol** at the microsecond level. It demonstrates precise timing control using TIM1, direct GPIO manipulation, and checksum verification for data integrity.

## Hardware & Technical Stack
*   **MCU**: STM32H503RB (Nucleo-64)
*   **Sensor**: DHT11 (Temperature and Humidity)
*   **Interface**: Single-Wire Serial Communication (Bit-Banging)
*   **Microsecond timing** with TIM1 at 1 MHz tick (prescaler 249 for a 250 MHz clock tree).
*   **Logging Output**: USART3 UART @ 115200 baud for sensor data and error reporting
*   **CMake integration** so the driver builds as part of the project target.

---

## Driver Source (Authored)
*   [`Core/Src/DHT11_Driver.c`](Core/Src/DHT11_Driver.c)
*   [`Core/Inc/DHT11_Driver.h`](Core/Inc/DHT11_Driver.h)

## Driver API (Quick Use)
```c
#include "DHT11_Driver.h"

int temp = 0;
int hum = 0;

if (DHT11_Read_Data(DHT11_Data_GPIO_Port, DHT11_Data_Pin, &temp, &hum) == 0) {
    // Success: temp and hum updated
} else {
    // Error: timeout or checksum failure
}
```

Return codes:
*   `0` = success
*   `-1` = timeout or checksum failure

---

## Technical Deep Dive

### 1. Protocol Implementation (State Machine)
The DHT11 uses a strict timing-based protocol. This driver implements the half-duplex communication sequence manually:

1.  **Start Signal**: The MCU pulls the data line LOW for 18 ms to wake up the sensor.
2.  **Handshake**: The MCU releases the line and waits for the sensor to pull it LOW (80 us) then HIGH (80 us).
3.  **Data Transmission**: The sensor sends 40 bits of data.
    *   **Logic 0**: 50 us LOW -> ~26-28 us HIGH
    *   **Logic 1**: 50 us LOW -> ~70 us HIGH
4.  **Stop Signal**: Release bus.

<img width="800"  alt="Diagram DHT11" src="https://github.com/user-attachments/assets/d90e5530-fc06-42d9-bc2d-fe66a1d1893d" />


### 2. Precise Timing Control
Standard `HAL_Delay()` only provides millisecond resolution, which is insufficient for the DHT11 protocol (20 us - 80 us tolerances). I address this by configuring **TIM1** as a microsecond counter:
```c
static void delay_microseconds(uint16_t microseconds) {
    htim1.Instance->CNT = 0;
    while (htim1.Instance->CNT < microseconds) {
        // Precise blocking delay based on hardware timer
    }
}
```

### 3. Bit Logic & Signal Capture
The driver reads the length of the voltage HIGH pulse to distinguish between '0' and '1'.
```c
// Shift left to prepare for new LSB
current_byte <<= 1;

// Wait 40 us (safely past the '0' threshold of 28 us)
delay_microseconds(40);

// Sample the pin
if (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin) == GPIO_PIN_SET) {
    current_byte |= 1; // Pulse is still HIGH, so it must be a '1' (~70 us)
}
// Else: Pulse would be LOW if it was a '0' (~28 us)
```

### 4. Data Validation
To ensure data integrity, the driver implements the checksum verification native to the DHT11. This prevents erroneous readings due to noise or timing violations.
*   **Formula**: `(Integral_RH + Decimal_RH + Integral_T + Decimal_T) == Checksum`

---

## Getting Started

### Hardware Connections
| STM32 Pin | DHT11 Pin | Description |
| :--- | :--- | :--- |
| **PC8** | Data | Data Signal (requires a pull-up if the module has none) |
| **5V/3.3V** | VCC | Power |
| **GND** | GND | Ground |

### Build Instructions
1.  Open the project in **STM32CubeIDE** (or build with CMake).
2.  Ensure **TIM1** is configured with Prescaler `249` (assuming 250 MHz clock -> 1 MHz tick) or adjust for your clock tree.
3.  Build and flash to the Nucleo-H503RB.
4.  Open a Serial Terminal:
    *   **Baud**: 115200
    *   **Parity**: None, **Stop Bits**: 1

## File Structure
*   `Core/Src/main.c`: Initializes peripherals and calls `DHT11_Read_Data`, then logs results over UART. (Visualize in Putty)
*   `Core/Inc/main.h`: Pin definitions and macros.
*   `Core/Src/DHT11_Driver.c`: DHT11 protocol implementation and timing.
*   `Core/Inc/DHT11_Driver.h`: Driver API and function prototype.

## Datasheets
*[DHT11 Datasheet](https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)*
<br>
*[STM32H503RB Nucleo-64 User Manual](https://www.st.com/resource/en/user_manual/um3121-stm32h5-nucleo64-board-mb1814-stmicroelectronics.pdf)*

---
*Anthony*
