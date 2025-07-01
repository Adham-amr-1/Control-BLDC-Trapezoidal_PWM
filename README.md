# 🚗 E-RALLY V1 – Trapezoidal PWM BLDC Motor Controller (STM32 Blue Pill)

This project presents a **Trapezoidal PWM (6-step commutation) control algorithm** for a **3-phase BLDC motor**, implemented on the **STM32F103C6T6** microcontroller (commonly known as the "Blue Pill"). It is part of the embedded system powering the **E-RALLY electric vehicle prototype**.

Designed to deliver real-time control of brushless motors, the firmware combines **Hall-effect sensor feedback**, **safe commutation logic**, **PWM signal generation**, and **filtered analog throttle input** to control motor torque and speed safely and efficiently.

---

## 👨‍💻 Project Contributors

Developed by the Embedded Systems Team of the E-RALLY competition vehicle:

- **Adham Amr** – PWM interfacing, Motor control logic, system integration
- **Nourhan Abdelnabi** – PWM and sensor interfacing, testing  
- **Amira Muhamed** – PWM interfacing, Code debugging, peripheral configuration  

---

## 🧠 Technical Overview

- **Microcontroller**: STM32F103C6T6 (ARM Cortex-M3, 72 MHz)
- **Motor Type**: 3-phase Brushless DC (BLDC)
- **Control Strategy**: Trapezoidal PWM (6-step commutation)
- **Sensor Feedback**: 3 digital Hall-effect sensors
- **PWM Generation**: TIM1 with complementary outputs (high and low side)
- **Throttle Input**: Analog signal via ADC1 (filtered using moving average)
- **Safety**: Dead-time insertion, invalid state protection, soft disable

---

## 🔍 Features

- ✅ Accurate Hall sensor decoding for rotor position detection  
- ✅ ADC sampling and filtering for smooth throttle response  
- ✅ Safe state transitions with dead-time to prevent shoot-through  
- ✅ Macros and modular functions for maintainability and readability  
- ✅ Diagnostic variables to help with debugging and validation  

---

## 📦 Directory Structure

```
E-RALLY_V1/
├── Core/
│ ├── Inc/ # Header files (main.h, user-defined modules)
│ ├── Src/ # Source files (main.c, commutation logic)
│ └── Startup/ # Startup code and vector table
├── Drivers/
│ ├── STM32F1xx_HAL_Driver # STM32 HAL driver files
│ └── CMSIS/ # ARM Cortex-M3 core support files
├── Debug/ # Compiled binaries and debug files
├── E-RALLY_V1.ioc # STM32CubeMX project configuration
├── .project, .cproject # STM32CubeIDE project files
└── STM32F103C6TX_FLASH.ld # Linker script
```

---


---

## 🧩 Pin Configuration

| Function           | STM32 Pin       | Description                          |
|--------------------|------------------|--------------------------------------|
| Hall Sensor A      | PA15             | Rotor feedback (bit 0)               |
| Hall Sensor B      | PA12             | Rotor feedback (bit 1)               |
| Hall Sensor C      | PA11             | Rotor feedback (bit 2)               |
| Phase A High       | PA8 (TIM1_CH1)   | High-side PWM for Phase A           |
| Phase B High       | PA9 (TIM1_CH2)   | High-side PWM for Phase B           |
| Phase C High       | PA10 (TIM1_CH3)  | High-side PWM for Phase C           |
| Phase A Low        | PA7 (TIM1_CH1N)  | Low-side PWM for Phase A            |
| Phase B Low        | PB0 (TIM1_CH2N)  | Low-side PWM for Phase B            |
| Phase C Low        | PB1 (TIM1_CH3N)  | Low-side PWM for Phase C            |
| Throttle Input     | PA0 (ADC1 IN0)   | Analog input (0–5V) mapped to PWM |

---

## 🛠️ Build & Flash Instructions

### ✅ Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- ST-Link V2 Programmer
- BLDC motor with Hall sensors
- 3-phase MOSFET driver (e.g., IR2101)

### 🧰 Setup Steps

1. **Clone or unzip** the project into your STM32CubeIDE workspace.
2. Open `E-RALLY_V1.ioc` in STM32CubeIDE to examine pin configuration and middleware.
3. Build the project (`Project > Build All`).
4. Connect the STM32 via ST-Link and flash the firmware (`Run > Debug`).
5. Supply motor and driver with appropriate power (12V–48V, depending on the motor).

---

## ⚙️ Control Logic Summary

### 📈 Throttle Mapping

- Reads analog voltage from the throttle
- Applies a moving average filter (buffer size: 254)
- Maps ADC result to PWM duty (range 0 to 3599)

### 🔄 Hall Sensor Interpretation

- Combines 3 inputs into a 3-bit value
- Matches value to 1 of 6 valid commutation states
- Calls corresponding `StepX()` function to energize 2 phases

### 🔒 Safety Features

- **Dead Time**: Short delay between switching phases to prevent overlap
- **Invalid Hall State**: Shuts off all phases if a wrong combination is detected
- **State Change Handling**: Uses `CheckState()` to safely disable phases before transitions

---

## 📚 References & Documentation

- [STM32F103C6 Datasheet – STMicroelectronics](https://www.st.com/resource/en/datasheet/stm32f103c6.pdf)

---

## 📜 License

This software is distributed with STM32Cube auto-generated content and custom application logic. The autogenerated content is governed by STMicroelectronics’ license (found in `/Drivers`). All custom code is provided **as-is** for educational and prototyping purposes.

---




