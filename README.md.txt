# SERVO_Control

**SERVO_Control** is an embedded systems project for smooth dual-axis servo control using an STM32F4 microcontroller running FreeRTOS. The system processes input from a two-axis joystick via two independent ADCs (with DMA) and communicates with external devices via UART. A Python-based GUI is also provided for testing and manual control of the servos.

---

## Table of Contents

- [Overview](#overview)
- [Module Descriptions](#module-descriptions)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [Build Instructions](#build-instructions)
- [Usage Instructions](#usage-instructions)
- [Future Work](#future-work)
- [License](#license)
- [Contributing](#contributing)
- [Contact](#contact)

---

## Overview

The SERVO_Control project implements dual-axis servo control using adaptive S-curve motion profiles. The firmware is built using FreeRTOS and is organized into separate tasks for joystick processing, servo control, UART communication, and LED debugging. The system achieves smooth motion by processing joystick ADC values with filtering and dead-zone logic and then generating adaptive S-curves with double buffering. A Python GUI provides a user-friendly interface for connecting via UART, sending servo commands, and viewing debug output.

---

## Module Descriptions

- **Joystick Task:**  
  Reads ADC data from two independent ADCs (X axis via ADC1 and Y axis via ADC2) using DMA. It applies averaging, exponential low-pass filtering, and dead-zone logic, then sends combined events to the servo task.

- **Servo Task:**  
  Receives events from both the joystick and UART tasks and generates adaptive S-curves for smooth servo motion. It uses double buffering to update PWM compare values safely from a high-priority PWM ISR.

- **UART Task:**  
  Implements a state-machine parser to decode incoming UART data packets and sends corresponding commands to the servo task for manual control.

- **LED Task:**  
  Blinks an LED at a fixed interval for debugging purposes, indicating system activity.

- **Python GUI & UART API:**  
  Provides a graphical interface (using Tkinter) for UART communication. The UART API module handles serial communication, including framing for sending commands and receiving data from the embedded system.

---

## Features

- **Dual-Axis Joystick Input:**  
  Uses two ADCs (with DMA) to read the X and Y axes separately, then processes these values with filtering and dead-zone logic.

- **Adaptive S-Curve Motion:**  
  Generates smooth S-curve profiles for servo motion using double buffering to avoid race conditions.

- **UART Communication:**  
  Implements a state-machine-based UART task on the MCU and a Python-based GUI for testing and control.

- **RTOS-Based Architecture:**  
  FreeRTOS tasks ensure time-critical operations (servo control) are prioritized, with separate tasks for input processing, communication, and debugging.

- **Modular Design:**  
  Code is split into independent modules (joystick, servo, UART, LED) to simplify maintenance and future expansion.

---

## Hardware Requirements

- **Microcontroller:** STM32F4 series (with FreeRTOS)
- **Joystick:** Two-axis analog joystick
- **Servos:** Two standard hobby servos (for pan and tilt control)
- **UART Interface:** For communication with the Python GUI (e.g., via an ST-Link or USB-to-UART adapter)
- **LED:** For debugging (onboard or external)

---

## Software Requirements

- **Firmware:**
  - STM32CubeMX / STM32CubeIDE (for peripheral configuration)
  - FreeRTOS
  - CMake (build configuration)
  - ARM GCC Toolchain
  - OpenOCD (for flashing and debugging)

- **Host Application (Python):**
  - Python 3.x
  - [pyserial](https://pypi.org/project/pyserial/)
  - Tkinter (for GUI)
  - (Optional) Other modules as specified in `requirements.txt`

---

## Project Structure

## Build Instructions

### Firmware Build (STM32F4)
1. **Clone the Repository:**
   ```bash
   git clone https://github.com/<USERNAME>/SERVO_Control.git
   cd SERVO_Control

2.Configure and Build with CMake:


bash
Copy
mkdir build && cd build
cmake ..
make

3.Flash the Firmware

bash
Copy
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
arm-none-eabi-gdb your_firmware.elf
(gdb) target remote localhost:3333	



Python GUI Application
Install Dependencies:

bash
Copy
pip install pyserial
Run the GUI:

bash
Copy
python gui.py

	