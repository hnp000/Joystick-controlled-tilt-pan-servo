

## About the Project
**SERVO_Control** implements real-time dual-axis servo control with:
- STM32F4 microcontroller running FreeRTOS
- Dual ADC (with DMA) for joystick input
- Adaptive S-curve motion profiling
- UART communication with Python GUI



---

## Key Features

 Dual-Axis Input: Two ADCs with DMA for low-latency sampling 
 Motion Control: Jerk-limited S-curve trajectories 
 Communication: Custom UART protocol with Python GUI 
 Real-Time: FreeRTOS tasks with priority scheduling 
 python GUI app: To control servo from gui

---

## Getting Started

### Hardware used

MCU: STM32F446
Joystick: Analog 2-axis 
Servos: Standard hobby tilt-pan servo

### Software Dependencies
- STM32CubeIDE and VSCode
- ARM GCC Toolchain
- CMAKE
- MAKE
- OpenOCD (for flashing)
- python


## How to use this repositary

1. Clone repository:


git clone https://github.com/hnp000/Joystick-controlled-tilt-pan-servo.git


2. Build with CMake:
mkdir build && cd build
cmake -B build -G 'Unix Makefiles'" ..
cmake build

3. Flash to device:
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program MyApp.elf verify reset exit"

## Usage
1. Joystick Control: 
    Power on the system. Move joystick and  servos will respond in real-time

2. Python GUI:
    - Open python gui from python_app folder by exucuting this comman: python gui.py
    - Connect to specific PORT (COM) and baudrate is 115200
    - now give nay value to servo1(base) and/or servo2(tilt) in degree to move servo
