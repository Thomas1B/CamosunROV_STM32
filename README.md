# CamosunROV STM32

This is the repository for the STM32 board used in the Camosun College ROV project. 
It contains the firmware and related resources for the STM32 microcontroller that controls the ROV's motors and sensors.

The STM32 chip used is the STM32F446RE, which is a high-performance ARM Cortex-M4 microcontroller.<br>
Datasheet: [STM32F446RE Datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32f446/documentation.html)

This custom STM32 board is designed to communicate with a Raspberry Pi 4 via UART. 
It receives commands from the Raspberry Pi and controls the ROV's motors and sensors accordingly.

## Features
- Communicates with Raspberry Pi 4 via UART
- Controls 6 motors with automatic cutoff with leak sensor
- Interfaces with various sensors (e.g., Pressure sensor, Temperature sensor)
- Battery Voltage Monitor
- Viewing Lights

<hr>

## Part Notes
- The ESC used in this project expects a max 500Hz PWM signal with the following specs:
	- Stop = 1500us
	- Forward (max) = 1900us
	- Reverse (max) = 1100us

[Afro30A - Datasheet](https://arduino.ua/docs/AfroESC30A.pdf)

If possible, upgrade to [BlueRobotics BasicESC](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/).<br>
Expects the same signal and specs as described above, so no change in firmware.
<hr>

## STM32 Configuration & Pins
These are set in STM32CubeMX.

The HCLK is set to 180MHz.

Programming Pins (SWD on PCB):
| SWD Pin | STM32 Pin | Function | Notes |
|:-------:|:---------:|:--------:|-------|
| 1 | 3V3 | Target Supply Sense | Debug/programming |
| 2 | PA14 | SWCLK | Debug/programming |
| 3 | GND | Ground | Debug/programming |
| 4 | PA13 | SWDIO | Debug/programming |
| 5 | NRST | Reset | Debug/programming |
| 6 | PB3 | SWO (optional) | Trace/debug output |

Pin configuration:
| STM32 Pin | Function | Peripheral / AF | Notes |
|:---:|:--------:|:----------------:|-------|
| PA11 | Motor 1 PWM | TIM1_CH4 | Generates 300Hz Signal |
| PA10 | Motor 2 PWM | TIM1_CH3 | |
| PA9  | Motor 3 PWM | TIM1_CH2 | |
| PA8  | Motor 4 PWM | TIM1_CH1 | |
| PC9  | Motor 5 PWM | TIM8_CH4 | Generates 300Hz Signal |
| PC8  | Motor 6 PWM | TIM8_CH3 | |
| PA6 | Leak Sensor[^1] | TIM8 BKIN | auto terminates TIM8 PWM signals |
| PB12 | Leak Sensor[^1] | TIM1 BKIN | auto terminates TIM1 PWM signals |
| PB15 | Viewing Lights | TIM12_CH2 | Generates 1KHz Signal |
| PA0 | Battery Monitor | ADC1_IN0[^2] | Uses DMA |
| PA0 | Internal Temperature of ROV | ADC1_IN1[^2] | Uses DMA |

[^1]: Pins PA6 and PB12 are physically connected, so if a leak is detected they both detect a HIGH signal.
[^2]: Sampling rate is controlled by TIM2.

