# CamosunROV STM32

This is the repository for the STM32 board used in the CamosunROV project.
It contains the firmware and related resources for the STM32 microcontroller that controls the ROV's motors and sensors.

The STM32 chip used is the STM32F446RE, which is a high-performance ARM Cortex-M4F microcontroller.<br>
Datasheet: [STM32F446RE Datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32f446/documentation.html)

This custom STM32 board communicates with a Raspberry Pi 4 over UART (UART4, DMA in both directions).<br>
The Pi sends thruster and camera-tilt commands at 50 Hz; the STM32 applies them and sends sensor telemetry back at 50 Hz.

Built with STM32CubeIDE 2.2.0 / STM32CubeMX 6.18.1, firmware package **STM32Cube FW_F4 V1.28.3**.

<hr>

# TODO:
- [ ] Test I2C Water Temperature Sensor firmware (waiting for sensor to arrive)
- [ ] Test Recovery (restarts PWM generation, ADC reads, re-enables BKIN interrupts, etc.)
- [ ] Find each ESC's deadband range (all motors still use the default 1450 µs / 1550 µs onset values)
- [ ] Full test of hardware prototype using Pilot UI
- [ ] Full Test on Custom on PCB
- [x] Test status LEDs on hardware (lamp test, boot sequence, each pattern in the LED table)
- [x] Finish Emergency Shutdown Function
- [x] UART to/from Pi
- [X] Live I2C sensor fault detection + 1 s retry (code done)
- [x] Live fault flags to the Pi (faults clear when they recover; only LEAK stays set)
- [x] Test camera tilt servo on hardware

<hr>

## Features
- Communicates with Raspberry Pi 4 via UART4 (115200 bit/s, 8N1, DMA RX with idle-line detection + DMA TX)
- Controls 6 thrusters with automatic hardware cutoff on leak (TIM1/TIM8 break input)
- Camera tilt servo (MG90S)
- Leak detection (hardware break input, latched fault)
- Command timeout failsafe: thrusters stop if no valid command for 150 ms
- Sensors:
	- Depth and absolute pressure — Blue Robotics Bar30 (MS5837-30BA)
	- Water temperature — Blue Robotics Celsius (TSYS01)
	- Orientation (heading, roll, pitch) — BNO055 IMU in NDOF mode
	- Enclosure temperature — NTC thermistor on ADC
	- Battery voltage — resistor divider on ADC
- Live fault flags reported to the Pi in every telemetry frame (clear when the fault recovers; LEAK stays set)
- Status LEDs (red / yellow / green)
- SWO debug output (`printf` → ITM) every 1.5 s

<hr>

## Project Layout
| Folder | Contents |
|--------|----------|
| `Core/Src/main.c` | Boot sequence, main loop, fault handling, sensor reads, telemetry packing |
| `Core/UART_DMA/` | Pi ↔ STM32 serial protocol (frame building/parsing, checksums, DMA RX/TX). Keep in sync with `UART_DMA.py` on the Pi |
| `Core/Motors/` | Thruster ESC and camera-tilt servo PWM control, per-motor calibration constants |
| `Core/Bar30/` | MS5837 (Bar30) pressure/depth driver |
| `Core/TSYS01/` | TSYS01 (Celsius) water temperature driver |
| `Core/BNO055/` | BNO055 IMU driver |
| `Core/Thermistor/` | NTC thermistor resistance → temperature (Steinhart-Hart) |
| `Core/MyUtils/` | Helpers: value mapping, ADC averaging, fault-flag checks; status LED patterns (`status_led.c`) |

<hr>

## UART Protocol (Pi ↔ STM32)
UART4, 115200 bit/s, 8 data bits, no parity, 1 stop bit. All multi-byte values are little-endian.<br>
Start byte = `0xAA`. Checksum = sum of all bytes between START and CHECKSUM, mod 256.<br>
Full details are in `Core/UART_DMA/UART_DMA.h`.

**Pi → STM32 command (11 bytes, 50 Hz):**
| Byte(s) | Field | Type | Units / Range |
|:---:|-------|:---:|-------|
| 0 | START | u8 | `0xAA` |
| 1–6 | motor1 … motor6 | i8 | % throttle, −100 … 100 |
| 7–8 | cam_tilt_dc | i16 | camera tilt ×10 (deci-degrees), 0 = level, + = up |
| 9 | recover | u8 | 0 = normal, 1 = clear a leak shutdown (see below) |
| 10 | CHECKSUM | u8 | |

**Pi → STM32 reference (10 bytes, only when the values change):** the Bar30 depth settings from the command centre (CONNECT dialog / File ▸ Reference Values…). The Pi sends each new set 3 times, about 10 ms after a command frame, and again whenever the STM32 reconnects.
| Byte(s) | Field | Type | Units / Range |
|:---:|-------|:---:|-------|
| 0 | START_REF | u8 | `0xAB` (commands use `0xAA`) |
| 1–4 | surface_hpa_x100 | u32 | surface pressure reference, hPa (= mbar) ×100, 80000 … 110000 |
| 5–6 | density_x10 | u16 | water density, kg/m³ ×10, 9000 … 11000 |
| 7 | flags | u8 | bit 0 = second-order calibration on |
| 8 | ref_id | u8 | +1 each time the values change (the repeats share it) |
| 9 | CHECKSUM | u8 | |

Out-of-range values are rejected even with a good checksum. A valid frame is stored by the UART interrupt and applied by the main loop (`apply_reference()`), replacing the `SURFACE_PRESSURE` / `WATER_DENSITY` / `second_order_calculation` boot defaults. Reference frames are not commands: they don't reset the link timeout and aren't counted as lost commands.

**STM32 → Pi telemetry (20 bytes, 50 Hz):**
| Byte(s) | Field | Type | Units |
|:---:|-------|:---:|-------|
| 0 | START | u8 | `0xAA` |
| 1–2 | pressure_x10 | u16 | kPa ×10, absolute (≈ 1013 at the surface) |
| 3–4 | depth_x100 | i16 | m ×100 |
| 5–6 | water_temp_x100 | i16 | °C ×100 |
| 7–8 | battery_x100 | u16 | V ×100 |
| 9–10 | inside_temp_x100 | i16 | °C ×100 |
| 11–12 | heading_x100 | u16 | ° ×100 (0 … 36000) |
| 13–14 | roll_x100 | i16 | ° ×100 |
| 15–16 | pitch_x100 | i16 | ° ×100 |
| 17 | leak | u8 | 0 = dry, 1 = water detected now |
| 18 | fault_flags | u8 | fault bits active now (see below) |
| 19 | CHECKSUM | u8 | |

Values outside the field range are clamped by the sender (i16 ×100: −327.68 … 327.67, u16 ×100: 0 … 655.35, u16 ×10: 0 … 6553.5).

- `pressure_x10` is the raw Bar30 reading, **not** minus the surface reference (that is applied to `depth_x100` only). It is scaled ×10 instead of ×100 because the Bar30 reads up to ~3000 kPa, which would overflow 16 bits at ×100. 0.1 kPa = 100 Pa ≈ 1 cm of water, the same resolution as depth.
- The Pi side (`UART_DMA.py`) must expect the 20-byte frame; an 18-byte parser will fail every checksum.

**Fault flags** (live: a bit clears when its fault recovers, except LEAK, which stays set until a recover command. While a sensor's bit is set, its telemetry field holds its last good value and should be ignored):
| Bit | Value | Fault |
|:---:|:---:|-------|
| 0 | `0x01` | LEAK |
| 1 | `0x02` | IMU (BNO055 not responding, or reset itself) |
| 2 | `0x04` | DEPTH (Bar30 not responding) |
| 3 | `0x08` | TEMP (TSYS01 not responding) |
| 4 | `0x10` | UART (Pi link lost: command timeout after link was established) |

<hr>

## Leak / Emergency Shutdown
- The leak sensor drives the TIM1 and TIM8 break inputs (PB12, PA6, active HIGH). On a leak the hardware immediately forces all six thruster PWM outputs off — no software involved.
- The break ISR only latches `SYS_STATE_FAULT_LEAK` and disables further break interrupts. `emergency_shutdown()` in the main loop then stops everything except the status LEDs and UART4: thruster and camera-servo PWM stopped and pins held LOW, ADC + TIM2 stopped, I2C1 de-initialized, break and ADC DMA interrupts disabled. Red LED solid (LEAK pattern).
- Telemetry keeps running at 50 Hz so the Pi sees `leak = 1` and the LEAK fault flag. Sensor fields are frozen at their last reading (0 if the leak was present at boot).
- Motor and tilt values from the Pi are ignored during a leak.
- If a leak is present at boot, the board skips sensor start-up and goes straight to the shutdown state (UART still runs).

### Recovery
- The Pi sets `recover` = 1 in the command frame. It acts on the **0 → 1 change** only, so send 0 normally, then 1 (holding it at 1 is fine). To retry, send 0 again, then 1.
- Refused while the leak sensor is still wet (PB12 or PA6 HIGH): nothing changes and the Pi keeps seeing `leak = 1`.
- On success `emergency_recover()` restarts thruster PWM at 0 % and the servo at level, restarts the ADC, restarts I2C1 and re-initializes the sensors within 1 s (yellow LED until they answer), then re-arms leak detection.
- Thrusters stay at 0 % until the Pi sends a new motor command. The ESCs may need a moment at neutral to re-arm.

<hr>

## Status LEDs
Each LED shows one thing, so statuses combine (e.g. green blinking + yellow solid = waiting for the Pi with a sensor fault).

**Power-up:**
1. Lamp test: all three on for ~1 s
2. Green solid while the sensors initialize (solid, so it can't be mistaken for "Waiting for Pi")
3. Fast green flicker (0.6 s): boot finished
4. Switch to the status table below

If a leak is present at boot, the power-up sequence is skipped and the red LED stays on (LEAK).

| Status | Green | Yellow | Red |
|-------|:---:|:---:|:---:|
| Pi connected | ● | ○ | ○ |
| Waiting for Pi | ◐ | ○ | ○ |
| Pi link lost | ○ | ○ | ◐ |
| I2C sensor fault | ○ | ● | ○ |
| **LEAK** | ○ | ○ | ● |
| STM32 not running | ○ | ○ | ○ |

● = solid, ◐ = blinking 1 Hz (500 ms on / 500 ms off), ○ = off. Power is indicated by separate LEDs connected to the 3V3 and 5V rails.

- Solid red = LEAK, blinking red = link lost.
- LEAK overrides everything (only red is lit) and stays on until a recover command.
- Sensor faults and link state are live: the LEDs clear when the fault recovers.
- A sensor is faulted after 3 failed reads in a row (60 ms). Faulted sensors are re-initialized every 1 s; the IMU's operation mode is also checked every 1 s to catch it resetting itself.
- "Waiting" = booted, no valid command from the Pi yet. "Link lost" = commands were received, then timed out (150 ms).
- The blink is timed from `HAL_GetTick()` in `status_led_update()` (non-blocking), so green and red always blink in step.
- The SWO debug print shows which sensor is faulted, e.g. `Faults: 0x04 [ DEPTH ]`.
- If the firmware hangs, the LEDs freeze on their last pattern (a blinking LED stops blinking, either on or off).


<hr>

## Part Notes
- The ESC used in this project expects a max 500 Hz PWM signal with the following specs:
	- Stop = 1500 µs
	- Forward (max) = 1900 µs
	- Reverse (max) = 1100 µs

[Afro30A - Datasheet](https://arduino.ua/docs/AfroESC30A.pdf)

If possible, upgrade to [BlueRobotics BasicESC](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/).<br>
Expects the same signal and specs as described above, so no change in firmware.

- Per-motor deadband (onset) pulses are set in `Core/Motors/motors.h` (`MOTORn_REVERSE_ONSET_PULSE` / `MOTORn_FORWARD_ONSET_PULSE`).

- Camera tilt servo: MG90S, 50 Hz, 1000 µs = −90°, 1500 µs = centre, 2000 µs = +90°.<br>
  Commanded range is clamped to ±45.0°. Set `SERVO_REVERSED` to 1 in `motors.h` if + tilts the camera down.

I2C devices (all on I2C1, 100 kHz):

| Device | 7-bit Address |
|--------|:---:|
| BNO055 IMU | `0x28` |
| Bar30 (MS5837-30BA) | `0x76` |
| Celsius (TSYS01) | `0x77` |


**Depth sensor (Bar30) settings**: boot defaults in `main.c` (under USER CODE BEGIN PD). The command centre can change all three at run time (sent through the Pi as a reference frame, see the UART protocol above). After a reset the defaults apply again until the Pi re-sends them, which it does whenever the STM32 reconnects. Keep the defaults the same as the command centre's (`config.py`):

| Setting | Set with | Current value | Notes |
|---------|----------|:---:|-------|
| Surface reference | `SURFACE_PRESSURE` | 1013.25 mbar (101 325 Pa, standard sea-level pressure) | Pressure read as 0 m depth (affects `depth_x100` only, not `pressure_x10`) |
| Water density | `WATER_DENSITY` | 1000.00 kg/m³ (freshwater) | Use 1025 kg/m³ for saltwater (average). Affects `depth_x100` only |
| Second-order compensation | `second_order_calculation` | `true` (on) | Extra temperature correction below 20 °C. Set `false` to disable |

- Battery monitor divider: R1 = 100 kΩ, R2 = 18 kΩ (`BATTERY_MONITOR_R1/R2` in `main.c`). V<sub>batt</sub> = V<sub>ADC</sub> × (R1 + R2) / R2.

- Enclosure thermistor: Vishay NTCLE100E3103JB0 (10 kΩ at 25 °C) with a 10 kΩ divider resistor.

<hr>

## STM32 Configuration & Pins
These are set in STM32CubeMX (`CamosunROV_STM32.ioc`).

The HCLK is set to 180 MHz (HSE 8 MHz → PLL). APB1 = 45 MHz (timers 90 MHz), APB2 = 90 MHz (timers 180 MHz).

Programming Pins (SWD on PCB):
| SWD Pin | STM32 Pin | Function | Notes |
|:-------:|:---------:|:--------:|-------|
| 1 | 3V3 | Target Supply Sense | Debug/programming |
| 2 | PA14 | SWCLK | Debug/programming |
| 3 | GND | Ground | Debug/programming |
| 4 | PA13 | SWDIO | Debug/programming |
| 5 | NRST | Reset | Debug/programming |
| 6 | PB3 | SWO (optional) | Trace/debug output (`printf`) |

Pin configuration:
| STM32 Pin | Function | Peripheral / AF | Notes |
|:---:|:--------:|:----------------:|-------|
| PA11 | Motor 1 PWM | TIM1_CH4 | ~300 Hz signal |
| PA10 | Motor 2 PWM | TIM1_CH3 | |
| PA9  | Motor 3 PWM | TIM1_CH2 | |
| PA8  | Motor 4 PWM | TIM1_CH1 | |
| PC9  | Motor 5 PWM | TIM8_CH4 | ~300 Hz signal |
| PC8  | Motor 6 PWM | TIM8_CH3 | |
| PB0  | Camera Tilt Servo PWM | TIM3_CH3 | ~50 Hz signal |
| PA6  | Leak Sensor | TIM8_BKIN | auto terminates TIM8 PWM signals |
| PB12 | Leak Sensor | TIM1_BKIN | auto terminates TIM1 PWM signals |
| PC10 | UART TX → Pi RX | UART4_TX | DMA1 Stream 4 |
| PC11 | UART RX ← Pi TX | UART4_RX | DMA1 Stream 2, idle-line detection |
| PB6  | I2C SCL | I2C1_SCL | BNO055, Bar30, TSYS01 |
| PB7  | I2C SDA | I2C1_SDA | BNO055, Bar30, TSYS01 |
| PA0  | Battery Monitor | ADC1_IN0 | Uses DMA |
| PA1  | Internal Temperature | ADC1_IN1 | Uses DMA |
| PC0  | Red LED | GPIO Output | |
| PC1  | Green LED | GPIO Output | |
| PC2  | Yellow LED | GPIO Output | |
| PA2  | USART2_TX | USART2 | Configured, not used by firmware |
| PA3  | USART2_RX | USART2 | Configured, not used by firmware |

Timer configuration:
| Timer | Use | Prescaler (PSC) | Period (ARR) | Output |
|:---:|-----|:---:|:---:|-------|
| TIM1 | Motors 1–4 | 180 | 3333 | ~300 Hz, ~1 µs per tick |
| TIM8 | Motors 5–6 | 180 | 3333 | ~300 Hz, ~1 µs per tick |
| TIM3 | Camera tilt servo | 90 | 20000 | ~50 Hz, ~1 µs per tick |
| TIM2 | ADC trigger (TRGO) | 90 | 1000 | ~1 kHz |

*Note:*<br>
- *PA6 and PB12 are physically connected, hence a leak triggers HIGH on both.*
- *ADC sampling rate is controlled by TIM2 (16 samples per channel are averaged), conversion rate is still controlled by the APB2 frequency.*
- *The Pi's UART pins are 3.3 V logic, same as the STM32 — connect directly, TX ↔ RX crossed, with a common ground.*
