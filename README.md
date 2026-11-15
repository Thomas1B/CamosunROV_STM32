# CamosunROV STM32

Firmware for the CamosunROV's STM32F446RE (Cortex-M4F) board, which runs the motors and sensors. It talks to a Raspberry Pi 4 over UART4: the Pi sends thruster and camera-tilt commands at 50 Hz, and the STM32 sends telemetry back at 50 Hz.

Built with STM32CubeIDE 2.2.0 / STM32CubeMX 6.18.1, firmware package **STM32Cube FW_F4 V1.28.3**. [STM32F446RE datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32f446/documentation.html)

<hr>

# TODO:
- [ ] Test I2C Water Temperature Sensor firmware (waiting for sensor to arrive)
- [ ] Test Recovery (restarts PWM generation, ADC reads, re-enables BKIN interrupts, etc.)
- [ ] Find each ESC's deadband range (all motors still use the default 1450 µs / 1550 µs onset values)
- [ ] Full Test on Custom on PCB
- [ ] Test I2C bus clear on hardware (scope on SCL/SDA: hold SDA LOW with a jumper to GND, release it, check up to 9 SCL pulses + STOP and that the sensors come back)
- [x] Full test of hardware prototype using Pilot UI
- [x] Test IWDG on hardware (temporary `while (1) {}` in the main loop: chip resets within ~0.5 s, board boots normally)
- [x] Test status LEDs on hardware (lamp test, boot sequence, each pattern in the LED table)
- [x] Finish Emergency Shutdown Function
- [x] UART to/from Pi
- [X] Live I2C sensor fault detection + 1 s retry (code done)
- [x] Live fault flags to the Pi (faults clear when they recover; only LEAK stays set)
- [x] Test camera tilt servo on hardware

<hr>

## Features
- 6 thrusters, cut off in hardware on a leak (TIM1/TIM8 break input)
- Camera tilt servo (MG90S)
- Failsafes: thrusters stop after 150 ms with no valid command; IWDG resets the chip if the main loop hangs ~0.5 s
- Sensors:
	- Depth and absolute pressure — Blue Robotics Bar30 (MS5837-30BA)
	- Water temperature — Blue Robotics Celsius (TSYS01)
	- Orientation (heading, roll, pitch) — BNO055 IMU in NDOF mode
	- Enclosure temperature — NTC thermistor on ADC
	- Battery voltage — resistor divider on ADC
- Live fault flags in every telemetry frame
- Status LEDs (red / yellow / green); SWO debug `printf` every 1.5 s

<hr>

## UART Protocol (Pi ↔ STM32)
UART4, 115200 bit/s, 8N1, DMA RX (idle-line detection) + DMA TX. Multi-byte values are little-endian. Checksum = sum of the bytes between START and CHECKSUM, mod 256. Full details: `Core/UART_DMA/UART_DMA.h`.

**Pi → STM32 command (11 bytes, 50 Hz):**
| Byte(s) | Field | Type | Units / Range |
|:---:|-------|:---:|-------|
| 0 | START | u8 | `0xAA` |
| 1–6 | motor1 … motor6 | i8 | % throttle, −100 … 100 |
| 7–8 | cam_tilt_dc | i16 | camera tilt ×10 (deci-degrees), 0 = level, + = up |
| 9 | recover | u8 | 0 = normal, 1 = clear a leak shutdown (see [Recovery](#recovery)) |
| 10 | CHECKSUM | u8 | |

**Pi → STM32 reference (10 bytes, on change):** Bar30 depth settings from the command centre. Sent 3 times (~10 ms after a command frame) and again whenever the STM32 reconnects.
| Byte(s) | Field | Type | Units / Range |
|:---:|-------|:---:|-------|
| 0 | START_REF | u8 | `0xAB` |
| 1–4 | surface_hpa_x100 | u32 | surface pressure, hPa (= mbar) ×100, 80000 … 110000 |
| 5–6 | density_x10 | u16 | water density, kg/m³ ×10, 9000 … 11000 |
| 7 | flags | u8 | bit 0 = second-order calibration on |
| 8 | ref_id | u8 | +1 each time the values change (repeats share it) |
| 9 | CHECKSUM | u8 | |

Out-of-range values are rejected even with a good checksum. The main loop applies valid frames (`apply_reference()`), replacing the boot defaults. Reference frames don't reset the link timeout and aren't counted as lost commands.

**STM32 → Pi telemetry (21 bytes, 50 Hz):**
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
| 17 | leak | u8 | 0 = dry, 1 = leak latched (until dry + successful recover) |
| 18 | fault_flags | u8 | active faults (see below) |
| 19 | reset_cause | u8 | why the STM32 last restarted (see below) |
| 20 | CHECKSUM | u8 | |

- Out-of-range values are clamped by the sender (i16 ×100: −327.68 … 327.67, u16 ×100: 0 … 655.35, u16 ×10: 0 … 6553.5).
- `pressure_x10` is the raw Bar30 reading; the surface reference only affects `depth_x100`. It is ×10 because the Bar30 reads up to ~3000 kPa (0.1 kPa ≈ 1 cm of water, same resolution as depth).
- The Pi parser (`UART_DMA.py`) must expect 21 bytes; a 20-byte parser fails every checksum.

**Reset cause** — read once at boot from the RCC flags and sent in every frame until the next reset. An event, not a fault, so it isn't in `fault_flags`. The Pi should log a watchdog reset as a warning.
| Value | Cause | Meaning |
|:---:|-------|-------|
| 0 | Power-on / brown-out | Normal power-up, or the supply dipped too low |
| 1 | Watchdog (IWDG) | Main loop hung ~0.5 s: firmware bug or stuck bus, worth investigating |
| 2 | Reset pin | Reset button or debugger/ST-LINK |
| 3 | Software reset | `NVIC_SystemReset()` (not used yet) |
| 4 | Other | Window watchdog or low-power reset (neither used) |

**Fault flags** — live: a bit clears when the fault recovers, except LEAK, which needs a recover command. While a sensor's bit is set, its telemetry holds the last good value and should be ignored.
| Bit | Value | Fault |
|:---:|:---:|-------|
| 0 | `0x01` | LEAK |
| 1 | `0x02` | IMU (BNO055 not responding, or reset itself) |
| 2 | `0x04` | DEPTH (Bar30 not responding) |
| 3 | `0x08` | TEMP (TSYS01 not responding) |
| 4 | `0x10` | UART (command timeout after the link was established) |

A sensor faults after 3 failed reads in a row (60 ms). Faulted sensors are re-initialized every 1 s, and the IMU's operation mode is checked every 1 s to catch self-resets.

<hr>

## Leak / Emergency Shutdown
- The leak sensor drives the TIM1 and TIM8 break inputs (PB12, PA6, active HIGH, wired together). The hardware forces all six thruster outputs off immediately.
- The break ISR only latches `SYS_STATE_FAULT_LEAK`. `emergency_shutdown()` in the main loop then stops everything except the LEDs and UART4: thruster and servo PWM off with pins LOW, ADC + TIM2 stopped, I2C1 de-initialized, break and ADC DMA interrupts disabled.
- Telemetry continues with `leak = 1`; sensor fields are frozen (0 if the leak was present at boot). Motor and tilt commands are ignored.
- A leak at boot skips sensor start-up and goes straight to shutdown.

### Recovery
- Triggered by a **0 → 1 change** of `recover` (holding 1 is fine; to retry, send 0 then 1).
- Refused while PB12 or PA6 is still HIGH.
- On success, `emergency_recover()` restarts thruster PWM at 0 %, the servo at level, the ADC and I2C1, re-initializes the sensors within 1 s (yellow LED until they answer), then re-arms leak detection.
- Thrusters stay at 0 % until the next motor command; the ESCs may need a moment at neutral to re-arm.

<hr>

## Watchdog (IWDG)
The command timeout needs the main loop running. If the STM32 hangs (HardFault, stuck loop, jammed I2C bus), the timers keep outputting the last pulse and the thrusters keep running; the IWDG covers this.

- LSI clock (~32 kHz), prescaler 32, reload 500 → ≈ 0.5 s. LSI tolerance (17–47 kHz) gives 0.34–0.94 s. The slowest normal loop pass is ~100 ms.
- On reset the PWM pins stop, so the ESCs stop. The board reboots with thrusters at 0 % (`failsafeActive = 1`) and reports `reset_cause = 1`.

| Where | What |
|-------|------|
| `.ioc` → Project Manager → Advanced Settings | `MX_IWDG_Init` set to **Do Not Generate Function Call**, so the watchdog doesn't start before the boot sequence |
| `main.c`, USER CODE BEGIN Init | `__HAL_DBGMCU_FREEZE_IWDG()`: breakpoints don't cause resets |
| `main.c`, end of USER CODE BEGIN 2 | Reset cause read and RCC flags cleared; `MX_IWDG_Init()` called just before the main loop |
| `main.c`, top of main loop (USER CODE BEGIN 3) | `HAL_IWDG_Refresh(&hiwdg)`: the only refresh, placed so the leak branch's `continue`s still reach it |

**Rules:**
- Refresh only from the main loop, never from an ISR or callback.
- Nothing in the main loop may block > ~300 ms. `imu_start()`, `bno055_setup()`, `bno055_reset()` (700 ms) and the BNO055 calibration loops are boot-only.
- Once started, the IWDG can only be stopped by a reset.

<hr>

## I2C Bus Clear
An IWDG reset doesn't power-cycle the sensors. A reset mid-read can leave a sensor holding SDA LOW, so I2C1 sees a busy bus and all three sensors fault until the ROV is power-cycled.

`i2c1_bus_clear()` (UM10204 Rev. 7.0, §3.1.16) drives PB6/PB7 as open-drain GPIO, clocks up to nine SCL pulses until SDA releases, then sends a STOP; `HAL_I2C_Init()` returns the pins to I2C1. Pulses are below 100 kHz (half-period ≥ 5 µs); the routine takes well under 1 ms.

| Where | When it runs |
|-------|------|
| `MX_I2C1_Init()`, USER CODE BEGIN I2C1_Init 0 | Every I2C1 init (boot and `emergency_recover()`) |
| `sensor_health_check()` | IMU, DEPTH and TEMP all faulted **and** SDA reads LOW |

<hr>

## Status LEDs
Each LED shows one thing, so statuses combine (e.g. green blinking + yellow solid = waiting for Pi with a sensor fault).

**Power-up:** lamp test (all on ~1 s) → green solid while sensors initialize → fast green flicker (0.6 s) → status table. A leak at boot skips this and shows red.

| Status | Green | Yellow | Red |
|-------|:---:|:---:|:---:|
| Pi connected | ● | ○ | ○ |
| Waiting for Pi (no valid command yet) | ◐ | ○ | ○ |
| Pi link lost (commands timed out, 150 ms) | ○ | ○ | ◐ |
| I2C sensor fault | ○ | ● | ○ |
| **LEAK** (overrides all, until recover) | ○ | ○ | ● |
| STM32 not running | ○ | ○ | ○ |

● = solid, ◐ = 1 Hz blink (500 ms on / off), ○ = off. Separate LEDs on the 3V3 and 5V rails show power.

- Sensor and link states are live and clear on recovery. The SWO print names the fault, e.g. `Faults: 0x04 [ DEPTH ]`.
- Blinking is non-blocking (`HAL_GetTick()` in `status_led_update()`), so green and red blink in step.
- If the firmware hangs, the LEDs freeze until the watchdog resets the chip (~0.5 s) and the power-up sequence runs again.

<hr>

## Part Notes
- **ESC** ([Afro30A datasheet](https://arduino.ua/docs/AfroESC30A.pdf)): max 500 Hz PWM; stop 1500 µs, full forward 1900 µs, full reverse 1100 µs. The [Blue Robotics BasicESC](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/) is a drop-in upgrade with no firmware change. Per-motor deadband (onset) pulses: `MOTORn_REVERSE_ONSET_PULSE` / `MOTORn_FORWARD_ONSET_PULSE` in `Core/Motors/motors.h`.
- **Camera tilt servo** (MG90S): 50 Hz, 1000 µs = −90°, 1500 µs = centre, 2000 µs = +90°. Clamped to ±45.0°. Set `SERVO_REVERSED` to 1 in `motors.h` if + tilts down.
- **Battery divider:** R1 = 100 kΩ, R2 = 18 kΩ (`BATTERY_MONITOR_R1/R2` in `main.c`). V<sub>batt</sub> = V<sub>ADC</sub> × (R1 + R2) / R2.
- **Enclosure thermistor:** Vishay NTCLE100E3103JB0 (10 kΩ at 25 °C) with a 10 kΩ divider resistor.

I2C devices (all on I2C1, 100 kHz):

| Device | 7-bit Address |
|--------|:---:|
| BNO055 IMU | `0x28` |
| Bar30 (MS5837-30BA) | `0x76` |
| Celsius (TSYS01) | `0x77` |

**Bar30 depth settings:** boot defaults in `main.c` (USER CODE BEGIN PD), overridden at run time by reference frames from the command centre. Keep them in sync with the command centre's `config.py`.

| Setting | Set with | Default | Notes |
|---------|----------|:---:|-------|
| Surface reference | `SURFACE_PRESSURE` | 1013.25 mbar (101 325 Pa) | Pressure read as 0 m; affects `depth_x100` only |
| Water density | `WATER_DENSITY` | 1000.00 kg/m³ (fresh) | ~1025 kg/m³ for salt; affects `depth_x100` only |
| Second-order compensation | `second_order_calculation` | `true` | Extra temperature correction below 20 °C |

<hr>

## STM32 Configuration & Pins
Set in STM32CubeMX (`CamosunROV_STM32.ioc`). HCLK 180 MHz (HSE 8 MHz → PLL); APB1 45 MHz (timers 90 MHz), APB2 90 MHz (timers 180 MHz).

Programming pins (SWD on PCB):
| SWD Pin | STM32 Pin | Function |
|:-------:|:---------:|:--------:|
| 1 | 3V3 | Target supply sense |
| 2 | PA14 | SWCLK |
| 3 | GND | Ground |
| 4 | PA13 | SWDIO |
| 5 | NRST | Reset |
| 6 | PB3 | SWO (optional, `printf` trace) |

Pin configuration:
| STM32 Pin | Function | Peripheral / AF | Notes |
|:---:|:--------:|:----------------:|-------|
| PA11 | Motor 1 PWM | TIM1_CH4 | ~300 Hz |
| PA10 | Motor 2 PWM | TIM1_CH3 | |
| PA9  | Motor 3 PWM | TIM1_CH2 | |
| PA8  | Motor 4 PWM | TIM1_CH1 | |
| PC9  | Motor 5 PWM | TIM8_CH4 | ~300 Hz |
| PC8  | Motor 6 PWM | TIM8_CH3 | |
| PB0  | Camera Tilt Servo PWM | TIM3_CH3 | ~50 Hz |
| PA6  | Leak Sensor | TIM8_BKIN | cuts TIM8 PWM |
| PB12 | Leak Sensor | TIM1_BKIN | cuts TIM1 PWM |
| PC10 | UART TX → Pi RX | UART4_TX | DMA1 Stream 4 |
| PC11 | UART RX ← Pi TX | UART4_RX | DMA1 Stream 2, idle-line detection |
| PB6  | I2C SCL | I2C1_SCL | BNO055, Bar30, TSYS01 |
| PB7  | I2C SDA | I2C1_SDA | BNO055, Bar30, TSYS01 |
| PA0  | Battery Monitor | ADC1_IN0 | DMA |
| PA1  | Internal Temperature | ADC1_IN1 | DMA |
| PC0  | Red LED | GPIO Output | |
| PC1  | Green LED | GPIO Output | |
| PC2  | Yellow LED | GPIO Output | |
| PA2  | USART2_TX | USART2 | Configured, unused |
| PA3  | USART2_RX | USART2 | Configured, unused |

Timer configuration:
| Timer | Use | Prescaler (PSC) | Period (ARR) | Output |
|:---:|-----|:---:|:---:|-------|
| TIM1 | Motors 1–4 | 180 | 3333 | ~300 Hz, ~1 µs per tick |
| TIM8 | Motors 5–6 | 180 | 3333 | ~300 Hz, ~1 µs per tick |
| TIM3 | Camera tilt servo | 90 | 20000 | ~50 Hz, ~1 µs per tick |
| TIM2 | ADC trigger (TRGO) | 90 | 1000 | ~1 kHz |
| IWDG | Watchdog (LSI ~32 kHz) | 32 | 500 | ~0.5 s timeout |

*Notes:*
- *ADC sampling is paced by TIM2 (16 samples per channel averaged); conversion speed is set by the APB2 clock.*
- *The Pi's UART is 3.3 V logic like the STM32: connect directly, TX ↔ RX crossed, common ground.*