# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32G474RE firmware for a single-axis pick-and-place robot arm. Built with STM32CubeIDE (Eclipse + CubeMX). Active branch is `dev`.

## Build & Flash

**All builds must be done inside STM32CubeIDE** — there is no standalone Makefile workflow supported here.

1. Open the project in STM32CubeIDE (`File → Open Projects from File System`)
2. Build: `Project → Build Project` (or `Ctrl+B`)
3. Flash + debug: `Run → Debug` (ST-Link on Nucleo board)
4. Clean build: `Project → Clean` then rebuild

**After any `.ioc` file change**: Right-click project → `Generate Code` (or open the `.ioc` and click **Generate Code** in CubeMX). This regenerates `main.c` init section, `main.h` pin defines, `stm32g4xx_hal_msp.c`, and `stm32g4xx_hal_conf.h`.

If a new peripheral is added in the `.ioc`, it must also appear in `ProjectManager.functionlistsort` inside the `.ioc` file, otherwise CubeMX will not generate the `MX_*_Init` function or copy the HAL driver files. Pattern: `N-MX_<Periph>_Init-<Periph>-false-HAL-true`.

## CubeMX Code-Generation Rules (Critical)

**Never manually write code in these files** — CubeMX owns them and will overwrite anything outside `USER CODE` blocks:

- `Core/Src/main.c` — peripheral init calls, `SystemClock_Config`
- `Core/Inc/main.h` — pin `#define`s (e.g., `Current_Sensor_Pin`)
- `Core/Src/stm32g4xx_hal_msp.c` — `HAL_*_MspInit` / `HAL_*_MspDeInit`
- `Core/Inc/stm32g4xx_hal_conf.h` — `HAL_*_MODULE_ENABLED` defines

User application code lives **only** inside `/* USER CODE BEGIN x */ ... /* USER CODE END x */` blocks. Everything outside those delimiters is regenerated on every CubeMX code-gen.

## Architecture

### Firmware Layers

```
main.c (USER CODE blocks)
  ├── Motor_Init()          ← motor_controller.c — cascade PID + S-curve trajectory
  ├── HW_Init()             ← hw_io.c            — GPIO relay/opto I/O abstraction
  ├── CurrentSensor_Init()  ← current_sensor.c   — WCS1800 ADC driver
  ├── ModbusBridge_Init()   ← modbus_bridge.c    — Modbus RTU bridge to base system
  ├── Telemetry_Init()      ← telemetry_hub.c    — dashboard serial protocol
  └── Kalman_Init()         ← kalman.c           — 4-state DC motor Kalman filter
```

### Timer Assignments

| Timer | Freq | Role |
|-------|------|------|
| TIM1  | PWM  | Motor PWM output (H-bridge) |
| TIM3  | —    | Quadrature encoder interface |
| TIM6  | 1 kHz | `HAL_TIM_PeriodElapsedCallback` → Kalman_Tick + triggers 100 Hz control loop |
| TIM16 | —   | Telemetry tick |

### UART Assignments

| UART     | Normal mode | Switched mode |
|----------|-------------|---------------|
| LPUART1  | Dashboard (115200 8N1) | Modbus/Base System (19200 8E1) |
| USART3   | ESP32 joystick (always active) | — |

`LPUART1_SetMode(bool modbus_mode)` in `main.c` reconfigures baud/parity at runtime.

**`printf` routing rule (critical):** `_write()` sends debug output to **LPUART1 only** (dashboard), never to USART3. The ESP32 joystick module expects only single-character echo ACKs from STM32. Sending debug strings to USART3 causes the ESP32 to fire a `D` (disconnect) status packet → `FAULT_JOYSTICK_LOST` → `emergency_stop = true`. Do NOT add `huart3` back to `_write()`. Mode toggle is deferred via `mode_toggle_request` flag (set ISR-safe, executed in main loop) because `HAL_Delay` and `HAL_UART_DeInit` cannot run inside ISRs.

### Control Loop (100 Hz, motor_controller.c)

`Motor_ControlLoop()` is called from the 1 kHz TIM6 ISR with a `/10` divider.

Flow: encoder read → **ZVD input shaper** (filters position setpoint) → S-curve trajectory step → **outer position PID** → **inner speed PID** → PWM + velocity/acceleration/disturbance feedforward → safety checks.

All tunable parameters live in `params.h` as `#define` constants; runtime-mutable copies live in the `Motor_TuningParams_t tuning` struct (`motor_controller.h`).

**ZVD Input Shaper:** 3-impulse Zero-Vibration-Derivative shaper on the position setpoint. Enabled via `tuning.shaper_enable`; coefficients derived from `tuning.shaper_omega_n` (rad/s) and `tuning.shaper_zeta`. Ring buffer size = 100 ticks (1 s max delay). Defaults in `params.h` under `DEFAULT_SHAPER_*`.

**Offset Homing:** `tuning.home_offset_deg` shifts position-0 relative to the proximity sensor centre. Survives re-homing via `original_home_offset_deg`. Settable at runtime with `$SET:HOME_OFFSET=<deg>*`.

**Disturbance Feedforward:** `DEFAULT_K_TFF` scales the Kalman τ_L (load-torque) estimate into a direct PWM pre-compensation term. Tune 0 → 1; large values may cause oscillation.

### HW I/O (`hw_io.c`)

`HW_Debug_t hw` is a flat struct visible in **STM32CubeIDE Live Expressions**. Add `hw` to Live Expressions to read/force any I/O at runtime without recompiling.

`HW_RefreshIO()` runs at 100 Hz and: reads all opto inputs → samples ADC current → updates `hw.*` fields → writes gripper relay outputs from `hw.out_gripper_up / hw.out_gripper_down`.

Gripper logic — **2 independent spring-return actuators on 2 pins**:
- PA1 (`out_gripper_up`): 1 = UP relay energised → gripper moves UP; 0 = spring returns DOWN. `Gripper_Up/Down()` only touch this pin.
- PA4 (`out_gripper_down`): 1 = CLOSE relay energised → claw CLOSES; 0 = spring opens claw. `Gripper_Open/Close()` only touch this pin.

All 4 states (UP+OPEN, UP+CLOSE, DOWN+OPEN, DOWN+CLOSE) are independently controllable with no extra hardware.

### Telemetry Protocol

Packet format: `$KEY:VAL,KEY:VAL,...*\r\n` (both directions).

- **Fast telemetry** (50 Hz): POS, VEL, ACC, TAR, VSET, ASET, PWM, MODE, SYSM, JOGM, JOY, ESTOP, FAULT, PROX, GHOST, GUP (vertical up relay), GDN (claw close relay), CURR + Kalman KTH/KOM/KTL/KIA/KIV/KP00–KP33
- **Slow telemetry** (1 Hz): PID gains, S-curve limits, safety config
- **Incoming commands**: `$CMD:ESTOP=1*`, `$SET:TARGET=90.0*`, `$SET:SPEED_KP=1.5*`, etc.

Parser is in `telemetry_hub.c → Telemetry_ProcessByte()` (byte-by-byte, called from UART RX interrupt).

### Current Sensor (`current_sensor.c`)

WCS1800 on PA0 via ADC1_IN1, with a 1 kΩ / 1.8 kΩ voltage divider (k = 0.6429).

`CurrentSensor_Init()` runs a 64-sample average at startup to calibrate `_v_zero` (actual sensor zero-current voltage). This replaces the hardcoded 2.5 V `CS_VOFFSET` constant and eliminates phantom-current offset caused by sensor VCC tolerance. `CurrentSensor_GetVzero()` returns the calibrated value (expected ≈ 2.5 V ± 0.3 V).

### Kalman Filter (`kalman.c`)

4-state filter: `x = [θ, ω, τ_L, i_a]` in output-shaft frame. Runs at 1 kHz in the TIM6 ISR via `Kalman_Tick(u_volts, theta_rad)`. Motor physical constants (R, L, K_e, K_t, B, J, N, η) are in `params.h` under `MOT_*`.

### Dashboard (`dashboard/`)

Static HTML/JS app using the **Web Serial API** (requires Chrome or Edge). Open `index.html` directly — no server needed.

- `app.js` — serial connect/disconnect, `processPacket()` key→state parser, `updateUI()`, tuning send, path sequencer, ghost/sine test modes
- `charts.js` — `TelemetryChart` class (live + tuning capture modes with run history)
- `visualizer.js` — `RobotVisualizer` (top-view arm) and `GripperVisualizer` (isDown, isOpen)
- `testing.js` — structured test suite with CSV export

Key parser mapping (telemetry key → `state.*`):
- `GUP` → `state.gripper_ud = (GUP === '0')` — `gripper_ud` true = DOWN (UP relay is off)
- `GDN` → `state.gripper_co = (GDN === '1')` — `gripper_co` true = CLOSED (claw close relay is on)
- `CURR` → `state.current` (displayed in System Status card)

## Key Files for Common Tasks

| Task | Files to edit |
|------|--------------|
| Change PID defaults / motion limits / safety thresholds | `Core/Inc/params.h` |
| Tune ZVD input shaper (ωn, ζ) or enable/disable it | `Core/Inc/params.h` (`DEFAULT_SHAPER_*`) + `tuning.shaper_*` at runtime |
| Change homing sensor offset | `Core/Inc/params.h` (`DEFAULT_HOME_OFFSET`) or `$SET:HOME_OFFSET=<deg>*` at runtime |
| Tune disturbance feedforward | `Core/Inc/params.h` (`DEFAULT_K_TFF`) |
| Add/change a GPIO pin | `.ioc` (CubeMX GUI) → regenerate → never touch `main.h` directly |
| Add a new HAL peripheral | `.ioc` → add to `functionlistsort` → regenerate |
| Change telemetry packet fields | `Core/Src/telemetry_hub.c` + `dashboard/app.js` (processPacket switch) |
| Gripper relay logic | `Core/Src/motor_controller.c` (`Gripper_Up/Down/Open/Close`) |
| HW I/O struct fields | `Core/Inc/hw_io.h` (HW_Debug_t) + `Core/Src/hw_io.c` (HW_RefreshIO) |
| Dashboard UI elements | `dashboard/index.html` + `dashboard/app.js` (updateUI) |
| Motor physical model (Kalman) | `Core/Inc/params.h` (`MOT_*` constants) |
