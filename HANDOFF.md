# HANDOFF — PickPlace Robot Firmware G06

> เขียนโดย: Thadzy | วันที่ส่งต่อ: 2026-06-03  
> ผู้รับต่อ: อ่านไฟล์นี้ก่อนแตะโค้ดทุกครั้ง

---

## 1. ระบบทำอะไร

แขนกลหยิบวางชิ้นงาน 1 แกน (single-axis pick-and-place)  
ควบคุมผ่าน 2 channel:

```
[Base System PC] ──Modbus RTU (RS-485)──► [STM32G474RE] ──PWM──► [DC Motor + Gearbox 70:1]
[ESP32 Joystick] ──UART 115200──────────►      │                        │
[Dashboard PC]   ──UART 115200──────────►      │                   [Encoder TIM3]
                                               └──relay──► [Gripper: ↕ + claw]
```

---

## 2. Build & Flash

**ต้องใช้ STM32CubeIDE เท่านั้น** — ไม่มี Makefile standalone

```
1. File → Open Projects from File System → เลือก folder นี้
2. Build:  Project → Build Project  (หรือ Ctrl+B)
3. Flash:  Run → Debug  (ต้องต่อ ST-Link บน Nucleo)
4. Clean:  Project → Clean → Build
```

**ถ้าแก้ .ioc:** คลิก Generate Code ใน CubeMX ก่อน build เสมอ  
**ห้ามแก้ไฟล์เหล่านี้ตรงๆ** (CubeMX จะ overwrite): `main.c`, `main.h`, `stm32g4xx_hal_msp.c`, `stm32g4xx_hal_conf.h`  
แก้โค้ดใน `/* USER CODE BEGIN */` blocks เท่านั้น

---

## 3. Architecture Overview

```
┌─────────────────────────────────────────────────┐
│                   main.c                        │
│  USER CODE blocks only — CubeMX owns the rest  │
│                                                 │
│  Motor_Init()   HW_Init()   Kalman_Init()       │
│  ModbusBridge_Init()   Telemetry_Init()         │
│  CurrentSensor_Init()                           │
└────┬──────────┬──────────┬──────────┬───────────┘
     │          │          │          │
     ▼          ▼          ▼          ▼
motor_      hw_io.c    kalman.c   modbus_     telemetry_
controller                       bridge.c    hub.c
.c                                   │
     │                           modbus_rtu.c
     └── PID + S-curve + ZVD shaper + safety
```

### Timer Map

| Timer | Rate | Role |
|-------|------|------|
| TIM1 | PWM | Motor H-bridge output |
| TIM3 | — | Quadrature encoder |
| TIM6 | 1 kHz | ISR: Kalman_Tick + triggers 100 Hz control loop |
| TIM16 | — | Modbus T3.5 timeout |

### UART Map

| UART | ใช้ทำอะไร | หมายเหตุ |
|------|----------|---------|
| LPUART1 | Dashboard (115200 8N1) **หรือ** Modbus (19200 8E1) | ใช้ร่วมกัน — mode switch ด้วย `LPUART1_SetMode()` |
| USART3 | ESP32 joystick (fixed 115200) | ห้ามส่ง printf ที่นี่เด็ดขาด |

**⚠️ กฎสำคัญ:** `printf` / `_write()` ส่งไป LPUART1 เท่านั้น  
ถ้าส่งไป USART3 → ESP32 เข้าใจผิด → `FAULT_JOYSTICK_LOST` → emergency stop

---

## 4. Module Map (ไฟล์ไหนทำอะไร)

### Existing Modules (build-tested on STM32)

| ไฟล์ | หน้าที่ | แก้เมื่อ |
|------|---------|---------|
| `Core/Inc/params.h` | **ค่าตั้งต้นทุกอย่าง** PID gains, S-curve limits, Kalman noise, motor model | tune ค่า |
| `Core/Src/motor_controller.c` | Control loop 100 Hz: encoder → ZVD → S-curve → PID → PWM + safety | แก้ control logic |
| `Core/Inc/motor_controller.h` | enum modes, fault codes, structs, FAULT_SET/CLR macros | เพิ่ม fault code |
| `Core/Src/kalman.c` | 4-state Kalman filter (θ, ω, τ_L, i_a) รัน 1 kHz | แก้ motor model |
| `Core/Src/modbus_bridge.c` | Register map + heartbeat + PnP state machine | เพิ่ม/แก้ register |
| `Core/Src/modbus_rtu.c` | Modbus RTU protocol: CRC16, FC03/FC06/FC16, state machine | แก้ protocol |
| `Core/Src/modbus_frame.c` | **ใหม่** Pure C99 Modbus framer: CRC16 + FC03/FC06/FC16 — unit-testable | แก้ framing |
| `Core/Src/telemetry_hub.c` | Dashboard serial protocol (50 Hz fast + 1 Hz slow) | เพิ่ม telemetry field |
| `Core/Src/hw_io.c` | GPIO relay/opto: gripper relay, reed sensors, proximity | เพิ่ม I/O |
| `Core/Src/current_sensor.c` | WCS1800 ADC: auto-calibrate zero, get amps | แก้ sensor |
| `dashboard/` | Web Serial HTML/JS dashboard (Chrome/Edge เท่านั้น) | แก้ UI |

### New Layers — Pure Logic (unit-testable on PC)

| ไฟล์ | หน้าที่ | สถานะ |
|------|---------|-------|
| `Core/Lib/pid.c/.h` | PID controller: P+I+D, anti-windup, derivative filter | ✅ implement + 19 tests |
| `Core/Lib/scurve.h` | S-curve trajectory planner | 🔲 stub (ดูใน motor_controller.c) |
| `Core/Lib/input_shaper.h` | ZVD input shaper | 🔲 stub |
| `Core/Drivers/encoder.c/.h` | Encoder math: delta, rollover, RPM filter (no HAL) | ✅ implement + 26 tests |
| `Core/Drivers/encoder_hal.c` | HAL adapter: อ่าน TIM3 → calls encoder.c | ✅ implement |
| `Core/Drivers/pwm_output.c/.h` | TIM1 PWM + direction GPIO wrapper | ✅ implement |
| `Core/App/safety.c/.h` | Fault manager: SetFault/ClearFault/IsEStop (PRIMASK-safe) | ✅ implement + 21 tests |
| `Core/App/gripper.c/.h` | Gripper relay: Up/Down/Open/Close + Pick/Place sequences | ✅ implement |
| `Core/App/sequencer.h` | Pick-and-place state machine | 🔲 stub |
| `Core/Middleware/joystick.h` | ESP32 USART3 packet parser | 🔲 stub |
| `Core/Config/config.h` | Flash config save/load | 🔲 stub |

**ModbusBridge เพิ่มใหม่ (จาก remote merge):**
- `ModbusBridge_IsBaseAlive()` — ตรวจว่า heartbeat จาก Base System ยังมาอยู่
- `ModbusBridge_GetPnPState()` — อ่าน PnP task state จาก remote register
- `ModbusBridge_UartErrorRecovery()` — เรียก `Modbus_UartErrorRecovery()` เมื่อ UART error

---

## 5. Modbus Register Map (ครบ)

**Slave ID: 21 | Baud: 19200 8E1 | FC: 0x03 Read, 0x06 Write**

### Write Registers (Base System → Robot)

| Address | Access | ชื่อ | ค่า / หน่วย | หมายเหตุ |
|---------|--------|------|------------|---------|
| `0x00` | RW | Heartbeat | Robot writes **22881** (YA), Base ตอบ **18537** (HI) | Timeout 3s → `base_system_alive = false` |
| `0x01` | W | Command bits | bitmask | bit0=Home, bit1=Jog, bit2=Auto, bit3=SetHome, bit4=Test |
| `0x02` | W | Manual gripper | 0=Up, 1=Down, 2=Open, 4=Close | edge-triggered |
| `0x03` | W | Gripper sequence | 1=Pick, 2=Place | auto-clears |
| `0x04` | W | Config bits | bitmask | bit0=grip_enable สำหรับ PnP |
| `0x05` | W | Jog step | int16, deg | **ค่าถูก invert** ก่อนส่งให้ motor |
| `0x12`–`0x21` | W | PnP positions | int16 × 10, deg | คู่ละ 2 reg: [pick, place] × 5 คู่ |
| `0x22` | W | PnP trigger | pair count (1–5) | เขียนค่า > 0 เพื่อ start, auto-clears |
| `0x24` | W | Point-to-Point | int16, deg | **ค่าถูก invert** เช่นกัน |
| `0x25` | W | Safety | bit0=E-Stop ON, bit1=E-Stop OFF | — |

### Read Registers (Robot → Base System)

| Address | Access | ชื่อ | ค่า / หน่วย | หมายเหตุ |
|---------|--------|------|------------|---------|
| `0x26` | R | Reed sensors | bitmask | bit0=reed_up, bit1=reed_down, bit2=reed_close, bit3=reed_open |
| `0x27` | R | Task state | bitmask | bit0=Homing, bit1=GoPick, bit2=GoPlace, bit3=GoPoint |
| `0x28` | R | Position | int16 × 10, deg | **sign inverted** vs firmware |
| `0x29` | R | Speed | int16 × 10, RPM | **sign inverted** vs firmware |
| `0x30` | R | Acceleration | int16 × 10, RPM/s | **sign inverted** vs firmware |
| `0x31` | R | E-Stop state | 0=OK, 1=stopped | — |

**⚠️ Sign convention:** Base System และ firmware ใช้ทิศทางตรงข้ามกัน  
ทุก register ที่เป็น position/speed/accel จะ **invert เครื่องหมาย** ก่อน read/write

---

## 6. Fault Codes (Bitmask)

```c
FAULT_MOTOR_STALLED   = 0x001  // PWM>25%, vel<0.5 RPM นาน 2s
FAULT_ENCODER_ERROR   = 0x002  // direction inconsistency
FAULT_JOYSTICK_LOST   = 0x004  // ไม่มี packet > 200ms
FAULT_OVER_ROTATION   = 0x008  // เกิน 720° จาก home
FAULT_ESTOP_PHYSICAL  = 0x010  // ปุ่ม E-Stop hardware
FAULT_PROX_LOST       = 0x020  // proximity sensor เปิด
FAULT_ESTOP_JOYSTICK  = 0x040  // joystick กด P/X
FAULT_ESTOP_DASHBOARD = 0x080  // dashboard กด EMERGENCY STOP
FAULT_ESTOP_MODBUS    = 0x100  // Modbus reg 0x25 bit0
FAULT_STARTUP_ESTOP   = 0x200  // power-on latch — cleared by self-test (DIAG)
FAULT_OVERCURRENT     = 0x400  // WCS1800 > OVERCURRENT_LIMIT_AMPS
```

**ใช้ Safety API ใหม่ (Core/App/safety.c):**
```c
Safety_SetFault(FAULT_MOTOR_STALLED);   // ISR-safe, PRIMASK-guarded
Safety_ClearFault(FAULT_MOTOR_STALLED);
if (Safety_IsEStop()) { /* ห้ามขยับ */ }
FaultCode_t f = Safety_GetFaults();     // อ่านทั้ง bitmask
```

*(เดิม: ใช้ FAULT_SET/FAULT_CLR macros ใน motor_controller.h ก็ยังใช้ได้อยู่)*

---

## 7. Telemetry Protocol

Format: `$KEY:VAL,KEY:VAL,...*\r\n`

**Fast (50 Hz):** POS, VEL, ACC, TAR, PWM, MODE, FAULT, PROX, CURR, KTH, KOM, KTL, KIA, GUP, GDN  
**Slow (1 Hz):** PID gains, S-curve limits, safety config  
**Command in:** `$CMD:ESTOP=1*`, `$SET:TARGET=90.0*`, `$SET:SPEED_KP=1.5*`

เพิ่ม field ใหม่: แก้ `telemetry_hub.c` → `Telemetry_Update()` + `dashboard/app.js` → `processPacket()`

---

## 8. How to Tune (ไม่ต้อง recompile)

```
จาก Dashboard → ช่อง SET command:

$SET:SPEED_KP=1.5*      Speed PID Kp
$SET:SPEED_KI=2.0*      Speed PID Ki
$SET:POS_KP=0.4*        Position PID Kp
$SET:TARGET=90.0*       Move to 90 degrees
$SET:SHAPER_EN=1*       Enable ZVD shaper
$SET:SHAPER_WN=12.13*   Shaper natural frequency
$CMD:HOME*              Run homing sequence
$CMD:ESTOP=1*           Emergency stop
$CMD:ESTOP=0*           Clear e-stop
$SET:HOME_OFFSET=5.0*   Shift position-0 by 5 degrees
```

ค่าที่ tune บน Dashboard **หายเมื่อ reset** — ยังไม่มี Flash save (ดู Known Issues)

---

## 9. Known Issues (ปัญหาที่รู้อยู่)

| # | ปัญหา | สาเหตุ | แนวทางแก้ |
|---|-------|--------|-----------|
| 1 | Dashboard ค้าง ต้อง reset browser + board | LPUART1 ใช้ร่วมกัน 2 protocol, mode-switch ล้มเหลวใน field (noise/timing) | ย้าย dashboard ไป USB CDC |
| 2 | Base system connect ไม่ได้เป็นบางครั้ง | Modbus heartbeat miss จาก UART framing error, ไม่มี error recovery | เพิ่ม UART error recovery ใน `modbus_rtu.c` |
| 3 | Joystick ส่งไปแต่ไม่ตอบ | Field EMI ทำให้ USART3 RX error, re-arm ไม่สำเร็จ | เพิ่ม `HAL_UART_ErrorCallback` |
| 4 | Tune PID แล้ว reset หาย | ไม่มี Flash config save | implement `config.c` + EEPROM emulation |
| 5 | Web Serial ทำงานได้แค่บน Chrome/Edge | Web Serial API spec | rewrite dashboard เป็น Python + PyQt6 |

---

## 10. How to Add a New Modbus Register

```c
// 1. modbus_bridge.c → ModbusBridge_HandleCommands() : เพิ่ม write handler
if (register_frame[0xXX].U16 != 0) {
    // do something
    register_frame[0xXX].U16 = 0;  // clear ถ้าเป็น write-once
}

// 2. modbus_bridge.c → ModbusBridge_UpdateRegisters() : เพิ่ม read sync
register_frame[0xXX].U16 = (uint16_t)(some_value * 10);

// 3. อัปเดต Register Map table ใน HANDOFF.md นี้
// 4. แจ้ง Base System team ว่า address คืออะไร
```

---

## 11. Unit Tests

```bash
cd tests/
make test          # build + run ทุก test (117 tests total)
make clean         # ล้าง binary
```

**Test files:**

| ไฟล์ | ครอบคลุม | จำนวน test |
|------|----------|-----------|
| `test_modbus_frame.c` | CRC16, FC03, FC06, **FC16**, bad CRC, exceptions | 51 |
| `test_pid.c` | P/I/D terms, anti-windup, clamping, reset, dt=0 | 19 |
| `test_encoder.c` | position, rollover, noise filter, RPM filter | 26 |
| `test_safety.c` | SetFault, ClearFault, IsEStop, bitmask groups | 21 |
| **รวม** | | **117** |

ต้องการแค่ `gcc` + `libm` บน Mac/Linux — **ไม่ต้องมี board**

**Modules ที่ยังไม่มี test** (HAL-dependent):
- `pwm_output.c` — TIM1 + GPIO, ทดสอบบน hardware
- `gripper.c` — relay + reed switch timeout, ทดสอบบน hardware
- `encoder_hal.c` — TIM3 read, ทดสอบบน hardware

---

## 12. Next Steps (สิ่งที่ยังไม่ได้ทำ)

ตามลำดับ priority:

```
✅ UART error recovery — Modbus_UartErrorRecovery() + mode-aware HAL_UART_ErrorCallback
✅ FC16 Write Multiple Registers — modbus_frame.c + 16 tests pass
✅ Pure modules — pid, encoder, safety, gripper แยกออกจาก motor_controller.c แล้ว
✅ Unit tests — 117 tests, 0 fail

🔲 1. ย้าย dashboard ไป USB CDC (แก้ปัญหา LPUART1 sharing root cause)
🔲 2. Rewrite dashboard เป็น Python + PyQt6 (ออกจาก Chrome-only)
🔲 3. Flash config save — implement Core/Config/config.c (tune แล้วรอด reset)
🔲 4. scurve.c — extract scurve_plan/eval จาก motor_controller.c → Core/Lib/scurve.c
🔲 5. input_shaper.c — extract ZVD shaper → Core/Lib/input_shaper.c
🔲 6. sequencer.c — extract PnP state machine → Core/App/sequencer.c
🔲 7. joystick.c — extract USART3 parser → Core/Middleware/joystick.c
🔲 8. FreeRTOS (เมื่อ module แยกครบแล้ว เพื่อ deterministic scheduling)
```

แผนเต็ม: Second Brain → `projects/pickplace-full-refactor-plan.md`

---

## 13. Contact

ถ้าติดปัญหาเรื่อง:
- **Control tuning** → ดู `params.h` + `CLAUDE.md` section Control Loop
- **Modbus register** → ดู section 5 ในไฟล์นี้
- **Build fail หลัง .ioc change** → Generate Code ใน CubeMX ก่อน build
- **Dashboard ไม่เชื่อมต่อ** → ใช้ Chrome/Edge, เลือก correct COM port, ตรวจว่า mode เป็น Dashboard ไม่ใช่ Modbus
