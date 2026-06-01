# Core/Lib — Pure Math Library

**กฎ: ไฟล์ในนี้ต้องไม่มี `#include "stm32*.h"` เลย**

ทุก module ต้อง unit test บน PC ได้ด้วย `gcc` โดยไม่ต้องมี board

## Modules

| File | Status | Extract from |
|------|--------|-------------|
| `pid.h/.c` | 🔲 stub | `motor_controller.c` — PID update logic |
| `scurve.h/.c` | 🔲 stub | `motor_controller.c` — S-curve trajectory step |
| `input_shaper.h/.c` | 🔲 stub | `motor_controller.c` — ZVD ring buffer |
| `kalman_lib.h/.c` | 🔲 stub | `kalman.c` — remove HAL_GetTick(), inject dt |
| `modbus_frame.h/.c` | ✅ done | (แยกออกมาแล้ว) |

## วิธีเพิ่ม module ใหม่

1. สร้าง `xxx.h` — interface เท่านั้น ไม่มี HAL
2. สร้าง `xxx.c` — implementation
3. สร้าง `../../tests/test_xxx.c` — unit tests
4. เพิ่มใน `../../tests/Makefile`
5. `make test` ต้องผ่านก่อน commit
