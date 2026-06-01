# Core/App — Application Logic

Business logic — รู้ว่าหุ่นยนต์ทำอะไร แต่คุยกับ hardware ผ่าน Drivers/Lib เท่านั้น

## Modules

| File | Status | Extract from |
|------|--------|-------------|
| `safety.h/.c` | 🔲 stub | `motor_controller.h/.c` — FAULT_SET/CLR, fault checks |
| `gripper.h/.c` | 🔲 stub | `motor_controller.c` — Gripper_Up/Down/Open/Close |
| `sequencer.h/.c` | 🔲 stub | `modbus_bridge.c` — PnP_StateMachine() |

## กฎ

- `safety.*` เท่านั้นที่แก้ fault_code โดยตรง
- `sequencer.*` เรียก `Motor_MoveToPosition()` และ `Gripper_*()` เท่านั้น
- ไม่แตะ HAL ตรง — ใช้ผ่าน Drivers
