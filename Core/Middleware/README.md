# Core/Middleware — Protocol Stacks

Layer ที่นั่งระหว่าง Drivers กับ App — รู้จัก protocol แต่ไม่รู้จัก business logic

## Modules

| File | Status | Role |
|------|--------|------|
| `joystick.h/.c` | 🔲 stub | ESP32 USART3 packet parser |
| `modbus_rtu.h/.c` | ✅ done (refactored) | Modbus RTU state machine |

## กฎ

- `joystick.*` เท่านั้นที่แตะ USART3 RX path
- `modbus_rtu.*` เท่านั้นที่แตะ LPUART1 ใน Modbus mode
