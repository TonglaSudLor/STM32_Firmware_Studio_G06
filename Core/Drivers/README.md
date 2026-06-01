# Core/Drivers — HAL Wrappers

HAL (`stm32g4xx_hal.h`) อยู่ที่นี่ที่เดียว — ไฟล์อื่นในโปรเจกต์ไม่ควรแตะ HAL ตรง

## Modules

| File | Status | Wraps |
|------|--------|-------|
| `encoder.h/.c` | 🔲 stub | TIM3 quadrature encoder |
| `pwm_output.h/.c` | 🔲 stub | TIM1 H-bridge PWM |

## กฎ

- `Drivers/encoder.*` เท่านั้นที่แตะ `htim3`
- `Drivers/pwm_output.*` เท่านั้นที่แตะ `htim1` PWM registers
- ไฟล์ใน `App/` และ `Lib/` เรียกผ่าน Driver API เท่านั้น
