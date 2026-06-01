# Core/Config — Runtime Configuration

Tuning params ที่รอดจาก power cycle — save/load จาก Flash

## Modules

| File | Status | Role |
|------|--------|------|
| `config.h/.c` | 🔲 stub | Flash save/load, CRC integrity |
| `params.h` | ✅ (อยู่ที่ Core/Inc/) | Compile-time defaults — ไม่ย้าย |

## วิธีใช้ (เมื่อ implement แล้ว)

```c
Config_Init();              // boot: โหลด Flash หรือ fallback params.h
SystemConfig_t *cfg = Config_Get();
cfg->speed_Kp = 1.5f;      // แก้ RAM
Config_Save();              // flush ไป Flash
```

Dashboard commands (ยังไม่ implement):
```
$CMD:SAVE*          บันทึก config ปัจจุบันลง Flash
$CMD:RESET_CONFIG*  กลับ defaults จาก params.h
```

## Flash target

STM32G474RE: 512 KB Flash  
Page 127 @ `0x0807F800`, size 2 KB → ใช้เก็บ `SystemConfig_t`
