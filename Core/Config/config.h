/**
 * @file config.h
 * @brief Runtime Configuration — Flash save/load with CRC integrity check.
 *
 * TODO: Implement from scratch.
 *   - Load tuning params from last Flash page on boot
 *   - Fall back to params.h defaults if CRC fails or Flash is blank
 *   - Save triggered by dashboard: $CMD:SAVE* or $CMD:SAVE_CONFIG*
 *   - Reset to defaults: $CMD:RESET_CONFIG*
 *
 * Flash target: last page of STM32G474RE Flash (512 KB total)
 *   Page 127 @ 0x0807F800, size 2 KB
 *   Use HAL_FLASH_Program + HAL_FLASH_Erase (word-by-word)
 *   Or: STM32 EEPROM emulation library (X-CUBE-EEPROM)
 *
 * Problem this solves:
 *   Currently tuning values from params.h reset to defaults on every power cycle.
 *   $SET:SPEED_KP=1.5* sets RAM only — lost on reset.
 *
 * When done: add Core/Config/config.c
 */

#pragma once
#include <stdint.h>

typedef struct {
    /* PID gains */
    float speed_Kp, speed_Ki, speed_Kd;
    float pos_Kp,   pos_Ki,   pos_Kd;
    /* Feedforward */
    float k_vff, k_aff, k_tff;
    /* Motion limits */
    float max_accel, max_jerk;
    float move_speed_coarse;
    /* Input shaper */
    float shaper_omega_n, shaper_zeta;
    /* Homing */
    float home_offset_deg;
    /* Integrity */
    uint32_t crc32;
} SystemConfig_t;

void             Config_Init(void);          /* load from Flash, fallback params.h */
void             Config_Save(void);          /* write to Flash */
void             Config_Reset(void);         /* back to params.h defaults */
SystemConfig_t  *Config_Get(void);           /* pointer to live config */
