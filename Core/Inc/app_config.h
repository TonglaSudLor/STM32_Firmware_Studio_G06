/**
 * @file config.h
 * @brief System configuration and Flash storage management.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Configuration structure stored in Flash.
 * @note Must be 8-byte aligned for STM32G4 Flash programming.
 * @note Total size must be less than 2KB (one Flash page).
 */
typedef struct {
    uint32_t magic;            /**< Magic number to verify config existence (0xDEADBEEF) */
    uint32_t version;          /**< Config structure version for migration */
    
    // PID Gains
    float speed_Kp, speed_Ki, speed_Kd;
    float pos_Kp,   pos_Ki,   pos_Kd;
    
    // Feedforward Gains
    float k_vff, k_aff, k_tff;
    
    // Motion Limits
    float move_speed_coarse;
    float max_accel;
    float max_jerk;
    
    // System Settings
    float home_offset_deg;
    float min_pwm;
    
    // ZVD Shaper
    float shaper_omega_n;
    float shaper_zeta;
    bool  shaper_enable;
    
    uint32_t crc32;            /**< CRC32 integrity check (must be the last field) */
} __attribute__((aligned(8))) SystemConfig_t;

/**
 * @brief Initialize configuration system.
 *        Loads from Flash if valid, otherwise falls back to defaults.
 */
void Config_Init(void);

/**
 * @brief Save current system parameters to Flash.
 */
bool Config_Save(void);

/**
 * @brief Restore parameters to compiled-in defaults (factory reset).
 */
void Config_ResetDefaults(void);

/**
 * @brief Get pointer to the active runtime configuration.
 */
SystemConfig_t* Config_Get(void);

#endif /* CONFIG_H */
