/**
 * @file config.c
 * @brief Implementation of Flash-based configuration storage.
 */

#include "config.h"
#include "params.h"
#include "motor_controller.h"
#include "main.h"
#include <string.h>

/* --- Private Definitions --- */
#define CONFIG_MAGIC           0xDEADBEEFu
#define CONFIG_VERSION         1u
#define FLASH_CONFIG_ADDRESS   0x0807F800u  /* Last page of 512KB Flash (Page 255) */
#define FLASH_CONFIG_PAGE      255u

static SystemConfig_t runtime_config;

/* --- Internal Helpers --- */
static uint32_t Calculate_CRC32(const uint8_t *data, size_t len)
{
    // Simple software CRC32 (Standard Ethernet polynomial)
    // In production, use the STM32 Hardware CRC unit for speed.
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
            else crc >>= 1;
        }
    }
    return ~crc;
}

static void Load_Defaults(void)
{
    runtime_config.magic = CONFIG_MAGIC;
    runtime_config.version = CONFIG_VERSION;
    
    runtime_config.speed_Kp = DEFAULT_SPEED_KP;
    runtime_config.speed_Ki = DEFAULT_SPEED_KI;
    runtime_config.speed_Kd = DEFAULT_SPEED_KD;
    
    runtime_config.pos_Kp = DEFAULT_POS_KP;
    runtime_config.pos_Ki = DEFAULT_POS_KI;
    runtime_config.pos_Kd = DEFAULT_POS_KD;
    
    runtime_config.k_vff = DEFAULT_K_VFF;
    runtime_config.k_aff = DEFAULT_K_AFF;
    runtime_config.k_tff = DEFAULT_K_TFF;
    
    runtime_config.move_speed_coarse = MOVE_SPEED_COARSE;
    runtime_config.max_accel = DEFAULT_MAX_ACCEL;
    runtime_config.max_jerk = DEFAULT_MAX_JERK;
    
    runtime_config.home_offset_deg = DEFAULT_HOME_OFFSET;
    runtime_config.min_pwm = DEFAULT_MIN_PWM;
    
    runtime_config.shaper_omega_n = DEFAULT_SHAPER_OMEGA_N;
    runtime_config.shaper_zeta = DEFAULT_SHAPER_ZETA;
    runtime_config.shaper_enable = false;
}

/* --- Public Implementation --- */

void Config_Init(void)
{
    // 1. Read from Flash
    const SystemConfig_t *flash_ptr = (const SystemConfig_t*)FLASH_CONFIG_ADDRESS;
    
    // 2. Verify Magic and CRC
    uint32_t data_len = sizeof(SystemConfig_t) - sizeof(uint32_t);
    uint32_t computed_crc = Calculate_CRC32((const uint8_t*)flash_ptr, data_len);
    
    if (flash_ptr->magic == CONFIG_MAGIC && flash_ptr->crc32 == computed_crc) {
        // Valid config found, copy to RAM
        memcpy(&runtime_config, flash_ptr, sizeof(SystemConfig_t));
    } else {
        // Corrupt or empty, load defaults
        Load_Defaults();
    }
    
    // 3. Apply loaded config to active tuning params
    tuning.speed_Kp = runtime_config.speed_Kp;
    tuning.speed_Ki = runtime_config.speed_Ki;
    tuning.speed_Kd = runtime_config.speed_Kd;
    tuning.pos_Kp   = runtime_config.pos_Kp;
    tuning.pos_Ki   = runtime_config.pos_Ki;
    tuning.pos_Kd   = runtime_config.pos_Kd;
    tuning.K_vff    = runtime_config.k_vff;
    tuning.K_aff    = runtime_config.k_aff;
    tuning.K_tff    = runtime_config.k_tff;
    tuning.move_speed_coarse = runtime_config.move_speed_coarse;
    tuning.max_accel = runtime_config.max_accel;
    tuning.max_jerk  = runtime_config.max_jerk;
    tuning.home_offset_deg = runtime_config.home_offset_deg;
    tuning.min_pwm   = runtime_config.min_pwm;
    tuning.shaper_omega_n = runtime_config.shaper_omega_n;
    tuning.shaper_zeta    = runtime_config.shaper_zeta;
    tuning.shaper_enable  = runtime_config.shaper_enable;
}

bool Config_Save(void)
{
    // 1. Snapshot current tuning values into runtime_config
    runtime_config.speed_Kp = tuning.speed_Kp;
    runtime_config.speed_Ki = tuning.speed_Ki;
    runtime_config.speed_Kd = tuning.speed_Kd;
    runtime_config.pos_Kp   = tuning.pos_Kp;
    runtime_config.pos_Ki   = tuning.pos_Ki;
    runtime_config.pos_Kd   = tuning.pos_Kd;
    runtime_config.k_vff    = tuning.K_vff;
    runtime_config.k_aff    = tuning.K_aff;
    runtime_config.k_tff    = tuning.K_tff;
    runtime_config.move_speed_coarse = tuning.move_speed_coarse;
    runtime_config.max_accel = tuning.max_accel;
    runtime_config.max_jerk  = tuning.max_jerk;
    runtime_config.home_offset_deg = tuning.home_offset_deg;
    runtime_config.min_pwm   = tuning.min_pwm;
    runtime_config.shaper_omega_n = tuning.shaper_omega_n;
    runtime_config.shaper_zeta    = tuning.shaper_zeta;
    runtime_config.shaper_enable  = tuning.shaper_enable;
    
    // 2. Re-calculate CRC
    uint32_t data_len = sizeof(SystemConfig_t) - sizeof(uint32_t);
    runtime_config.crc32 = Calculate_CRC32((const uint8_t*)&runtime_config, data_len);
    
    // 3. Flash Programming Sequence
    HAL_FLASH_Unlock();
    
    // Erase Page
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks     = FLASH_BANK_1;
    erase_init.Page      = FLASH_CONFIG_PAGE;
    erase_init.NbPages   = 1;
    
    uint32_t page_error = 0;
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    
    // Program in double-word (64-bit) chunks
    uint64_t *data_ptr = (uint64_t*)&runtime_config;
    uint32_t address = FLASH_CONFIG_ADDRESS;
    size_t iterations = sizeof(SystemConfig_t) / 8;
    
    for (size_t i = 0; i < iterations; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_ptr[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        address += 8;
    }
    
    HAL_FLASH_Lock();
    return true;
}

void Config_ResetDefaults(void)
{
    Load_Defaults();
    Config_Save();
}

SystemConfig_t* Config_Get(void)
{
    return &runtime_config;
}
