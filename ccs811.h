#ifndef __CCS811_H__
#define __CCS811_H__

#include <stdint.h>
#include "driver/i2c.h"

#define CCS811_DEV_ADDR (0x5A)

enum ccs811_status {
    CCS811_ERROR = -1,
    CCS811_SUCCESS,
};

enum ccs811_drive_mode {
    CCS811_DRIVE_MODE_IDLE = 0,
    CCS811_DRIVE_MODE_CONSTANT_POWER_1000MS,
    CCS811_DRIVE_MODE_PULSE_HEATING,
    CCS811_DRIVE_MODE_LOW_POWER_PULSE_HEATING,
    CCS811_DRIVE_MODE_CONSTANT_POWER_250MS,
};

/* 測定完了割り込みの設定。
 * ENABLEとした場合、測定完了ごとに[nINT]がlowに駆動される。
 * ALG_RESULT_DATAをreadすることで自動的に[nINT]のlow駆動は解除される。 */
enum ccs811_interrupt_enable {
    CCS811_INTERRUPT_DISABLE = 0,
    CCS811_INTERRUPT_ENABLE,
};

/* eCO2絶対値による割り込みの設定。
 * ENABLEとした場合、測定値がTHRESHOLDSレジスタに設定された値を上回ると[nINT]がlowに駆動される。
 * ALG_RESULT_DATAをreadすることで自動的に[nINT]のlow駆動は解除される。 */
enum ccs811_thresh_enable
{
    CCS811_THRESH_ENABLE = 0,
    CCS811_THRESH_DISABLE,
};

typedef struct ccs811_config {
    uint8_t drive_mode;
    uint8_t interruput_enable;
    uint8_t thresh_enable;
} ccs811_config_t;

typedef struct ccs811_measure_data {
    uint16_t eco2;
    uint16_t tvoc;
    uint8_t status;
    uint8_t error_id;
    uint16_t raw_data;
} ccs811_measure_data_t;

typedef struct ccs811_environment_data {
    int32_t temperature;
    uint32_t humidity;
} ccs811_environment_data_t;

typedef struct ccs811_eco2_threshold {
    uint16_t lower_thresh;
    uint16_t upper_thresh;
    uint8_t hysteresis;
} ccs811_eco2_threshold_t;

int8_t ccs811_init(ccs811_config_t *ccs811_config);
int8_t ccs811_exit();
int8_t ccs811_get_measure_data(ccs811_measure_data_t *measure_data);
int8_t ccs811_write_environment_value(float temperature, float humidity);
int8_t ccs811_write_eco2_threshold(ccs811_eco2_threshold_t *eco2_threshold);
int8_t ccs811_get_hardware_id(uint8_t *hardware_id);
int8_t ccs811_get_hardware_version(uint8_t *hardware_version);
int8_t ccs811_get_firmware_bootloader_version(uint16_t *bootloader_version);
int8_t ccs811_get_firmware_app_version(uint16_t *app_version);
int8_t ccs811_get_error_id(uint8_t *error_id);
int8_t ccs811_app_verify();
int8_t ccs811_software_reset();
int8_t ccs811_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t size);
int8_t ccs811_write_reg(uint8_t reg_addr, uint8_t *data, uint8_t size);

#endif /* __CCS811_H__ */