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

/* Measurement completion interrupt setting
 * When ENABLE is set, [nINT] is driven low at each completion of measurement.
 * The low drive of [nINT] is automatically released by reading ALG_RESULT_DATA. */
enum ccs811_interrupt_enable {
    CCS811_INTERRUPT_DISABLE = 0,
    CCS811_INTERRUPT_ENABLE,
};

/* Setting of interrupt by eCO2 absolute value.
 * When ENABLE is set, [nINT] is driven low at each completion of measurement.
 * The low drive of [nINT] is automatically released by reading ALG_RESULT_DATA. */
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

/**
 * @brief Perfome CCS811 initialization
 *
 * @param ccs811_config measurement settings
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_init(ccs811_config_t *ccs811_config);

/**
 * @brief Perfome CCS811 exit. At present, nothing is being done.
 *
 * @return
 *      - CCS811_SUCCESS   Success
 */
int8_t ccs811_exit();

/**
 * @brief Acquire the measurement value from the data register.
 *
 * @param measure_data Pointer to the data structure that stores the measurement data
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_get_measure_data(ccs811_measure_data_t *measure_data);

/**
 * @brief Write down the current temperature and humidity to compensate for the measured values.
 *
 * @param temperature Current temperature
 * @param humidity Current humidity
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_write_environment_value(float temperature, float humidity);

/**
 * @brief Write the upper and lower CO2 limits.
 *        @note Measurement a value outside this range is measured,
 *              an interpolation is generated.
 *
 * @param eco2_threshold Pointer to the CO2 threshold data structure
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_write_eco2_threshold(ccs811_eco2_threshold_t *eco2_threshold);

/**
 * @brief Get hardware id
 *
 * @param hardware_id Pointer to the data that stores the hardware id
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_get_hardware_id(uint8_t *hardware_id);

/**
 * @brief Get hardware version
 *
 * @param hardware_version Pointer to the data that stores the hardware version
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_get_hardware_version(uint8_t *hardware_version);

/**
 * @brief Get firmware bootloader version
 *
 * @param bootloader_version Pointer to the data that stores the firmware bootloader version
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_get_firmware_bootloader_version(uint16_t *bootloader_version);

/**
 * @brief Get firmware application version
 *
 * @param app_version Pointer to the data that stores the firmware application version
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_get_firmware_app_version(uint16_t *app_version);

/**
 * @brief Get error id
 *
 * @param bootloader_version Pointer to the data that stores the error id
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_get_error_id(uint8_t *error_id);

/**
 * @brief Software reset
 *
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_software_reset();

/**
 * @brief Reading register values
 *
 * @param reg_addr Register address
 * @param data Pointer to the data array that stores register value
 * @param size Number of bytes to read
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t size);

/**
 * @brief Writing data to registers
 *
 * @param reg_addr Register address
 * @param data Pointer to the data array that stores writing data
 * @param size Number of bytes to write
 * @return
 *      - CCS811_SUCCESS   Success
 *      - CCS811_ERROR   Error
 */
int8_t ccs811_write_reg(uint8_t reg_addr, uint8_t *data, uint8_t size);

#endif /* __CCS811_H__ */