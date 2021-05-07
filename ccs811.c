#include "ccs811.h"

/* Register */
#define STATUS_REG (0x00)
#define MEAS_MODE_REG (0x01)
#define ALG_RESULT_DATA_REG (0x02)
#define RAW_DATA_REG (0x03)
#define ENV_DATA_REG (0x05)
//#define NTC_REG (0x06)
#define THRESHOLDS_REG (0x10)
#define BASELINE_REG (0x11)
#define HW_ID_REG (0x20)
#define HW_VERSION_REG (0x21)
#define FW_BOOT_VERSION_REG (0x23)
#define FW_APP_VERSION_REG (0x24)
#define ERROR_ID_REG (0xE0)
#define APP_START_REG (0xF4)
#define SW_RESET_REG (0xFF)

/* define */
#define ENABLE_ACK_CHECK (true)
#define DISABLE_ACK_CHECK (false)
#define ONE_BYTE (1)
#define READ_ONE_BYTE (1)
#define WRITE_NO_DATA (0)
#define WRITE_ONE_BYTE (1)
#define DRIVE_MODE_SHIFT (4)
#define INTERRUPT_ENABLE_SHIFT (3)
#define THRESH_ENABLE_SHIFT (2)
#define CCS811_HARDWARE_ID (0x81)
#define APP_VALID_MASK (0x10)
#define APP_VALID_SHIFT (4)
#define APP_VALID_TRUE (1)
#define DELAY_100_MS (100 / portTICK_PERIOD_MS)
#define FW_MODE_MASK (0x80)
#define FW_MODE_SHIFT (7)
#define FW_MODE_BOOT (0)
#define FW_MODE_APP (1)
#define UPPER_BYTE_SHIFT (8)
#define MEASURE_DATA_ARRAY_LENGTH (8)
#define ECO2_UPPER_BYTE (0)
#define ECO2_LOWER_BYTE (1)
#define TVOC_UPPER_BYTE (2)
#define TVOC_LOWER_BYTE (3)
#define STATUS_BYTE (4)
#define ERROR_ID_BYTE (5)
#define RAW_DATA_UPPER_BYTE (6)
#define RAW_DATA_LOWER_BYTE (7)
#define TEMPERATURE_OFFSET (25)
#define CCS811_TEMPERATURE_CORRECTION_FACTOR (512)
#define CCS811_HUMIDITY_CORRECTION_FACTOR (512)
#define ENVIRONMENT_VALUE_ARRAY_LENGTH (4)
#define LOWER_BYTE_MASK (0xFF)
#define HUMIDITY_UPPER_BYTE (0)
#define HUMIDITY_LOWER_BYTE (1)
#define TEMPERATURE_UPPER_BYTE (2)
#define TEMPERATURE_LOWER_BYTE (3)
#define THRESHOLD_ARRAY_LENGTH (5)
#define LOWER_THRESH_UPPER_BYTE (0)
#define LOWER_THRESH_LOWER_BYTE (1)
#define UPPER_THRESH_UPPER_BYTE (2)
#define UPPER_THRESH_LOWER_BYTE (3)
#define HYSTERESIS_BYTE (4)
#define BOOTLOADER_VERSION_ARRAY_LENGTH (2)
#define BOOTLOADER_UPPER_BYTE (0)
#define BOOTLOADER_LOWER_BYTE (1)
#define APP_VERSION_ARRAY_LENGTH (2)
#define APP_VERSION_UPPER_BYTE (0)
#define APP_VERSION_LOWER_BYTE (1)
#define MSG_INVALID_ERROR_MASK (0x01)
#define READ_REG_INVALID_ERROR_MASK (0x02)
#define READ_REG_INVALID_ERROR_SHIFT (1)
#define MEASMODE_INVALID_ERROR_MASK (0x04)
#define MEASMODE_INVALID_ERROR_SHIFT (2)
#define MAX_RESISTANCE_ERROR_MASK (0x08)
#define MAX_RESISTANCE_ERROR_SHIFT (3)
#define HEATER_FAULT_ERROR_MASK (0x10)
#define HEATER_FAULT_ERROR_SHIFT (4)
#define HEATER_SUPPLY_ERROR_MASK (0x20)
#define HEATER_SUPPLY_ERROR_SHIFT (5)
#define SOFTWARE_RESET_ARRAY_LENGTH (4)
#define SOFTWARE_RESET_FIRST_BYTE (0x11)
#define SOFTWARE_RESET_SECOND_BYTE (0xE5)
#define SOFTWARE_RESET_THIRD_BYTE (0x72)
#define SOFTWARE_RESET_FOURTH_BYTE (0x8A)
#define CCS811_DEV_ADDR_SHIFT (1)
#define WAIT_UNTIL_10_MS (10 / portTICK_PERIOD_MS)

static ccs811_config_t *config_table;

/* Static functions */
static int8_t ccs811_set_measure_setting(ccs811_config_t *ccs811_config)
{
    int8_t status = CCS811_SUCCESS;
    uint8_t write_data = 0;

    write_data = (ccs811_config->drive_mode << DRIVE_MODE_SHIFT)
               | (ccs811_config->interruput_enable << INTERRUPT_ENABLE_SHIFT)
               | (ccs811_config->thresh_enable << THRESH_ENABLE_SHIFT);
    status = ccs811_write_reg(MEAS_MODE_REG, &write_data, WRITE_ONE_BYTE);

    return status;
}

/* Public functions */
/* Perfome CCS811 initialization.
 * Basically, I followed the procedure in the datasheet, but only added a software reset. */
int8_t ccs811_init(ccs811_config_t *ccs811_config)
{
    /* argument check */
    if (ccs811_config == NULL) {
        return CCS811_ERROR;
    }
    config_table = ccs811_config;

    int8_t status = CCS811_SUCCESS;
    uint8_t hardware_id = 0;
    uint8_t status_reg_val = 0;

    /* Check the hardware ID of the CCS811 */
    status = ccs811_get_hardware_id(&hardware_id);
    if (status == CCS811_SUCCESS) {
        if (hardware_id != CCS811_HARDWARE_ID) {
            printf("This device is not CCS811.\n");
            goto ERROR;
        }
        status = ccs811_software_reset();
    } else {
        printf("Could not read hardware_id register.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        /* Get the status to check whether sensor is in bootloader mode. */
        status = ccs811_read_reg(STATUS_REG, &status_reg_val, READ_ONE_BYTE);
    } else {
        printf("Could not resrt the sensor.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        /* Check the application firmware */
        if (((status_reg_val & APP_VALID_MASK) >> APP_VALID_SHIFT != APP_VALID_TRUE)) {
            printf("Application firmware is not loaded!\n");
            goto ERROR;
        }
        /* Switch to application mode */
        status = ccs811_write_reg(APP_START_REG, &status_reg_val, WRITE_NO_DATA);
    } else {
        printf("Could not read status register.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        /* Wait 100ms after starting the app. */
        vTaskDelay(DELAY_100_MS);
        /* Check mode  */
        status = ccs811_read_reg(STATUS_REG, &status_reg_val, READ_ONE_BYTE);
    } else {
        printf("Could not start application.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        if (((status_reg_val & FW_MODE_MASK) >> FW_MODE_SHIFT) != FW_MODE_APP) {
            printf("Firmware is not application mode now,\n");
            goto ERROR;
        }
        status = ccs811_set_measure_setting(config_table);
    } else {
        printf("Could not read status register!\n");
        goto ERROR;
    }

    if (status != CCS811_SUCCESS) {
        printf("Could not set measurement settings.\n");
        goto ERROR;
    }

    return status;

ERROR:
    return CCS811_ERROR;
}

int8_t ccs811_get_measure_data(ccs811_measure_data_t *ccs811_measure_data)
{
    /* argument check */
    if (ccs811_measure_data == NULL) {
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;
    uint8_t data[MEASURE_DATA_ARRAY_LENGTH] = {0};

    status = ccs811_read_reg(ALG_RESULT_DATA_REG, &data[0], sizeof(data) / sizeof(data[0]));
    if (status == CCS811_SUCCESS) {
        ccs811_measure_data->eco2 = (((uint16_t)data[ECO2_UPPER_BYTE] << UPPER_BYTE_SHIFT)
                                    | (uint16_t)data[ECO2_LOWER_BYTE]);
        ccs811_measure_data->tvoc = (((uint16_t)data[TVOC_UPPER_BYTE] << UPPER_BYTE_SHIFT)
                                    | (uint16_t)data[TVOC_LOWER_BYTE]);
        ccs811_measure_data->status = data[STATUS_BYTE];
        ccs811_measure_data->error_id = data[ERROR_ID_BYTE];
        ccs811_measure_data->raw_data = (((uint16_t)data[RAW_DATA_UPPER_BYTE] << UPPER_BYTE_SHIFT)
                                        | (uint16_t)data[RAW_DATA_LOWER_BYTE]);
        printf("read measure data is succeeded.\n");
    } else {
        printf("read measure data is failed.\n");
        return CCS811_ERROR;
    }

    return status;
}

int8_t ccs811_write_environment_value(float temperature, float humidity)
{
    uint8_t status = CCS811_SUCCESS;
    /* Correct humidity and temperature according to register specifications. */
    uint16_t ccs811_temp = (uint16_t)((temperature + TEMPERATURE_OFFSET)
                                      * CCS811_TEMPERATURE_CORRECTION_FACTOR);
    uint16_t ccs811_hum = (uint16_t)(humidity * CCS811_HUMIDITY_CORRECTION_FACTOR);

    uint8_t data[ENVIRONMENT_VALUE_ARRAY_LENGTH];
    data[HUMIDITY_UPPER_BYTE] = ccs811_hum << UPPER_BYTE_SHIFT;
    data[HUMIDITY_LOWER_BYTE] = ccs811_hum & LOWER_BYTE_MASK;
    data[TEMPERATURE_UPPER_BYTE] = ccs811_temp << UPPER_BYTE_SHIFT;
    data[TEMPERATURE_LOWER_BYTE] = ccs811_temp & LOWER_BYTE_MASK;

    status = ccs811_write_reg(ENV_DATA_REG, data, sizeof(data) / sizeof(data[0]));

    if (status != CCS811_SUCCESS) {
        printf("write environment value is failed.\n");
    }

    return status;
}

int8_t ccs811_write_eco2_threshold(ccs811_eco2_threshold_t *eco2_threshold)
{
    /* argument check */
    if (eco2_threshold == NULL) {
        printf("eco2_threshold is NULL.\n");
        return CCS811_ERROR;
    }

    uint8_t status = CCS811_SUCCESS;
    uint8_t data[THRESHOLD_ARRAY_LENGTH] = {0};
    data[LOWER_THRESH_UPPER_BYTE] = (uint8_t)(eco2_threshold->lower_thresh >> UPPER_BYTE_SHIFT);
    data[LOWER_THRESH_LOWER_BYTE] = (uint8_t)(eco2_threshold->lower_thresh & LOWER_BYTE_MASK);
    data[UPPER_THRESH_UPPER_BYTE] = (uint8_t)(eco2_threshold->upper_thresh >> UPPER_BYTE_SHIFT);
    data[UPPER_THRESH_LOWER_BYTE] = (uint8_t)(eco2_threshold->upper_thresh & LOWER_BYTE_MASK);
    data[HYSTERESIS_BYTE] = eco2_threshold->hysteresis;

    status = ccs811_write_reg(THRESHOLDS_REG, data, sizeof(data) / sizeof(data[0]));

    if (status != CCS811_SUCCESS) {
        printf("write eco2 threshold is failed\n");
    }

    return status;
}

int8_t ccs811_get_hardware_id(uint8_t *hardware_id)
{
    /* argument check */
    if (hardware_id == NULL) {
        printf("hardware_id is NULL.\n");
        return CCS811_ERROR;
    }

    uint8_t status = CCS811_SUCCESS;
    status = ccs811_read_reg(HW_ID_REG, hardware_id, READ_ONE_BYTE);

    if (status != CCS811_SUCCESS) {
        printf("Get hardware ID is failed.\n");
    }

    return status;
}

int8_t ccs811_get_hardware_version(uint8_t *hardware_version)
{
    /* argument check */
    if (hardware_version == NULL) {
        printf("hardware_version is NULLl\n");
        return CCS811_ERROR;
    }

    uint8_t status = CCS811_SUCCESS;
    status = ccs811_read_reg(HW_VERSION_REG, hardware_version, READ_ONE_BYTE);

    if (status != CCS811_SUCCESS) {
        printf("Get hardware version is failed.\n");
    }

    return status;
}

int8_t ccs811_get_firmware_bootloader_version(uint16_t *bootloader_version)
{
    /* argument check */
    if (bootloader_version == NULL) {
        printf("bootloader_version is NULL\n");
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;
    uint8_t data[BOOTLOADER_VERSION_ARRAY_LENGTH] = {0};

    status = ccs811_read_reg(FW_BOOT_VERSION_REG, data, sizeof(data) / sizeof(data[0]));
    *bootloader_version = (uint16_t)data[BOOTLOADER_UPPER_BYTE] << UPPER_BYTE_SHIFT
                         | data[BOOTLOADER_LOWER_BYTE];

    if (status != CCS811_SUCCESS) {
        printf("Get firmware bootloader version is failed\n");
    }

    return status;
}

int8_t ccs811_get_firmware_app_version(uint16_t *app_version)
{
    /* argument check */
    if (app_version == NULL) {
        printf("app_version is NULL\n");
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;
    uint8_t data[APP_VERSION_ARRAY_LENGTH] = {0};

    status = ccs811_read_reg(FW_APP_VERSION_REG, data, sizeof(data) / sizeof(data[0]));
    *app_version = (uint16_t)data[APP_VERSION_UPPER_BYTE] << UPPER_BYTE_SHIFT
                 | data[APP_VERSION_LOWER_BYTE];

    if (status != CCS811_SUCCESS) {
        printf("Get firmware application version is failed\n");
    }

    return status;
}

int8_t ccs811_get_error_id(uint8_t *error_id)
{
    /* argument check */
    if (error_id == NULL) {
        printf("error_id is NULL\n");
    }

    uint8_t status = CCS811_SUCCESS;
    status = ccs811_read_reg(ERROR_ID_REG, error_id, READ_ONE_BYTE);
    if (status == CCS811_SUCCESS) {
        if (*error_id & MSG_INVALID_ERROR_MASK) {
            printf("MSG INVALID error has occured.\n");
        }
        if (*error_id & READ_REG_INVALID_ERROR_MASK >> READ_REG_INVALID_ERROR_SHIFT) {
            printf("READ_REG INVALID error has occured.\n");
        }
        if (*error_id & MEASMODE_INVALID_ERROR_MASK >> MEASMODE_INVALID_ERROR_SHIFT) {
            printf("MEASMODE INVALID error has occured.\n");
        }
        if (*error_id & MAX_RESISTANCE_ERROR_MASK >> MAX_RESISTANCE_ERROR_SHIFT) {
            printf("MAX RESISTANCE error has occured.\n");
        }
        if (*error_id & HEATER_FAULT_ERROR_MASK >> HEATER_FAULT_ERROR_SHIFT) {
            printf("HEATER FAULT error has occured.\n");
        }
        if (*error_id & HEATER_SUPPLY_ERROR_MASK >> HEATER_SUPPLY_ERROR_SHIFT) {
            printf("HEATER SUPPLY error has occured.\n");
        }
    } else {
        printf("Get error id is failed.\n");
    }

    return status;
}

int8_t ccs811_software_reset(void)
{
    int8_t status = CCS811_SUCCESS;
    static uint8_t software_reset[SOFTWARE_RESET_ARRAY_LENGTH] = {
        SOFTWARE_RESET_FIRST_BYTE,
        SOFTWARE_RESET_SECOND_BYTE,
        SOFTWARE_RESET_THIRD_BYTE,
        SOFTWARE_RESET_FOURTH_BYTE
    };
    status = ccs811_write_reg(SW_RESET_REG,
                              software_reset,
                              sizeof(software_reset) / sizeof(software_reset[0]));

    if (status == CCS811_SUCCESS) {
        vTaskDelay(DELAY_100_MS);
    } else {
        printf("Could not resrt the sensor.\n");
    }

    return status;
}

/* Writing to the CCS811 register. */
int8_t ccs811_write_reg(uint8_t reg_address, uint8_t *data, uint8_t size)
{
    /* argument check */
    if (data == NULL) {
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    status = i2c_master_start(cmd_handle);

    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(
            cmd_handle,
            (CCS811_DEV_ADDR << CCS811_DEV_ADDR_SHIFT) | I2C_MASTER_WRITE,
            ENABLE_ACK_CHECK
        );
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle, reg_address, ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    if (size != 0) {
        if (status == CCS811_SUCCESS) {
            status = i2c_master_write(cmd_handle, data, size, ENABLE_ACK_CHECK);
        } else {
            goto ERROR;
        }
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_stop(cmd_handle);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, WAIT_UNTIL_10_MS);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        i2c_cmd_link_delete(cmd_handle);
    } else {
        goto ERROR;
    }

    return status;

ERROR:
    return CCS811_ERROR;
}

/* Reading from the CCS811 register. */
int8_t ccs811_read_reg(uint8_t reg_address, uint8_t *data, uint8_t size)
{
    /* argument check */
    if (data == NULL) {
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    status = i2c_master_start(cmd_handle);
    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(
            cmd_handle,
            (CCS811_DEV_ADDR << CCS811_DEV_ADDR_SHIFT) | I2C_MASTER_WRITE,
            ENABLE_ACK_CHECK
        );
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle, reg_address, ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_start(cmd_handle);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(
            cmd_handle,
            (CCS811_DEV_ADDR << CCS811_DEV_ADDR_SHIFT) | I2C_MASTER_READ,
            ENABLE_ACK_CHECK
        );
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        if (size > ONE_BYTE) {
            status = i2c_master_read(cmd_handle, data, size-ONE_BYTE, I2C_MASTER_ACK);
        }
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_read_byte(cmd_handle, data+size-ONE_BYTE, I2C_MASTER_NACK);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_stop(cmd_handle);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, WAIT_UNTIL_10_MS);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        i2c_cmd_link_delete(cmd_handle);
    } else {
        goto ERROR;
    }

    return status;

ERROR:
    return CCS811_ERROR;
}

