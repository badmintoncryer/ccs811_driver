#include "ccs811.h"

#define ENABLE_ACK_CHECK (true)
#define DISABLE_ACK_CHECK (false)

static ccs811_config_t *config_table;

/* Static functions */
static int8_t ccs811_set_measure_setting(ccs811_config_t *ccs811_config);

static int8_t ccs811_set_measure_setting(ccs811_config_t *ccs811_config)
{
    int8_t status = CCS811_SUCCESS;
    uint8_t write_data = 0;

    write_data = (ccs811_config->drive_mode << 4)
               | (ccs811_config->interruput_enable << 3)
               | (ccs811_config->thresh_enable << 2);
    status = ccs811_write_reg(0x01, &write_data, 1);

    return status;
}

/* CCS811の初期化を行う。 */
int8_t ccs811_init(ccs811_config_t *ccs811_config)
{
    if (ccs811_config == NULL) {
        return CCS811_ERROR;
    }
    config_table = ccs811_config;

    int8_t status = CCS811_SUCCESS;
    uint8_t hardware_id = 0;
    uint8_t status_reg_val = 0;

    status = ccs811_get_hardware_id(&hardware_id);
    if (status == CCS811_SUCCESS) {
        if (hardware_id != 0x81) {
            printf("This device is not CCS811!!\n");
            goto ERROR;
        }
        /* Doing a software reset */
        static uint8_t software_reset[4] = { 0x11, 0xE5, 0x72, 0x8A };
        status = ccs811_write_reg(0xFF, software_reset, 4);
    } else {
        printf("Could not read hardware_id register.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        /* Wait 100ms after the reset */
        vTaskDelay(100 / portTICK_PERIOD_MS);
        /* Get the status to check whether sensor is in bootloader mode. */
        status = ccs811_read_reg(0x00, &status_reg_val, 1);
    } else {
        printf("Could not resrt the sensor.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        if (((status_reg_val & 0x10) >> 4) != 1) {
            printf("Application firmware is not loaded!\n");
            goto ERROR;
        }
        /* Switch to application mode */
        status = ccs811_write_reg(0xF4, &status_reg_val, 0);
    } else {
        printf("Could not read status register.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        /* Wait 100ms after starting the app. */
        vTaskDelay(100 / portTICK_PERIOD_MS);
        status = ccs811_read_reg(0x00, &status_reg_val, 1);
    } else {
        printf("Could not start application.\n");
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        if (((status_reg_val & 0x80) >> 7) != 1) {
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
    if (ccs811_measure_data == NULL) {
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;
    uint8_t data[8] = {0};

    status = ccs811_read_reg(0x02, &data[0], 8);
    if (status == CCS811_SUCCESS) {
        ccs811_measure_data->eco2 = (((uint16_t)data[0] << 8) | (uint16_t)data[1]);
        ccs811_measure_data->tvoc = (((uint16_t)data[2] << 8) | (uint16_t)data[3]);
        ccs811_measure_data->status = data[4];
        ccs811_measure_data->error_id = data[5];
        ccs811_measure_data->raw_data = (((uint16_t)data[6] << 8) | (uint16_t)data[7]);
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
    uint16_t ccs811_temp = (uint16_t)((temperature + 25) * 512);
    uint16_t ccs811_hum = (uint16_t)(humidity * 512);

    uint8_t data[4];
    data[0] = ccs811_hum << 8;
    data[1] = ccs811_hum & 0x00FF;
    data[2] = ccs811_temp << 8;
    data[3] = ccs811_temp & 0x00FF;

    status = ccs811_write_reg(0x05, data, 4);

    if (status != CCS811_SUCCESS) {
        printf("write environment value is failed.\n");
    }

    return status;
}

int8_t ccs811_write_eco2_threshold(ccs811_eco2_threshold_t *eco2_threshold)
{
    if (eco2_threshold == NULL) {
        printf("eco2_threshold is NULL.\n");
        return CCS811_ERROR;
    }

    uint8_t status = CCS811_SUCCESS;
    uint8_t data[5] = {0};

    data[0] = (uint8_t)(eco2_threshold->lower_thresh >> 8);
    data[1] = (uint8_t)(eco2_threshold->lower_thresh & 0x00FF);
    data[2] = (uint8_t)(eco2_threshold->upper_thresh >> 8);
    data[3] = (uint8_t)(eco2_threshold->upper_thresh & 0x00FF);
    data[4] = eco2_threshold->hysteresis;

    status = ccs811_write_reg(0x10, data, 5);

    if (status != CCS811_SUCCESS) {
        printf("write eco2 threshold is failed\n");
    }

    return status;
}

int8_t ccs811_get_hardware_id(uint8_t *hardware_id)
{
    if (hardware_id == NULL) {
        printf("hardware_id is NULL.\n");
        return CCS811_ERROR;
    }

    uint8_t status = CCS811_SUCCESS;
    status = ccs811_read_reg(0x20, hardware_id, 1);

    if (status != CCS811_SUCCESS) {
        printf("Get hardware ID is failed.\n");
    }

    return status;
}

int8_t ccs811_get_hardware_version(uint8_t *hardware_version)
{
    if (hardware_version == NULL) {
        printf("hardware_version is NULLl\n");
    }

    uint8_t status = CCS811_SUCCESS;
    status = ccs811_read_reg(0x21, hardware_version, 1);

    if (status != CCS811_SUCCESS) {
        printf("Get hardware version is failed.\n");
    }

    return status;
}

/* CCS811レジスタへの書き込み */
int8_t ccs811_write_reg(uint8_t reg_address, uint8_t *data, uint8_t size)
{
    if (data == NULL) {
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    status = i2c_master_start(cmd_handle);

    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle,
                                       (CCS811_DEV_ADDR << 1) | I2C_MASTER_WRITE,
                                       ENABLE_ACK_CHECK);
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
        status = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 10 / portTICK_PERIOD_MS);
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

/* CCS811レジスタの読み取り */
int8_t ccs811_read_reg(uint8_t reg_address, uint8_t *data, uint8_t size)
{
    if (data == NULL) {
        return CCS811_ERROR;
    }

    int8_t status = CCS811_SUCCESS;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    status = i2c_master_start(cmd_handle);
    if (status == CCS811_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle,
                                       (CCS811_DEV_ADDR << 1) | I2C_MASTER_WRITE,
                                       ENABLE_ACK_CHECK);
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
        status = i2c_master_write_byte(cmd_handle,
                                       (CCS811_DEV_ADDR << 1) | I2C_MASTER_READ,
                                       ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        if (size > 1) {
            status = i2c_master_read(cmd_handle, data, size-1, I2C_MASTER_ACK);
        }
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_read_byte(cmd_handle, data+size-1, I2C_MASTER_NACK);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_stop(cmd_handle);
    } else {
        goto ERROR;
    }

    if (status == CCS811_SUCCESS) {
        status = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 10 / portTICK_PERIOD_MS);
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

