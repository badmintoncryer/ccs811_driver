#include "ccs811.h"

#define ENABLE_ACK_CHECK (true)
#define DISABLE_ACK_CHECK (false)

static ccs811_config_t *config_table;

/* CCS811の初期化を行う。 */
int8_t ccs811_init(ccs811_config_t *ccs811_config)
{
    if (ccs811_config == NULL) {
        return CCS811_ERROR;
    }
    config_table = ccs811_config;

    int8_t status = CCS811_SUCCESS;
    uint8_t data = 0;

    status = ccs811_read_reg(0x20, &data, 1);
    printf("WHO AM I is %x\n", data);

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

    if (status == CCS811_SUCCESS) {
        status = i2c_master_write(cmd_handle, data, size, ENABLE_ACK_CHECK);
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

