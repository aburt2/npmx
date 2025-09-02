#include "npm1300.hpp"
static npmx_error_t i2c_write(void * p_context, uint32_t register_address,
    uint8_t * p_data, size_t num_of_bytes);
static npmx_error_t i2c_read(void * p_context, uint32_t register_address,
    uint8_t * p_data, size_t num_of_bytes);


NPM1300_PMIC::NPM1300_PMIC(TwoWire &i2c_ref) {
  i2c = &i2c_ref; 
}


int NPM1300_PMIC::begin() {
    npmx_instance_t npm1300_instance;
    npmx_backend_t npm1300_backend;
    npm1300_backend.p_read = i2c_read;
    npm1300_backend.p_write = i2c_write;
    npm1300_backend.p_context = NULL; // Optional context for our use

    npmx_error_t npmx_err = npmx_core_init(&npm1300_instance, &npm1300_backend, NULL, true);
}


npmx_error_t i2c_read(void * p_context, uint32_t register_address, uint8_t * p_data, size_t num_of_bytes) {
    NPM1300_PMIC* obj = (NPM1300_PMIC*)p_context;
    uint8_t err = 0;
    uint16_t offset = 0;
    uint8_t reg_addr[2];

    reg_addr[0] = register_address >> 8;
    reg_addr[1] = register_address;


    obj->i2c->beginTransmission(obj->i2c_address);
    obj->i2c->write(reg_addr, 2);
    err = obj->i2c->endTransmission(false);
    if (err) {
        return NPMX_ERROR_IO;
    }

    while(offset < num_of_bytes)
    {
        uint16_t rx_bytes = 0;
        if(offset != 0)
        obj->i2c->beginTransmission(obj->i2c_address);
        uint16_t length = ((rlen - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (rlen - offset) ;
        rx_bytes = obj->i2c->requestFrom(obj->i2c_address, length);
        if (rx_bytes == length) {
        for(uint8_t i = 0; i < length; i++) {
            rbuffer[offset+i] = obj->i2c->read();
        }
        offset += length;
        obj->i2c->endTransmission((offset == rlen));
        } else {
        obj->i2c->endTransmission((offset == rlen));
        }
    }

    return NPMX_SUCCESS;
}


npmx_error_t i2c_write(void * p_context, uint32_t register_address, uint8_t * p_data, size_t num_of_bytes) {
    uint8_t data[TWIM_BUF_SIZE];

    nrfx_twim_xfer_desc_t desc = {
        .type = NRFX_TWIM_XFER_TX,
        .address = NPM1300_ADDR,
        .primary_length = num_of_bytes + 2,
        .secondary_length = 0,
        .p_primary_buf = data,
        .p_secondary_buf = NULL
    };

    if (num_of_bytes > (TWIM_BUF_SIZE - 2)) {
        return NPMX_ERROR_INVALID_PARAM;
    }

    data[0] = register_address >> 8;
    data[1] = register_address;
    memcpy(&data[2], p_data, num_of_bytes);

    nrfx_err_t err = nrfx_twim_xfer(&twim, &desc, 0);
    if (err) {
        return NPMX_ERROR_IO;
    }

    while (!nrf_atomic_flag_clear_fetch(&twim_op_done)) {
        __WFE();
    }

    return NPMX_SUCCESS;
}