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

    if (npmx_err == NPMX_SUCCESS) {
        return 0;
    } else {
        return -1;
    }
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
        uint16_t length = ((num_of_bytes - offset) > I2C_BUFFER_LENGTH) ? I2C_BUFFER_LENGTH : (num_of_bytes - offset) ;
        rx_bytes = obj->i2c->requestFrom(obj->i2c_address, length);
        if (rx_bytes == length) {
        for(uint8_t i = 0; i < length; i++) {
            p_data[offset+i] = obj->i2c->read();
        }
        offset += length;
        obj->i2c->endTransmission((offset == num_of_bytes));
        } else {
        obj->i2c->endTransmission((offset == num_of_bytes));
        }
    }

    return NPMX_SUCCESS;
}


npmx_error_t i2c_write(void * p_context, uint32_t register_address, uint8_t * p_data, size_t num_of_bytes) {
    NPM1300_PMIC* obj = (NPM1300_PMIC*)p_context;
    uint8_t reg_addr[2];

    reg_addr[0] = register_address >> 8;
    reg_addr[1] = register_address;

    obj->i2c->beginTransmission(obj->i2c_address);
    obj->i2c->write(reg_addr, 2);
    for(uint8_t i = 0; i < num_of_bytes; i++) {
        obj->i2c->write(p_data[i]);
    }
    obj->i2c->endTransmission();

    return NPMX_SUCCESS;
}