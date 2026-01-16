#include "npm1300.hpp"
static npmx_error_t i2c_write(void * p_context, uint32_t register_address,
    uint8_t * p_data, size_t num_of_bytes);
static npmx_error_t i2c_read(void * p_context, uint32_t register_address,
    uint8_t * p_data, size_t num_of_bytes);

npmx_instance_t npm1300_instance;
npmx_backend_t npm1300_backend;

NPM1300_PMIC::NPM1300_PMIC(TwoWire &i2c_ref) {
    i2c = &i2c_ref; 
}

int NPM1300_PMIC::begin(TwoWire &i2c_ref) {
    i2c = &i2c_ref; 

    return begin();
}

int NPM1300_PMIC::begin() {
    npmx_error_t npmx_err;

    // Setup backend
    npm1300_backend.p_read = i2c_read;
    npm1300_backend.p_write = i2c_write;
    npm1300_backend.p_context = (void *)this; // Optional context for our use

    // Initialise core
    npmx_err = npmx_core_init(&npm1300_instance, &npm1300_backend, NULL, true);

    if (npmx_err == NPMX_SUCCESS) {
        return 0;
    } else {
        return -1;
    }

    // Configure LEDs
    configureLEDs();

    // Enable battery current readings
    npmx_err = npmx_adc_ibat_meas_enable_set(npmx_adc_get(&npm1300_instance, 0), true);

    if (npmx_err == NPMX_SUCCESS) {
        return 0;
    } else {
        return -1;
    }

    // Increase VBUS input limit
    vbus_current_limit_set(NPMX_VBUSIN_CURRENT_1500_MA);

    // Enable charger
    set_charge_endvoltage(NPMX_CHARGER_VOLTAGE_4V20); // set max voltage for 1S lithium ion battery
    enable_charger();
}

void NPM1300_PMIC::vbus_current_limit_set(npmx_vbusin_current_t current_limit) {
    // Configure VBUS current limit
    npmx_vbusin_current_limit_set(npmx_vbusin_get(&npm1300_instance, 0), current_limit);

    // Actually set it
    delay(100);
    npmx_vbusin_task_trigger(npmx_vbusin_get(&npm1300_instance, 0), NPMX_VBUSIN_TASK_APPLY_CURRENT_LIMIT);

    // Wait a bit for it to set
    delay(100);
}

npmx_vbusin_current_t NPM1300_PMIC::vbus_current_limit_get(bool print) {
    npmx_vbusin_current_t current_limit;
    npmx_vbusin_current_limit_get(npmx_vbusin_get(&npm1300_instance, 0), &current_limit);
    
    if (print) {
        print_vbus_current_limit(current_limit);
    }
    return current_limit;
}

void NPM1300_PMIC::print_vbus_current_limit(npmx_vbusin_current_t current_limit) {
    switch (current_limit)
    {
    case NPMX_VBUSIN_CURRENT_100_MA:
        /* code */
        Serial.println("Vbus limit: 100mA");
        break;
    case NPMX_VBUSIN_CURRENT_500_MA:
        /* code */
        Serial.println("Vbus limit: 500mA");
        break;
    case NPMX_VBUSIN_CURRENT_600_MA:
        /* code */
        Serial.println("Vbus limit: 600mA");
        break;
    case NPMX_VBUSIN_CURRENT_700_MA:
        /* code */
        Serial.println("Vbus limit: 700mA");
        break;
    case NPMX_VBUSIN_CURRENT_800_MA:
        /* code */
        Serial.println("Vbus limit: 800mA");
        break;
    case NPMX_VBUSIN_CURRENT_900_MA:
        /* code */
        Serial.println("Vbus limit: 900mA");
        break;
    case NPMX_VBUSIN_CURRENT_1000_MA:
        /* code */
        Serial.println("Vbus limit: 1000mA");
        break;
    case NPMX_VBUSIN_CURRENT_1100_MA:
        /* code */
        Serial.println("Vbus limit: 1100mA");
        break;
    case NPMX_VBUSIN_CURRENT_1200_MA:
        /* code */
        Serial.println("Vbus limit: 1200mA");
        break;
    case NPMX_VBUSIN_CURRENT_1300_MA:
        /* code */
        Serial.println("Vbus limit: 1300mA");
        break;
    case NPMX_VBUSIN_CURRENT_1400_MA:
        /* code */
        Serial.println("Vbus limit: 1400mA");
        break;
    case NPMX_VBUSIN_CURRENT_1500_MA:
        /* code */
        Serial.println("Vbus limit: 1500mA");
        break;
    default:
        break;
    }
}

void NPM1300_PMIC::battery_current_limit_set(uint16_t current) {
    npmx_charger_discharging_current_set(npmx_charger_get(&npm1300_instance, 0), current);
}
void NPM1300_PMIC::enable_charger() {
    npmx_charger_module_enable_set(npmx_charger_get(&npm1300_instance, 0), NPMX_CHARGER_MODULE_CHARGER_MASK);
}
void NPM1300_PMIC::set_charge_current(uint32_t current) {
    npmx_charger_charging_current_set(npmx_charger_get(&npm1300_instance, 0), current);
}
void NPM1300_PMIC::set_charge_endvoltage(npmx_charger_voltage_t voltage) {
    npmx_charger_termination_normal_voltage_set(npmx_charger_get(&npm1300_instance, 0), voltage);
}
float NPM1300_PMIC::getvoltage() {
    int32_t rawData;
    float voltage; // voltage in Volts

    npmx_adc_meas_get(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_MEAS_VBAT, &rawData);

    // if there is no battery attached
    if (rawData == 0) {
        npmx_adc_meas_get(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_MEAS_VSYS, &rawData);
    }

    // Convert voltage to Volts
    voltage = rawData * 0.001f;
    return voltage;
}
float NPM1300_PMIC::getcurrent() {
    int32_t rawData;
    float current; // current in mA

    npmx_adc_meas_get(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_MEAS_VBAT2_IBAT, &rawData);

    // current in mA
    current = rawData;
    return current;
}
float NPM1300_PMIC::getsoc() {
    float soc;
    return soc;
}
void NPM1300_PMIC::sleep() {
    // Go to ship mode
    npmx_ship_task_trigger(npmx_ship_get(&npm1300_instance, 0), NPMX_SHIP_TASK_SHIPMODE);
}

void NPM1300_PMIC::configureLEDs() {
    // Configure LED1 to show charging status 
    configureLED(NPM1300_LED1, NPMX_LED_MODE_CHARGING);

    // Configure LED2 to be host controlled
    configureLED(NPM1300_LED2, NPMX_LED_MODE_HOST);

    // Configure LED3 to be host controlled
    configureLED(NPM1300_LED3, NPMX_LED_MODE_HOST);
}

void NPM1300_PMIC::configureLEDs(npm1300_led_config conf) {
    // Configure LED1
    configureLED(NPM1300_LED1, conf.led1_mode);

    // Configure LED2
    configureLED(NPM1300_LED2, conf.led2_mode);

    // Configure LED3
    configureLED(NPM1300_LED3, conf.led3_mode);
}

void NPM1300_PMIC::configureLED(int led, npmx_led_mode_t led_mode) {
    // Configure LED
    npmx_led_mode_set(npmx_led_get(&npm1300_instance, led), led_mode);
}

bool NPM1300_PMIC::led_on(int led) {
    // check that led is valid (should be 0, 1 or 2)
    if (led > 2) {
        return false;
    }

    // Set led on
    npmx_led_state_set(npmx_led_get(&npm1300_instance, led), true);
}
bool NPM1300_PMIC::led_off(int led) {
    // check that led is valid (should be 0, 1 or 2)
    if (led > 2) {
        return false;
    }

    // Set led on
    npmx_led_state_set(npmx_led_get(&npm1300_instance, led), false);
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