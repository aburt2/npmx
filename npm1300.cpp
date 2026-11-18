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
    if (npmx_err != NPMX_SUCCESS) return -1;

    // Configure LEDs
    configureLEDs();

    // Enable battery current readings
    npmx_err = npmx_adc_ibat_meas_enable_set(npmx_adc_get(&npm1300_instance, 0), true);
    if (npmx_err != NPMX_SUCCESS) return -1;

    // Increase VBUS input limit (a few tries, rather than hanging if the PMIC doesn't answer)
    vbus_current_limit_set(vbus_limit);
    for (int tries = 0; vbus_current_limit_get(true) != vbus_limit; tries++) {
        if (tries >= 5) return -1;
        delay(500);
        vbus_current_limit_set(vbus_limit);
    }
    // poll() watches this event to catch USB being plugged back in
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSVBUSIN0CLR), MAIN_EVENTSVBUSIN0SET_EVENTVBUSDETECTED_Msk);

    // Why the PMIC last reset (e.g. a wake from hibernate), then clear it for next time
    if (read_reg(NPMX_REG_TO_ADDR(NPM_ERRLOG->RSTCAUSE), &last_reset_cause) == 0) {
        Serial.print("PMIC last reset: ");
        print_reset_cause(last_reset_cause);
        write_reg(NPMX_REG_TO_ADDR(NPM_ERRLOG->TASKCLRERRLOG), 1);
    }

    // Wake from ship / hibernate with the SHPHLD button
    if (configure_wake_button() != 0) return -1;

    // Configure and enable the charger
    if (configure_charger() != 0) return -1;

    // Return 0 for no error
    return 0;
}

int NPM1300_PMIC::write_reg(uint16_t address, uint8_t value) {
    return (npmx_backend_register_write(&npm1300_backend, address, &value, 1) == NPMX_SUCCESS) ? 0 : -1;
}

int NPM1300_PMIC::read_reg(uint16_t address, uint8_t *value) {
    return (npmx_backend_register_read(&npm1300_backend, address, value, 1) == NPMX_SUCCESS) ? 0 : -1;
}

/* Detect a battery thermistor. The charger only charges between the cold and hot NTC thresholds, so with no
 * thermistor (NTC pin grounded or open) and NTC monitoring left on (the default), it never starts charging. */
npmx_adc_ntc_type_t NPM1300_PMIC::detect_ntc() {
    // Measure with the default 10 kOhm bias. Written directly: npmx_adc_ntc_config_set() also stores the beta in
    // the SCRATCH registers, whose bit 0 enables the boot monitor.
    write_reg(NPMX_REG_TO_ADDR(NPM_ADC->ADCNTCRSEL), NPMX_ADC_NTC_TYPE_10_K);
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSADCCLR), MAIN_EVENTSADCSET_EVENTADCNTCRDY_Msk);
    npmx_adc_task_trigger(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_TASK_SINGLE_SHOT_NTC);

    bool ready = false;
    for (int i = 0; (i < 20) && !ready; i++) {
        delay(1);
        npmx_adc_meas_check(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_MEAS_BAT_TEMP, &ready);
    }
    uint8_t msb = 0, lsbs = 0;
    if (!ready || read_reg(NPMX_REG_TO_ADDR(NPM_ADC->ADCNTCRESULTMSB), &msb) ||
        read_reg(NPMX_REG_TO_ADDR(NPM_ADC->ADCGP0RESULTLSBS), &lsbs)) {
        Serial.println("PMIC: battery thermistor measurement failed, charging without one");
        return NPMX_ADC_NTC_TYPE_HI_Z;
    }
    // 10-bit code = 1024 * R_ntc / (R_ntc + 10k): about 512 at 25 C, 237 at 60 C, 891 at -20 C
    uint16_t code = ((uint16_t)msb << 2) |
                    ((lsbs & ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_Msk) >> ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_Pos);
    bool fitted = (code > 40) && (code < 1000); // Not shorted to ground and not open
    Serial.printf("PMIC: NTC reading %u, battery thermistor %s\n", code, fitted ? "found (10k)" : "not fitted");
    return fitted ? NPMX_ADC_NTC_TYPE_10_K : NPMX_ADC_NTC_TYPE_HI_Z;
}

int NPM1300_PMIC::configure_charger() {
    npmx_charger_t *charger = npmx_charger_get(&npm1300_instance, 0);

    // The charge current can only be changed with the charger off
    disable_charger();

    // Battery thermistor
    npmx_adc_ntc_type_t ntc = (ntc_type == NPM1300_NTC_AUTO) ? detect_ntc() : (npmx_adc_ntc_type_t)ntc_type;
    if (write_reg(NPMX_REG_TO_ADDR(NPM_ADC->ADCNTCRSEL), ntc) != 0) return -1;
    if (ntc == NPMX_ADC_NTC_TYPE_HI_Z) {
        // No thermistor: ignore the battery temperature limits (die temperature regulation still applies)
        npmx_charger_module_disable_set(charger, NPMX_CHARGER_MODULE_NTC_LIMITS_MASK);
    } else {
        npmx_charger_module_enable_set(charger, NPMX_CHARGER_MODULE_NTC_LIMITS_MASK);
    }
    // Start the automatic thermistor and die temperature measurements used while charging
    npmx_adc_task_trigger(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_TASK_UPDATE_AUTO_INTERVAL);

    // Termination voltage, also for the warm region (it defaults to 3.6 V)
    set_charge_endvoltage(charge_end_voltage);
    set_charge_current(charge_current_ua);

    // Leave any error state from before (e.g. a safety timer that expired), then charge, recharging when it drops
    clear_charger_errors();
    npmx_charger_module_enable_set(charger, NPMX_CHARGER_MODULE_RECHARGE_MASK);
    enable_charger();

    // Check it took
    uint32_t modules = 0;
    if ((npmx_charger_module_get(charger, &modules) != NPMX_SUCCESS) ||
        !(modules & NPMX_CHARGER_MODULE_CHARGER_MASK)) {
        Serial.println("PMIC: charger did not enable");
        return -1;
    }
    return 0;
}

void NPM1300_PMIC::clear_charger_errors() {
    npmx_charger_t *charger = npmx_charger_get(&npm1300_instance, 0);
    npmx_charger_task_trigger(charger, NPMX_CHARGER_TASK_CLEAR_ERROR);
    npmx_charger_task_trigger(charger, NPMX_CHARGER_TASK_RELEASE);
    npmx_charger_task_trigger(charger, NPMX_CHARGER_TASK_CLEAR_TIMERS);
    // The charger's events must be cleared for charging to proceed
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSBCHARGER0CLR), 0xFF);
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSBCHARGER1CLR), 0xFF);
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSBCHARGER2CLR), 0xFF);
}

bool NPM1300_PMIC::vbus_present() {
    uint8_t status = 0;
    npmx_vbusin_vbus_status_get(npmx_vbusin_get(&npm1300_instance, 0), &status);
    return (status & NPMX_VBUSIN_STATUS_CONNECTED_MASK) != 0;
}

void NPM1300_PMIC::print_charger_status(Print &out) {
    npmx_charger_t *charger = npmx_charger_get(&npm1300_instance, 0);
    uint32_t modules = 0;
    uint8_t status = 0, ntc = 0, error = 0, sensor = 0, ntc_sel = 0;
    npmx_charger_module_get(charger, &modules);
    npmx_charger_status_get(charger, &status);
    npmx_charger_ntc_status_get(charger, &ntc);
    read_reg(NPMX_REG_TO_ADDR(NPM_BCHARGER->BCHGERRREASON), &error);
    read_reg(NPMX_REG_TO_ADDR(NPM_BCHARGER->BCHGERRSENSOR), &sensor);
    read_reg(NPMX_REG_TO_ADDR(NPM_ADC->ADCNTCRSEL), &ntc_sel);

    out.printf("USB power: %s\n", vbus_present() ? "yes" : "no");
    out.print("Last PMIC reset: ");
    print_reset_cause(last_reset_cause, out);
    out.printf("Charger: %s, NTC limits %s, thermistor %s, recharge %s\n",
               (modules & NPMX_CHARGER_MODULE_CHARGER_MASK) ? "enabled" : "DISABLED",
               (modules & NPMX_CHARGER_MODULE_NTC_LIMITS_MASK) ? "on" : "off",
               (ntc_sel == NPMX_ADC_NTC_TYPE_HI_Z) ? "none" : (ntc_sel == NPMX_ADC_NTC_TYPE_10_K) ? "10k"
               : (ntc_sel == NPMX_ADC_NTC_TYPE_47_K) ? "47k" : "100k",
               (modules & NPMX_CHARGER_MODULE_RECHARGE_MASK) ? "on" : "off");
    out.printf("Charging: %s%s%s%s%s%s%s(status 0x%02X)\n",
               (status & NPMX_CHARGER_STATUS_TRICKLE_CHARGE_MASK) ? "trickle " : "",
               (status & NPMX_CHARGER_STATUS_CONSTANT_CURRENT_MASK) ? "constant-current " : "",
               (status & NPMX_CHARGER_STATUS_CONSTANT_VOLTAGE_MASK) ? "constant-voltage " : "",
               (status & NPMX_CHARGER_STATUS_COMPLETED_MASK) ? "complete " : "",
               (status & NPMX_CHARGER_STATUS_RECHARGE_MASK) ? "recharge-needed " : "",
               (status & NPMX_CHARGER_STATUS_DIE_TEMP_HIGH_MASK) ? "paused-die-hot " : "",
               (status & NPMX_CHARGER_STATUS_SUPPLEMENT_ACTIVE_MASK) ? "supplement " : "", status);
    out.printf("Battery temperature region (NTCSTATUS): 0x%02X (1 cold, 2 cool, 4 warm, 8 hot, 0 nominal)\n", ntc);
    out.printf("Errors: reason 0x%02X, sensor 0x%02X%s\n", error, sensor,
               error ? " (1 NTC sensor, 2 VBAT sensor, 4 VBAT low, 8 Vtrickle, 16 meas timeout, 32 charge timeout,"
                       " 64 trickle timeout)" : "");
    // Fresh VBAT reading, and IBAT after it (IBAT is in microamps)
    npmx_adc_task_trigger(npmx_adc_get(&npm1300_instance, 0), NPMX_ADC_TASK_SINGLE_SHOT_VBAT);
    delay(5);
    out.printf("Battery: %.3f V, %.1f mA\n", getvoltage(), getcurrent() / 1000.0f);
}

bool NPM1300_PMIC::poll() {
    // VBUS detected is latched, so a quick unplug and replug between calls is still caught
    uint8_t events = 0;
    if (read_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSVBUSIN0SET), &events) != 0) return false;
    if (!(events & MAIN_EVENTSVBUSIN0SET_EVENTVBUSDETECTED_Msk)) return false;
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSVBUSIN0CLR), MAIN_EVENTSVBUSIN0SET_EVENTVBUSDETECTED_Msk);

    // VBUSINILIM0 keeps its value (so reading it back says nothing), but the PMIC switched back to the startup
    // limit when USB was removed: select VBUSINILIM0 again
    npmx_vbusin_t *vbusin = npmx_vbusin_get(&npm1300_instance, 0);
    npmx_vbusin_current_limit_set(vbusin, vbus_limit);
    npmx_vbusin_task_trigger(vbusin, NPMX_VBUSIN_TASK_APPLY_CURRENT_LIMIT);
    Serial.print("PMIC: USB connected, ");
    print_vbus_current_limit(vbus_limit);
    return true;
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
void NPM1300_PMIC::disable_charger() {
    npmx_charger_module_disable_set(npmx_charger_get(&npm1300_instance, 0), NPMX_CHARGER_MODULE_CHARGER_MASK);
}
void NPM1300_PMIC::set_charge_current(uint32_t current) {
    // Only takes effect with the charger disabled, so turn it off around the change if it was on
    npmx_charger_t *charger = npmx_charger_get(&npm1300_instance, 0);
    uint32_t modules = 0;
    npmx_charger_module_get(charger, &modules);
    bool was_enabled = modules & NPMX_CHARGER_MODULE_CHARGER_MASK;
    if (was_enabled) disable_charger();
    if (npmx_charger_charging_current_set(charger, current) != NPMX_SUCCESS) {
        Serial.printf("PMIC: invalid charge current %lu uA (32000 to 800000, 2000 steps)\n", (unsigned long)current);
    }
    if (was_enabled) enable_charger();
}
void NPM1300_PMIC::set_charge_endvoltage(npmx_charger_voltage_t voltage) {
    npmx_charger_termination_normal_voltage_set(npmx_charger_get(&npm1300_instance, 0), voltage);
    npmx_charger_termination_warm_voltage_set(npmx_charger_get(&npm1300_instance, 0), voltage);
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
int NPM1300_PMIC::configure_wake_button() {
    // SHPHLD button (active low) held for wake_button_time exits ship / hibernate mode. Must be set (and strobed,
    // which npmx_ship_config_set does) before entering either mode.
    npmx_ship_config_t config = {.time = wake_button_time, .inverted_polarity = false};
    return (npmx_ship_config_set(npmx_ship_get(&npm1300_instance, 0), &config) == NPMX_SUCCESS) ? 0 : -1;
}

int NPM1300_PMIC::sleep() {
    // Go to ship mode (needs USB power removed too)
    if (vbus_present()) return NPM1300_HIBERNATE_VBUS;
    if (configure_wake_button() != 0) return -1;
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSSHPHLDCLR), 0xFF);
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSVBUSIN0CLR), 0xFF);
    led_off(NPM1300_LED2);
    led_off(NPM1300_LED3);
    if (npmx_ship_task_trigger(npmx_ship_get(&npm1300_instance, 0), NPMX_SHIP_TASK_SHIPMODE) != NPMX_SUCCESS) {
        return -1;
    }
    return 0;
}

void NPM1300_PMIC::print_reset_cause(uint8_t cause, Print &out) {
    static const char *names[] = {"ship/hibernate exit", "boot monitor", "watchdog", "SHPHLD long press",
                                  "thermal shutdown", "VSYS low (power fail)", "software reset"};
    if (cause == 0) {
        out.println("power on");
        return;
    }
    bool first = true;
    for (int i = 0; i < 7; i++) {
        if (cause & (1 << i)) {
            out.print(first ? "" : ", ");
            out.print(names[i]);
            first = false;
        }
    }
    out.println();
}

int NPM1300_PMIC::hibernate(uint32_t wake_after_ms) {
    // With USB power connected the PMIC doesn't hibernate (the datasheet says to wait for VBUS to be removed and
    // discharged before triggering it, and VBUS is itself a wake source)
    if (vbus_present()) return NPM1300_HIBERNATE_VBUS;

    if (configure_wake_button() != 0) return -1;

    // Wake-up timer: 64 Hz clock (15.625 ms per count), 24-bit compare value
    uint32_t counts = (wake_after_ms == 0) ? 0xFFFFFF : (uint32_t)(((uint64_t)wake_after_ms * 64 + 999) / 1000);
    if (counts < 1) counts = 1;
    if (counts > 0xFFFFFF) counts = 0xFFFFFF;
    npmx_timer_config_t timer = {
        .mode = NPMX_TIMER_MODE_WAKEUP,
        .prescaler = NPMX_TIMER_PRESCALER_SLOW,
        .compare_value = counts,
    };
    npmx_timer_t *wake_timer = npmx_timer_get(&npm1300_instance, 0);
    if (npmx_timer_config_set(wake_timer, &timer) != NPMX_SUCCESS) return -1; // Also strobes the target

    // Clear stale SHPHLD and VBUS events
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSSHPHLDCLR), 0xFF);
    write_reg(NPMX_REG_TO_ADDR(NPM_MAIN->EVENTSVBUSIN0CLR), 0xFF);

    // Turn off the host driven LEDs, then hibernate (power to the system is cut, so this doesn't return if it works)
    led_off(NPM1300_LED2);
    led_off(NPM1300_LED3);
    if (npmx_ship_task_trigger(npmx_ship_get(&npm1300_instance, 0), NPMX_SHIP_TASK_HIBERNATE) != NPMX_SUCCESS) {
        return -1;
    }
    return 0;
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
    npmx_error_t err = npmx_led_state_set(npmx_led_get(&npm1300_instance, led), true);

    // Return true if err is succes
    if (err == NPMX_SUCCESS) return true;
    else return false;
}
bool NPM1300_PMIC::led_off(int led) {
    // check that led is valid (should be 0, 1 or 2)
    if (led > 2) {
        return false;
    }

    // Set led on
    npmx_error_t err = npmx_led_state_set(npmx_led_get(&npm1300_instance, led), false);

    // Return true if err is succes
    if (err == NPMX_SUCCESS) return true;
    else return false;
}


npmx_error_t i2c_read(void * p_context, uint32_t register_address, uint8_t * p_data, size_t num_of_bytes) {
    NPM1300_PMIC* obj = (NPM1300_PMIC*)p_context;
    size_t offset = 0;

    // Registers auto-increment, so read in buffer sized chunks from consecutive addresses
    while (offset < num_of_bytes) {
        uint16_t address = register_address + offset;
        uint8_t reg_addr[2] = {(uint8_t)(address >> 8), (uint8_t)address};
        size_t length = num_of_bytes - offset;
        if (length > I2C_BUFFER_LENGTH) length = I2C_BUFFER_LENGTH;

        obj->i2c->beginTransmission(obj->i2c_address);
        obj->i2c->write(reg_addr, 2);
        if (obj->i2c->endTransmission(false) != 0) {
            return NPMX_ERROR_IO;
        }
        if (obj->i2c->requestFrom(obj->i2c_address, length, true) != length) {
            return NPMX_ERROR_IO;
        }
        for (size_t i = 0; i < length; i++) {
            p_data[offset + i] = obj->i2c->read();
        }
        offset += length;
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
    obj->i2c->write(p_data, num_of_bytes);
    if (obj->i2c->endTransmission() != 0) {
        return NPMX_ERROR_IO;
    }

    return NPMX_SUCCESS;
}
