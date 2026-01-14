#ifndef NPM1300_PMIC_H
#define NPM1300_PMIC_H

#include "Arduino.h"
#include "Wire.h"

extern "C" {
#include <npmx.h>
#include <npmx_core.h>
#include <npmx_instance.h>
}

#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 256
#endif

struct npm1300_led_config {
  npmx_led_mode_t led1_mode;
  npmx_led_mode_t led2_mode;
  npmx_led_mode_t led3_mode;
};

class NPM1300_PMIC {
  public:
    NPM1300_PMIC(TwoWire &i2c);
    int begin();
    int begin(TwoWire &i2c);

    // I2C address
    uint8_t i2c_address = 0x6b;
    TwoWire *i2c;


    // Set input limits
    void vbus_current_limit_set(npmx_vbusin_current_t current_limit);
    void battery_current_limit_set(uint16_t current);
    npmx_vbusin_current_t vbus_current_limit_get(bool print = false);
    void print_vbus_current_limit(npmx_vbusin_current_t current_limit);

    // Configure charger
    void enable_charger();
    void set_charge_current(uint32_t current);
    void set_charge_endvoltage(npmx_charger_voltage_t voltage);

    // Get Fuel gauge stats
    float getvoltage();
    float getcurrent();
    float getsoc();

    // Power management
    void sleep();

    // LED Control
    void configureLEDs();
    void configureLEDs(npm1300_led_config conf);
    bool led_on(int led);
    bool led_off(int led);

    // GPIO configuration
    void enable_gpio_interrupt(uint8_t npmx_pin);
};

#endif