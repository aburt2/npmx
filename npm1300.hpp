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

#define NPM1300_LED1 0
#define NPM1300_LED2 1
#define NPM1300_LED3 2

// Battery thermistor setting: auto detects whether one is fitted (see begin())
#define NPM1300_NTC_AUTO (-1)

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

    // Charger settings, applied by begin() (change them before calling it)
    uint32_t charge_current_ua = 32000;                              // 32 mA to 800 mA, 2 mA steps
    npmx_charger_voltage_t charge_end_voltage = NPMX_CHARGER_VOLTAGE_4V20; // 1S Li-ion / Li-po
    // Battery thermistor: NPM1300_NTC_AUTO, or an npmx_adc_ntc_type_t (NPMX_ADC_NTC_TYPE_HI_Z for none)
    int ntc_type = NPM1300_NTC_AUTO;
    // How long the SHPHLD button must be held to wake from ship / hibernate mode
    npmx_ship_time_t wake_button_time = NPMX_SHIP_TIME_96_MS;


    // Set input limits
    void vbus_current_limit_set(npmx_vbusin_current_t current_limit);
    void battery_current_limit_set(uint16_t current);
    npmx_vbusin_current_t vbus_current_limit_get(bool print = false);
    void print_vbus_current_limit(npmx_vbusin_current_t current_limit);

    // Configure charger
    int configure_charger();
    void enable_charger();
    void disable_charger();
    void clear_charger_errors();
    void set_charge_current(uint32_t current); // in microamps
    void set_charge_endvoltage(npmx_charger_voltage_t voltage);
    bool vbus_present();
    void print_charger_status(Print &out = Serial);

    // Get Fuel gauge stats
    float getvoltage();
    float getcurrent();
    float getsoc();

    // Power management
    void sleep(); // Ship mode: wakes on the SHPHLD button or USB power
    // Hibernate mode: wakes on the SHPHLD button, USB power, or after wake_after_ms (0: the timer's maximum, ~74 h).
    // Returns non-zero if the PMIC could not be told to hibernate.
    int hibernate(uint32_t wake_after_ms = 0);

    // LED Control
    void configureLEDs();
    void configureLEDs(npm1300_led_config conf);
    void configureLED(int led, npmx_led_mode_t led_mode);
    bool led_on(int led);
    bool led_off(int led);

    // GPIO configuration
    void enable_gpio_interrupt(uint8_t npmx_pin);

  private:
    npmx_adc_ntc_type_t detect_ntc();
    int write_reg(uint16_t address, uint8_t value);
    int read_reg(uint16_t address, uint8_t *value);
    int configure_wake_button();
};

#endif