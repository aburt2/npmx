#ifndef NPM1300_H
#define NPM1300_H

#include "Arduino.h"
#include "Wire.h"

extern "C" {
#include <npmx.h>
#include <npmx_core.h>
#include <npmx_instance.h>
}

class NPM1300_PMIC {
  public:
    NPM1300_PMIC(TwoWire &i2c);
    int begin();

    // I2C address
    uint8_t i2c_address = 0x6b;
    TwoWire *i2c;

    

  protected:
};

#endif