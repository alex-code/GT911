#ifndef GT911_H
#define GT911_H

#include <Arduino.h>
#include <Wire.h>
#include "GT911_Structs.h"

// 0x28/0x29 (0x14 7bit)
#define GT911_I2C_ADDR_28  0x14
// 0xBA/0xBB (0x5D 7bit)
#define GT911_I2C_ADDR_BA  0x5D

#define GT911_MAX_CONTACTS 5

#define GT911_REG_CFG  0x8047
#define GT911_REG_CHECKSUM 0x80FF
#define GT911_REG_DATA 0x8140
#define GT911_REG_ID 0x8140
#define GT911_REG_COORD_ADDR 0x814E

enum : uint8_t {
  GT911_MODE_INTERRUPT,
  GT911_MODE_POLLING
};

class GT911 {
  private:
    TwoWire *_wire;
    uint8_t _intPin;
    uint8_t _rstPin;
    uint8_t _addr;

    bool _configLoaded = false;
    GTConfig _config;
    GTInfo _info;
    GTPoint _points[GT911_MAX_CONTACTS];

    void reset();
    void i2cStart(uint16_t reg);
    bool write(uint16_t reg, uint8_t data);
    uint8_t read(uint16_t reg);
    bool writeBytes(uint16_t reg, uint8_t *data, uint16_t size);
    bool readBytes(uint16_t reg, uint8_t *data, uint16_t size);
    uint8_t calcChecksum(uint8_t *buf, uint8_t len);
    uint8_t readChecksum();
    int8_t readTouches();
    bool readTouchPoints();
  public:
    GT911(TwoWire *twi = &Wire);
    bool begin(int8_t intPin = -1, int8_t rstPin = -1, uint8_t addr = GT911_I2C_ADDR_BA,
               uint32_t clk = 400000);
    bool productID(uint8_t *buf, uint8_t len);
    GTConfig* readConfig();
    bool writeConfig();
    GTInfo* readInfo();

    uint8_t touched(uint8_t mode = GT911_MODE_INTERRUPT);
    GTPoint getPoint(uint8_t num);
    GTPoint *getPoints();
};

#endif
