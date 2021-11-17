#include "GT911.h"
#include <Wire.h>

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

// Interrupt handling
volatile bool gt911IRQ = false;

#if defined(ESP8266)
void ICACHE_RAM_ATTR _gt911_irq_handler() {
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#elif defined(ESP32)
void IRAM_ATTR _gt911_irq_handler() {
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#else
void _gt911_irq_handler() {
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#endif

GT911::GT911(TwoWire *twi) : _wire(twi ? twi : &Wire) {

}

void GT911::reset() {
  delay(1);

  pinMode(_intPin, OUTPUT);
  pinMode(_rstPin, OUTPUT);

  digitalWrite(_intPin, LOW);
  digitalWrite(_rstPin, LOW);

  delay(11);

  digitalWrite(_intPin, _addr == GT911_I2C_ADDR_28);

  delayMicroseconds(110);
  pinMode(_rstPin, INPUT);

  delay(6);
  digitalWrite(_intPin, LOW);

  delay(51);
}

void GT911::i2cStart(uint16_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg >> 8);
  _wire->write(reg & 0xFF);
}

bool GT911::write(uint16_t reg, uint8_t data) {
  i2cStart(reg);
  _wire->write(data);
  return _wire->endTransmission() == I2C_ERROR_OK;
}

uint8_t GT911::read(uint16_t reg) {
  i2cStart(reg);
  _wire->endTransmission();
  _wire->requestFrom(_addr, (uint8_t)1);
  while (_wire->available()) {
    return _wire->read();
  }
  return 0;
}

bool GT911::writeBytes(uint16_t reg, uint8_t *data, uint16_t size) {
  i2cStart(reg);
  for (uint16_t i = 0; i < size; i++) {
      _wire->write(data[i]);
  }
  return _wire->endTransmission() == I2C_ERROR_OK;
}

bool GT911::readBytes(uint16_t reg, uint8_t *data, uint16_t size) {
  i2cStart(reg);
  _wire->endTransmission();

  uint16_t index = 0;
  while (index < size) {
    uint8_t req = _min(size - index, I2C_BUFFER_LENGTH);
    _wire->requestFrom(_addr, req);
    while (_wire->available()) {
      data[index++] = _wire->read();
    }
    index++;
  }

  return size == index;
}

uint8_t GT911::calcChecksum(uint8_t *buf, uint8_t len) {
  uint8_t ccsum = 0;
  for (uint8_t i = 0; i < len; i++) {
    ccsum += buf[i];
  }

  return (~ccsum) + 1;
}

uint8_t GT911::readChecksum() {
  return read(GT911_REG_CHECKSUM);
}

int8_t GT911::readTouches() {
  uint8_t flag = read(GT911_REG_COORD_ADDR);
  if ((flag & 0x80) && ((flag & 0x0F) < 6)) {
    write(GT911_REG_COORD_ADDR, 0);
  }
  return flag & 0x0F;
}

bool GT911::readTouchPoints() {
  return readBytes(GT911_REG_COORD_ADDR + 1, (uint8_t*)_points, sizeof(GTPoint) * GT911_MAX_CONTACTS);
}

bool GT911::begin(int8_t intPin, int8_t rstPin, uint8_t addr, uint32_t clk) {
  _intPin = intPin;
  _rstPin = rstPin;
  _addr = addr;

  if (_rstPin > 0) {
    delay(300);
    reset();
    delay(200);
  }

  if (intPin > 0) {
    pinMode(_intPin, INPUT);
    attachInterrupt(_intPin, _gt911_irq_handler, FALLING);
  }

  _wire->setClock(clk);
  _wire->begin();
  _wire->beginTransmission(_addr);
  return _wire->endTransmission() == I2C_ERROR_OK;
}

bool GT911::productID(uint8_t *buf, uint8_t len) {
  if (len < 4) {
    return false;
  }

  memset(buf, 0, 4);
  return readBytes(GT911_REG_ID, buf, 4);
}

GTConfig* GT911::readConfig() {
  readBytes(GT911_REG_CFG, (uint8_t*)&_config, sizeof(_config));

  if (readChecksum() == calcChecksum((uint8_t*)&_config, sizeof(_config))) {
    _configLoaded = true;
    return &_config;
  }
  return nullptr;
}

bool GT911::writeConfig() {
  uint8_t checksum = calcChecksum((uint8_t*)&_config, sizeof(_config));
  if (_configLoaded && readChecksum() != checksum) { // Config is different
    writeBytes(GT911_REG_CFG, (uint8_t*)&_config, sizeof(_config));

    uint8_t buf[2] = { checksum, 1 };
    writeBytes(GT911_REG_CHECKSUM, buf, sizeof(buf));
    return true;
  }
  return false;
}

GTInfo* GT911::readInfo() {
  readBytes(GT911_REG_DATA, (uint8_t*)&_info, sizeof(_info));
  return &_info;
}

uint8_t GT911::touched(uint8_t mode) {
  bool irq = false;
  if (mode == GT911_MODE_INTERRUPT) {
    noInterrupts();
    irq = gt911IRQ;
    gt911IRQ = false;
    interrupts();
  } else if (mode == GT911_MODE_POLLING) {
    irq = true;
  }

  uint8_t contacts = 0;
  if (irq) {
    contacts = readTouches();

    if (contacts > 0) {
      readTouchPoints();
    }
  }

  return contacts;
}

GTPoint GT911::getPoint(uint8_t num) {
  return _points[num];
}

GTPoint *GT911::getPoints() {
  return _points;
}
