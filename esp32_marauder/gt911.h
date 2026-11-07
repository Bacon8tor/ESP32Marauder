#pragma once
#ifndef gt911_h
#define gt911_h

#ifdef HAS_GT911_TOUCH

#include <Wire.h>

// Goodix GT911 capacitive touch controller.
// Used by the ESP32-3248S035C ("CYD" 3.5" capacitive) panel.
// I2C pins and panel geometry are provided by configs.h:
//   CTP_SDA, CTP_SCL, CTP_RST, CTP_INT
//
// The GT911 uses 16-bit register addresses and selects its I2C address from
// the INT pin level as RST rises: INT low -> 0x5D, INT high -> 0x14. On the
// 3248S035C the INT line is frequently tied so that 0x5D is used; we probe
// both to be safe.

#ifndef GT911_ADDR_PRIMARY
  #define GT911_ADDR_PRIMARY 0x5D
#endif
#ifndef GT911_ADDR_BACKUP
  #define GT911_ADDR_BACKUP 0x14
#endif

// GT911 registers (16-bit)
#define GT911_REG_PRODUCT_ID 0x8140
#define GT911_REG_STATUS     0x814E
#define GT911_REG_POINT1     0x8150

static uint8_t _gt911_addr = GT911_ADDR_PRIMARY;

static bool _gt911_read(uint16_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(_gt911_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)_gt911_addr, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

static bool _gt911_write(uint16_t reg, uint8_t val) {
  Wire.beginTransmission(_gt911_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool _gt911_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void gt911_init() {
  // Reset pulse. Hold INT low while RST rises to request address 0x5D.
  #if defined(CTP_INT) && (CTP_INT >= 0)
    pinMode(CTP_INT, OUTPUT);
    digitalWrite(CTP_INT, LOW);
  #endif
  #if defined(CTP_RST) && (CTP_RST >= 0)
    pinMode(CTP_RST, OUTPUT);
    digitalWrite(CTP_RST, LOW);
    delay(12);
    #if defined(CTP_INT) && (CTP_INT >= 0)
      digitalWrite(CTP_INT, LOW);   // address select = 0x5D
    #endif
    delayMicroseconds(200);
    digitalWrite(CTP_RST, HIGH);
    delay(8);
  #endif
  #if defined(CTP_INT) && (CTP_INT >= 0)
    delay(50);
    pinMode(CTP_INT, INPUT);        // release INT so the controller can drive it
  #endif

  Wire.begin(CTP_SDA, CTP_SCL, 400000U);
  delay(10);

  // Use whichever address answers on the bus.
  if (_gt911_present(GT911_ADDR_PRIMARY))      _gt911_addr = GT911_ADDR_PRIMARY;
  else if (_gt911_present(GT911_ADDR_BACKUP))  _gt911_addr = GT911_ADDR_BACKUP;
  else                                         _gt911_addr = GT911_ADDR_PRIMARY;

  uint8_t id[4] = {0};
  bool ok = _gt911_read(GT911_REG_PRODUCT_ID, id, 4);
  Serial.printf("[Touch] GT911 @0x%02X id: %c%c%c%c%s\n",
                _gt911_addr, id[0], id[1], id[2], id[3],
                ok ? "" : " (no response)");

  _gt911_write(GT911_REG_STATUS, 0);  // clear any stale status
}

// Returns 1 on a fresh touch (raw panel-native coords in *x/*y),
// 0 for no new touch, -1 on an I2C transport error.
static int8_t gt911_read_raw(uint16_t *x, uint16_t *y) {
  uint8_t status = 0;
  if (!_gt911_read(GT911_REG_STATUS, &status, 1)) return -1;

  if ((status & 0x80) == 0) return 0;                 // buffer not ready
  uint8_t points = status & 0x0F;
  if (points == 0) { _gt911_write(GT911_REG_STATUS, 0); return 0; }

  uint8_t p[8];
  bool ok = _gt911_read(GT911_REG_POINT1, p, 7);
  _gt911_write(GT911_REG_STATUS, 0);                  // ack frame
  if (!ok) return -1;

  // p[0]=track id, p[1..2]=x (little-endian), p[3..4]=y, p[5..6]=size
  *x = (uint16_t)(p[1] | (p[2] << 8));
  *y = (uint16_t)(p[3] | (p[4] << 8));

  #ifdef GT911_DEBUG
    Serial.printf("[Touch] GT911 st=0x%02X pts=%u b=[%02X %02X %02X %02X %02X] x=%u y=%u\n",
                  status, points, p[0], p[1], p[2], p[3], p[4], *x, *y);
  #endif
  return 1;
}

#endif // HAS_GT911_TOUCH
#endif // gt911_h
