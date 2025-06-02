#include <Arduino.h>

#include <Wire.h>

#define SDA_PIN            21
#define SCL_PIN            22
#define PIN_RSTB           4

#define I2C_PRIMARY        0x14
#define I2C_ALT            0x5D

#define REG_CMD_CHECK      0x8046
#define REG_ESD            0x8040
#define REG_CFG_START      0x8047
#define REG_CFG_CHKSUM     0x80FF
#define REG_CFG_REFRESH    0x8100
#define CFG_SIZE           184

// =====184-БАЙТНЫЙ МАССИВ КОНФИГ (version=0x90) =====
uint8_t gt9271_cfg[CFG_SIZE] = {
  0x90,0x00,0x05,0x20,0x03,0x0A,0x3D,0x20, 0x01,0x0A,0x28,0x0F,0x6E,0x5A,0x03,0x05,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18, 0x1A,0x1E,0x14,0x8F,0x2F,0xAA,0x26,0x24,
  0x0C,0x08,0x00,0x00,0x00,0x81,0x03,0x2D, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x1A,0x3C,0x94,0xC5,0x02, 0x07,0x00,0x00,0x04,0x9E,0x1C,0x00,0x89,
  0x21,0x00,0x77,0x27,0x00,0x68,0x2E,0x00, 0x5B,0x37,0x00,0x5B,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x19,0x18,0x17,0x16,0x15,0x14,0x11,0x10, 0x0F,0x0E,0x0D,0x0C,0x09,0x08,0x07,0x06,
  0x05,0x04,0x01,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
  0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F, 0x10,0x11,0x12,0x13,0x14,0x19,0x1B,0x1C,
  0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25, 0x26,0x27,0x28,0x29,0xFF,0xFF,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
// ==================================================================

uint8_t before_cfg[CFG_SIZE], after_cfg[CFG_SIZE];
uint8_t before_chk, after_chk;
uint8_t i2c_addr = 0;



// —————— Вспомогательные ——————

uint8_t scanI2C() {
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return addr;
  }
  return 0;
}

int writeReg(uint8_t addr, uint16_t reg, uint8_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  Wire.write(v);
  int ret = Wire.endTransmission();
  delay(1);
  return ret;
}


uint8_t readReg(uint8_t addr, uint16_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

int writeBlock(uint16_t start, const uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    size_t chunk = min((size_t)30, len - off);
    Wire.beginTransmission(i2c_addr);
    Wire.write(start >> 8);
    Wire.write(start & 0xFF);
    for (size_t i = 0; i < chunk; ++i) Wire.write(buf[off + i]);
    int ret = Wire.endTransmission();
    Serial.printf(" W[%2u]@0x%04X → %d\n", chunk, start, ret);
    if (ret) return ret;
    start += chunk;
    off   += chunk;
    delay(1);
  }
  return 0;
}

void readConfig(uint16_t start, uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    size_t chunk = min((size_t)30, len - off);
    Wire.beginTransmission(i2c_addr);
    Wire.write(start >> 8);
    Wire.write(start & 0xFF);
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_addr, (uint8_t)chunk);
    for (size_t i = 0; i < chunk && Wire.available(); ++i)
      buf[off + i] = Wire.read();
    start += chunk;
    off   += chunk;
    delay(1);
  }
}

void dumpConfig(const uint8_t* buf, uint8_t checksum) {
  for (size_t i = 0; i < CFG_SIZE; ++i) {
    if (i % 16 == 0) Serial.printf("\n0x%04X: ", REG_CFG_START + i);
    Serial.printf("%02X ", buf[i]);
  }
  Serial.printf("\nChecksum reg: 0x%02X\n", checksum);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(PIN_RSTB, OUTPUT);
  digitalWrite(PIN_RSTB, HIGH);
  delay(50);

  // 1) Сканируем шину и выбираем адрес
  i2c_addr = scanI2C();
  if (!i2c_addr) {
    Serial.println("ERROR: No I2C device found!");
    while (1) delay(1000);
  }
  Serial.printf("I2C device at 0x%02X\n\n", i2c_addr);

  // 2) «До» — дамп конфига и checksum
  Serial.println("=== BEFORE UPDATE ===");
  readConfig(REG_CFG_START, before_cfg, CFG_SIZE);
  before_chk = readReg(i2c_addr, REG_CFG_CHKSUM);
  dumpConfig(before_cfg, before_chk);

  // 3) ESD-handshake: 0x8046 и 0x8040 → 0xAA
  writeReg(i2c_addr, REG_CMD_CHECK, 0xAA); delay(5);
  writeReg(i2c_addr, REG_ESD,       0xAA); delay(5);
  Serial.printf("\nESD HS read=0x%02X\n", readReg(i2c_addr, REG_ESD));

  // 4) Запись конфига чанками ≤30 байт
  Serial.println("\n=== WRITING CONFIG ===");
  writeBlock(REG_CFG_START, gt9271_cfg, CFG_SIZE);

  // 5) Считаем и пишем checksum
  uint16_t sum = 0;
  for (auto b : gt9271_cfg) sum += b;
  uint8_t chk = (~sum) + 1;
  Serial.printf("Writing checksum 0x%02X → ret=%d\n",
                chk, writeReg(i2c_addr, REG_CFG_CHKSUM, chk));

  // 6) Установка Config_Fresh
  Serial.printf("Writing REFRESH → ret=%d\n",
                writeReg(i2c_addr, REG_CFG_REFRESH, 1));

  // 7) Save-to-Flash & reboot командой 0x02
  Serial.printf("Save-to-Flash → ret=%d\n",
                writeReg(i2c_addr, REG_ESD, 2));
  delay(200);

  // 8) Hardware reset (опционально)
  digitalWrite(PIN_RSTB, LOW); delay(10); digitalWrite(PIN_RSTB, HIGH);
  Serial.println("\nPanel reset, waiting…");
  delay(100);

  // 9) «После» — новый дамп и checksum
  Serial.println("\n=== AFTER UPDATE ===");
  readConfig(REG_CFG_START, after_cfg, CFG_SIZE);
  after_chk = readReg(i2c_addr, REG_CFG_CHKSUM);
  dumpConfig(after_cfg, after_chk);
}

void loop() {
  // не используется
}