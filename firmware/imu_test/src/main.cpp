// Stufe 11f-a: IMU-Test (Modul "MPU-92.65", I2C 0x68) – Chipkennung, Gyroskop-Nullpunkt, Rauschen, Drift.
// Kein micro-ROS, keine Motorbefehle, kein Display. Ausgabe im seriellen Monitor (115200 Baud).
// Direkter Registerzugriff (keine Bibliothek), damit jeder Schritt nachvollziehbar ist.
// Registeradressen der MPU-6500/9250-Familie (Register Map des Herstellers):
//   0x6B PWR_MGMT_1, 0x1A CONFIG, 0x1B GYRO_CONFIG, 0x1C ACCEL_CONFIG,
//   0x3B ACCEL_XOUT_H … 0x48 GYRO_ZOUT_L (14 Byte am Stück), 0x75 WHO_AM_I
// Erwartete WHO_AM_I-Werte: 0x71 MPU-9250, 0x73 MPU-9255, 0x70 MPU-6500, 0x68 MPU-6050 (zur Einordnung).
//
// Befehle im Monitor:  c = Nullpunkt kalibrieren (Roboter 3 s NICHT bewegen)
//                      z = Winkel auf 0 setzen
//                      s = Statistik der letzten 5 s (Rauschen)
// Stand 09.10.2026: UNGETESTET.
#include <Arduino.h>
#include <Wire.h>

static const int     I2C_SDA = 21;           // Anleitung 4.2.3
static const int     I2C_SCL = 22;
static const uint8_t IMU_ADDR = 0x68;        // I2C-Scan 08.10.2026
static const float   GYRO_LSB_PER_DPS = 131.0f;   // Messbereich ±250 °/s (GYRO_CONFIG = 0)
static const float   ACCEL_LSB_PER_G = 16384.0f;  // Messbereich ±2 g (ACCEL_CONFIG = 0)
static const uint32_t SAMPLE_PERIOD_US = 10000;   // 100 Hz
static const uint32_t PRINT_PERIOD_MS = 500;

float gyroBiasZ = 0.0f;      // °/s
float angleZ = 0.0f;         // integrierter Winkel um die Hochachse [°]
uint32_t lastSampleUs = 0;
uint32_t lastPrintMs = 0;

// Statistik (Rauschen) über ein gleitendes 5-s-Fenster
static const int STAT_N = 500;
float statBuf[STAT_N];
int statIdx = 0;
int statCount = 0;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {    // Repeated Start
    return false;
  }
  if (Wire.requestFrom((int)IMU_ADDR, (int)n) != (int)n) {
    return false;
  }
  for (size_t i = 0; i < n; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

struct ImuRaw {
  int16_t ax, ay, az, temp, gx, gy, gz;
};

bool readImu(ImuRaw &r) {
  uint8_t b[14];
  if (!readRegs(0x3B, b, sizeof(b))) {
    return false;
  }
  r.ax   = (int16_t)((b[0] << 8) | b[1]);    // Big-Endian laut Datenblatt
  r.ay   = (int16_t)((b[2] << 8) | b[3]);
  r.az   = (int16_t)((b[4] << 8) | b[5]);
  r.temp = (int16_t)((b[6] << 8) | b[7]);
  r.gx   = (int16_t)((b[8] << 8) | b[9]);
  r.gy   = (int16_t)((b[10] << 8) | b[11]);
  r.gz   = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void calibrate() {
  Serial.println("Kalibrierung: Roboter 3 s NICHT bewegen ...");
  delay(500);
  double sum = 0.0;
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    ImuRaw r;
    if (readImu(r)) {
      sum += r.gz / GYRO_LSB_PER_DPS;
      n++;
    }
    delay(5);
  }
  if (n > 0) {
    gyroBiasZ = (float)(sum / n);
  }
  angleZ = 0.0f;
  statCount = statIdx = 0;
  Serial.printf("Nullpunkt gz = %+.4f °/s (n = %d)\n", gyroBiasZ, n);
}

void printStats() {
  if (statCount < 10) {
    Serial.println("Noch zu wenige Werte für die Statistik.");
    return;
  }
  double mean = 0.0;
  for (int i = 0; i < statCount; i++) mean += statBuf[i];
  mean /= statCount;
  double var = 0.0;
  float mn = statBuf[0], mx = statBuf[0];
  for (int i = 0; i < statCount; i++) {
    var += (statBuf[i] - mean) * (statBuf[i] - mean);
    mn = min(mn, statBuf[i]);
    mx = max(mx, statBuf[i]);
  }
  var /= (statCount - 1);
  Serial.printf("Statistik gz (korrigiert, n=%d): Mittel %+.4f °/s, Std %.4f °/s, min %+.3f, max %+.3f\n",
                statCount, mean, sqrt(var), mn, mx);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== IMU-Test (I2C 0x68) ===");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  uint8_t who = 0;
  if (!readRegs(0x75, &who, 1)) {
    Serial.println("FEHLER: keine Antwort von 0x68");
  } else {
    const char *name = who == 0x71 ? "MPU-9250" : who == 0x73 ? "MPU-9255" :
                       who == 0x70 ? "MPU-6500" : who == 0x68 ? "MPU-6050" : "unbekannt";
    Serial.printf("WHO_AM_I = 0x%02X (%s)\n", who, name);
  }

  bool ok = writeReg(0x6B, 0x00);            // aus dem Schlafmodus wecken, interner Takt
  delay(100);
  ok &= writeReg(0x6B, 0x01);                // Takt vom Gyroskop (PLL), stabiler
  ok &= writeReg(0x1A, 0x03);                // digitaler Tiefpass (DLPF_CFG = 3)
  ok &= writeReg(0x1B, 0x00);                // Gyroskop ±250 °/s
  ok &= writeReg(0x1C, 0x00);                // Beschleunigung ±2 g
  Serial.printf("Konfiguration %s\n", ok ? "ok" : "FEHLER");
  delay(100);

  calibrate();
  Serial.println("Ausgabe alle 0,5 s: gz roh [°/s] | gz korrigiert | Winkel z [°] | a [g] x y z");
  Serial.println("Befehle: c = kalibrieren, z = Winkel 0, s = Statistik");
  lastSampleUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  if (nowUs - lastSampleUs >= SAMPLE_PERIOD_US) {
    float dt = (nowUs - lastSampleUs) * 1e-6f;   // tatsächliche Zeit seit der letzten Messung
    lastSampleUs = nowUs;
    ImuRaw r;
    if (readImu(r)) {
      float gz = r.gz / GYRO_LSB_PER_DPS;
      float gzc = gz - gyroBiasZ;
      angleZ += gzc * dt;                     // Integration (Rechteckregel)
      statBuf[statIdx] = gzc;
      statIdx = (statIdx + 1) % STAT_N;
      if (statCount < STAT_N) statCount++;

      if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = millis();
        Serial.printf("gz %+8.3f | %+8.3f | Winkel %+8.2f | a %+.2f %+.2f %+.2f\n",
                      gz, gzc, angleZ, r.ax / ACCEL_LSB_PER_G, r.ay / ACCEL_LSB_PER_G,
                      r.az / ACCEL_LSB_PER_G);
      }
    } else {
      Serial.println("Lesefehler I2C");
    }
  }
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c') calibrate();
    else if (c == 'z') { angleZ = 0.0f; Serial.println("Winkel = 0"); }
    else if (c == 's') printStats();
  }
}
