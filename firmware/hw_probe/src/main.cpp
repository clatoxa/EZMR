// Hardware-Erkundung (vor Stufe 11): Bumper-Beschaltung und I2C-Geräte ermitteln.
// Kein micro-ROS, keine Motorbefehle. Ausgabe über den seriellen Monitor (115200 Baud).
//
// 1) Bumper an GPIO 12 und GPIO 14 (Anleitung 4.2.3, Tabelle 2):
//    Jeder Pin wird abwechselnd mit internem Pull-up und Pull-down gelesen.
//      PU=1 PD=0 -> Pin offen (nicht getrieben)
//      PU=1 PD=1 -> Pin wird von außen auf HIGH gezogen
//      PU=0 PD=0 -> Pin wird von außen auf LOW gezogen
//    Vergleich gedrückt / nicht gedrückt zeigt Polarität und Beschaltung.
//    Hinweis Anleitung: GPIO 12 ist ein Strapping-Pin. Ist der Bumper beim Start eingesteckt,
//    bootet der ESP32 nicht -> Stecker erst nach dem Start einstecken.
// 2) I2C-Scan an GPIO 21 (SDA) / GPIO 22 (SCL): listet antwortende Adressen
//    (OLED, APDS-9960, IMU teilen sich laut Anleitung diesen Bus).
//
// Befehle im Monitor: s = I2C-Scan wiederholen
// Stand 08.10.2026: UNGETESTET.
#include <Arduino.h>
#include <Wire.h>

static const int BUMPER_PINS[] = {12, 14};
static const int NUM_BUMPERS = sizeof(BUMPER_PINS) / sizeof(BUMPER_PINS[0]);
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;
static const uint32_t PRINT_PERIOD_MS = 500;

uint8_t lastState[NUM_BUMPERS];   // Bit 0 = PU, Bit 1 = PD
uint32_t lastPrintMs = 0;

uint8_t probePin(int pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(100);           // Pegel einschwingen lassen
  uint8_t pu = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN);
  delayMicroseconds(100);
  uint8_t pd = digitalRead(pin);
  return (uint8_t)(pu | (pd << 1));
}

const char *meaning(uint8_t s) {
  switch (s) {
    case 0b01: return "offen";
    case 0b11: return "HIGH getrieben";
    case 0b00: return "LOW getrieben";
    default:   return "unplausibel";   // PU=0, PD=1
  }
}

void printStates(const char *reason) {
  Serial.printf("[%7lu ms] %s:", (unsigned long)millis(), reason);
  for (int i = 0; i < NUM_BUMPERS; i++) {
    uint8_t s = lastState[i];
    Serial.printf("  GPIO%d PU=%d PD=%d (%s)", BUMPER_PINS[i], s & 1, (s >> 1) & 1, meaning(s));
  }
  Serial.println();
}

void i2cScan() {
  Serial.printf("I2C-Scan an SDA=GPIO%d, SCL=GPIO%d ...\n", I2C_SDA, I2C_SCL);
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Gerät antwortet auf 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("I2C-Scan fertig: %d Gerät(e)\n", found);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Hardware-Erkundung: Bumper + I2C ===");
  Wire.begin(I2C_SDA, I2C_SCL);
  i2cScan();
  for (int i = 0; i < NUM_BUMPERS; i++) {
    lastState[i] = probePin(BUMPER_PINS[i]);
  }
  printStates("Start");
  Serial.println("Bumper drücken/loslassen; Änderungen werden sofort ausgegeben. 's' = I2C-Scan.");
}

void loop() {
  bool changed = false;
  for (int i = 0; i < NUM_BUMPERS; i++) {
    uint8_t s = probePin(BUMPER_PINS[i]);
    if (s != lastState[i]) {
      lastState[i] = s;
      changed = true;
    }
  }
  uint32_t now = millis();
  if (changed) {
    printStates("ÄNDERUNG");
    lastPrintMs = now;
  } else if (now - lastPrintMs >= PRINT_PERIOD_MS * 10) {   // alle 5 s Lebenszeichen
    printStates("Stand");
    lastPrintMs = now;
  }
  while (Serial.available() > 0) {
    if (Serial.read() == 's') {
      i2cScan();
    }
  }
  delay(10);
}
