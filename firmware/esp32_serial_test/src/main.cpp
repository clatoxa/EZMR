// Testfirmware: prüft Build, Upload und serielle Ausgabe. Steuert keine Pins an.
#include <Arduino.h>

uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 Testfirmware gestartet");
}

void loop() {
  Serial.printf("Zaehler: %lu\n", (unsigned long)counter++);
  delay(1000);
}
